#include "tb45_sms.h"
#include "tb45_async_job.h"
#include "tb45_async_job_internal.h"
#include "tb45_delayable_retry.h"
#include "tb45_sms_event.h"
#include "tb45_sms_at_helper.h"

#if __has_include("work_queues.h")
#include "work_queues.h"
#else
extern struct k_work_q low_priority_wq;
#endif

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(tb45_sms, CONFIG_LOG_DEFAULT_LEVEL);

#define TB45_SMS_PHONE_MAX_LEN            CONFIG_APP_TB45_SMS_PHONE_MAX_LEN
#define TB45_SMS_TEXT_MAX_LEN             CONFIG_APP_TB45_SMS_TEXT_MAX_LEN
#define TB45_SMS_SEND_ATTEMPT_TIMEOUT_MS  CONFIG_APP_TB45_SMS_SEND_ATTEMPT_TIMEOUT_MS
#define TB45_SMS_SEND_MAX_RETRIES         CONFIG_APP_TB45_SMS_SEND_MAX_RETRIES
#define TB45_SMS_SEND_RETRY_DELAY_MS      CONFIG_APP_TB45_SMS_SEND_RETRY_DELAY_MS
#define TB45_SMS_INIT_WAIT_MS             CONFIG_APP_TB45_SMS_INIT_WAIT_MS
#define TB45_SMS_INIT_AT_TIMEOUT_MS       CONFIG_APP_TB45_SMS_INIT_AT_TIMEOUT_MS
#define TB45_SMS_RESULT_QUEUE_DEPTH       CONFIG_APP_TB45_SMS_RESULT_QUEUE_DEPTH
#define TB45_SMS_RX_TRIGGER_QUEUE_DEPTH   CONFIG_APP_TB45_SMS_RX_TRIGGER_QUEUE_DEPTH
#define TB45_SMS_RX_RESULT_QUEUE_DEPTH    CONFIG_APP_TB45_SMS_RX_RESULT_QUEUE_DEPTH
#define TB45_SMS_RX_SCAN_MAX_INDEXES      CONFIG_APP_TB45_SMS_RX_SCAN_MAX_INDEXES
#define TB45_SMS_RX_AUTO_DELETE           IS_ENABLED(CONFIG_APP_TB45_SMS_RX_AUTO_DELETE)
#define TB45_SMS_RX_STARTUP_CLEANUP       IS_ENABLED(CONFIG_APP_TB45_SMS_RX_STARTUP_CLEANUP)
#define TB45_SMS_RX_STORAGE               CONFIG_APP_TB45_SMS_RX_STORAGE

#define TB45_SMS_RX_CAPTURE_BUF_SIZE          768
#define TB45_SMS_RX_INIT_RETRY_DELAY_MS       2000U
#define TB45_SMS_RX_INIT_PIPE_WAIT_DELAY_MS   2000U
#define TB45_SMS_RX_INIT_BUSY_DELAY_MS        250U
#define TB45_SMS_RX_DELETE_RETRY_DELAY_MS     100U
#define TB45_SMS_RX_DELETE_MAX_ATTEMPTS       3U
#define TB45_SMS_RX_BUSY_LOG_INTERVAL         20U
#define TB45_SMS_SHELL_ASCII_CTRL_C       0x03U
#define TB45_SMS_SLEEP_SLICE_MS           50

struct tb45_sms_rx_trigger {
	uint16_t storage_index;
	uint8_t full_scan;
};

enum tb45_sms_rx_init_state {
	TB45_SMS_RX_INIT_STATE_IDLE = 0,
	TB45_SMS_RX_INIT_STATE_CMGF,
	TB45_SMS_RX_INIT_STATE_CSCS,
	TB45_SMS_RX_INIT_STATE_CPMS,
	TB45_SMS_RX_INIT_STATE_CNMI,
	TB45_SMS_RX_INIT_STATE_CLEANUP,
	TB45_SMS_RX_INIT_STATE_READY,
};

struct tb45_sms_rx_init_ctx {
	struct tb45_delayable_retry retry;
	struct k_work continue_work;
	enum tb45_sms_rx_init_state state;
	struct tb45_sms_rx_trigger pending_trigger;
	char cpms_cmd[48];
	int last_ret;
	bool pending_trigger_valid;
};

struct tb45_sms_rx_delete_ctx {
	struct tb45_delayable_retry retry;
	uint16_t storage_index;
	char cmd[24];
	int last_ret;
};

enum tb45_sms_rx_capture_state {
	TB45_SMS_RX_CAPTURE_STATE_IDLE = 0,
	TB45_SMS_RX_CAPTURE_STATE_CMGL,
};

struct tb45_sms_rx_capture_ctx {
	struct k_work continue_work;
	enum tb45_sms_rx_capture_state state;
	int last_ret;
	bool active;
};

K_SEM_DEFINE(tb45_sms_result_sem, 0, TB45_SMS_RESULT_QUEUE_DEPTH);
K_MUTEX_DEFINE(tb45_sms_result_lock);
static struct tb45_sms_result tb45_sms_results[TB45_SMS_RESULT_QUEUE_DEPTH] = {0};
static uint16_t tb45_sms_result_head = 0U;
static uint16_t tb45_sms_result_tail = 0U;
static uint16_t tb45_sms_result_count = 0U;
static atomic_t tb45_sms_request_id_seed = ATOMIC_INIT(1);
static atomic_t tb45_sms_rx_started = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_start_allowed = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_setup_done = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_cleanup_done = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_init_wait_logged = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_init_completed_logged = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_init_retry_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_processed_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_scan_fail_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_cmgr_fail_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_parse_fail_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_delete_fail_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_trigger_queue_drop_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_rx_result_queue_drop_count = ATOMIC_INIT(0);
static atomic_t tb45_sms_async_result_queue_drop_count = ATOMIC_INIT(0);
static char tb45_sms_rx_capture_buf[TB45_SMS_RX_CAPTURE_BUF_SIZE] = {0};

K_MSGQ_DEFINE(tb45_sms_rx_trigger_msgq, sizeof(struct tb45_sms_rx_trigger),
	      TB45_SMS_RX_TRIGGER_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tb45_sms_rx_result_msgq, sizeof(struct tb45_sms_rx_notice),
	      TB45_SMS_RX_RESULT_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tb45_sms_rx_delete_msgq, sizeof(uint16_t), TB45_SMS_RX_TRIGGER_QUEUE_DEPTH, 2);

static int tb45_sms_rx_enqueue_trigger(uint16_t storage_index, uint8_t full_scan);
static void tb45_sms_rx_reset_runtime_state(void);

static struct k_work_poll tb45_sms_rx_poll_work = {0};
static struct k_poll_event tb45_sms_rx_poll_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&tb45_sms_rx_trigger_msgq,
					0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&tb45_sms_rx_delete_msgq,
					0),
};
static struct tb45_sms_rx_init_ctx tb45_sms_rx_init_ctx = {0};
static struct tb45_sms_rx_delete_ctx tb45_sms_rx_delete_ctx = {0};
static struct tb45_sms_rx_capture_ctx tb45_sms_rx_capture_ctx = {0};
static atomic_t tb45_sms_rx_wq_not_ready_warned = ATOMIC_INIT(0);

static void tb45_sms_rx_reset_runtime_state(void)
{
	atomic_set(&tb45_sms_rx_setup_done, 0);
	atomic_set(&tb45_sms_rx_cleanup_done, 0);
	atomic_set(&tb45_sms_rx_init_wait_logged, 0);
	atomic_set(&tb45_sms_rx_init_completed_logged, 0);
	tb45_sms_rx_init_ctx.state = TB45_SMS_RX_INIT_STATE_IDLE;
	tb45_sms_rx_init_ctx.last_ret = 0;
	tb45_sms_rx_init_ctx.pending_trigger_valid = false;
	tb45_sms_rx_capture_ctx.state = TB45_SMS_RX_CAPTURE_STATE_IDLE;
	tb45_sms_rx_capture_ctx.last_ret = 0;
	tb45_sms_rx_capture_ctx.active = false;
}

static bool tb45_sms_rx_wq_ready(void)
{
	bool ready = k_work_queue_thread_get(&low_priority_wq) != NULL;

	if (ready) {
		atomic_set(&tb45_sms_rx_wq_not_ready_warned, 0);
	}

	return ready;
}

static int tb45_sms_rx_submit_poll(void)
{
	if (!tb45_sms_rx_wq_ready()) {
		if (atomic_cas(&tb45_sms_rx_wq_not_ready_warned, 0, 1)) {
			LOG_WRN("SMS_RCV queue not ready; waiting for work_queues_init()");
		}
		return -EAGAIN;
	}

	tb45_sms_rx_poll_events[0].state = K_POLL_STATE_NOT_READY;
	tb45_sms_rx_poll_events[1].state = K_POLL_STATE_NOT_READY;
	return k_work_poll_submit_to_queue(&low_priority_wq,
					   &tb45_sms_rx_poll_work,
					   tb45_sms_rx_poll_events,
					   ARRAY_SIZE(tb45_sms_rx_poll_events),
					   K_FOREVER);
}

static void tb45_sms_rx_resume_poll(void)
{
	int ret = tb45_sms_rx_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("SMS_RCV poll submit failed (%d)", ret);
	}
}

static bool tb45_sms_rx_init_is_ready(void)
{
	if (atomic_get(&tb45_sms_rx_setup_done) == 0) {
		return false;
	}

	if (TB45_SMS_RX_STARTUP_CLEANUP == 0) {
		return true;
	}

	return atomic_get(&tb45_sms_rx_cleanup_done) != 0;
}

static void tb45_sms_rx_init_merge_trigger(const struct tb45_sms_rx_trigger *trigger)
{
	struct tb45_sms_rx_trigger *pending = &tb45_sms_rx_init_ctx.pending_trigger;

	if (trigger == NULL) {
		return;
	}

	if (!tb45_sms_rx_init_ctx.pending_trigger_valid) {
		tb45_sms_rx_init_ctx.pending_trigger = *trigger;
		tb45_sms_rx_init_ctx.pending_trigger_valid = true;
		return;
	}

	if ((pending->full_scan != 0U) || (trigger->full_scan != 0U) ||
	    (pending->storage_index != trigger->storage_index)) {
		pending->storage_index = 0U;
		pending->full_scan = 1U;
	}
}

static void tb45_sms_rx_init_kick_pending_trigger(void)
{
	struct tb45_sms_rx_trigger trigger;

	if (!tb45_sms_rx_init_ctx.pending_trigger_valid) {
		return;
	}

	trigger = tb45_sms_rx_init_ctx.pending_trigger;
	tb45_sms_rx_init_ctx.pending_trigger_valid = false;
	(void)tb45_sms_rx_enqueue_trigger(trigger.storage_index, trigger.full_scan);
}

static int tb45_sms_rx_init_start_step(struct tb45_sms_rx_init_ctx *ctx);
static void tb45_sms_rx_init_continue_work_handler(struct k_work *work);

static void tb45_sms_rx_init_at_complete(int ret, void *user_data)
{
	struct tb45_sms_rx_init_ctx *ctx = user_data;
	int submit_ret;

	if (ctx == NULL) {
		return;
	}

	if (ret < 0) {
		ctx->last_ret = ret;
		(void)tb45_delayable_retry_complete(&ctx->retry, ret);
		return;
	}

	switch (ctx->state) {
	case TB45_SMS_RX_INIT_STATE_CMGF:
		ctx->state = TB45_SMS_RX_INIT_STATE_CSCS;
		break;
	case TB45_SMS_RX_INIT_STATE_CSCS:
		ctx->state = TB45_SMS_RX_INIT_STATE_CPMS;
		break;
	case TB45_SMS_RX_INIT_STATE_CPMS:
		ctx->state = TB45_SMS_RX_INIT_STATE_CNMI;
		break;
	case TB45_SMS_RX_INIT_STATE_CNMI:
		atomic_set(&tb45_sms_rx_setup_done, 1);
		ctx->state = (TB45_SMS_RX_STARTUP_CLEANUP != 0) ?
			TB45_SMS_RX_INIT_STATE_CLEANUP : TB45_SMS_RX_INIT_STATE_READY;
		break;
	case TB45_SMS_RX_INIT_STATE_CLEANUP:
		atomic_set(&tb45_sms_rx_cleanup_done, 1);
		ctx->state = TB45_SMS_RX_INIT_STATE_READY;
		break;
	case TB45_SMS_RX_INIT_STATE_IDLE:
	case TB45_SMS_RX_INIT_STATE_READY:
	default:
		break;
	}

	if (ctx->state == TB45_SMS_RX_INIT_STATE_READY) {
		(void)tb45_delayable_retry_complete(&ctx->retry, 0);
		return;
	}

	submit_ret = k_work_submit_to_queue(&low_priority_wq, &ctx->continue_work);
	if (submit_ret < 0) {
		ctx->last_ret = submit_ret;
		(void)tb45_delayable_retry_complete(&ctx->retry, submit_ret);
	}
}
static void tb45_sms_rx_init_continue_work_handler(struct k_work *work)
{
	struct tb45_sms_rx_init_ctx *ctx = CONTAINER_OF(work, struct tb45_sms_rx_init_ctx,
							continue_work);
	int start_ret;

	if (ctx == NULL) {
		return;
	}

	start_ret = tb45_sms_rx_init_start_step(ctx);
	if (start_ret != -EINPROGRESS) {
		ctx->last_ret = start_ret;
		(void)tb45_delayable_retry_complete(&ctx->retry, start_ret);
	}
}


static int tb45_sms_rx_init_start_step(struct tb45_sms_rx_init_ctx *ctx)
{
	const char *cmd = NULL;
	int ret;

	if (ctx == NULL) {
		return -EINVAL;
	}

	switch (ctx->state) {
	case TB45_SMS_RX_INIT_STATE_CMGF:
		cmd = "AT+CMGF=1";
		break;
	case TB45_SMS_RX_INIT_STATE_CSCS:
		cmd = "AT+CSCS=\"IRA\"";
		break;
	case TB45_SMS_RX_INIT_STATE_CPMS:
		ret = snprintf(ctx->cpms_cmd, sizeof(ctx->cpms_cmd),
			      "AT+CPMS=\"%s\",\"%s\",\"%s\"", TB45_SMS_RX_STORAGE,
			      TB45_SMS_RX_STORAGE, TB45_SMS_RX_STORAGE);
		if ((ret < 0) || ((size_t)ret >= sizeof(ctx->cpms_cmd))) {
			return -EINVAL;
		}
		cmd = ctx->cpms_cmd;
		break;
	case TB45_SMS_RX_INIT_STATE_CNMI:
		cmd = "AT+CNMI=2,1,0,0,0";
		break;
	case TB45_SMS_RX_INIT_STATE_CLEANUP:
		cmd = "AT+CMGD=1,4";
		break;
	case TB45_SMS_RX_INIT_STATE_READY:
		return 0;
	case TB45_SMS_RX_INIT_STATE_IDLE:
	default:
		return -EINVAL;
	}

	ret = tb45_sms_at_run_async_cb(NULL, cmd, "OK", TB45_SMS_INIT_AT_TIMEOUT_MS,
					tb45_sms_rx_init_at_complete, ctx);
	if (ret < 0) {
		ctx->last_ret = ret;
		return ret;
	}

	return -EINPROGRESS;
}

static int tb45_sms_rx_init_run_attempt(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	struct tb45_sms_rx_init_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_init_ctx, retry);

	ARG_UNUSED(attempt);

	if (ctx == NULL) {
		return -EINVAL;
	}

	if (tb45_sms_rx_init_is_ready()) {
		ctx->state = TB45_SMS_RX_INIT_STATE_READY;
		return 0;
	}

	ctx->state = (atomic_get(&tb45_sms_rx_setup_done) == 0) ?
		TB45_SMS_RX_INIT_STATE_CMGF : TB45_SMS_RX_INIT_STATE_CLEANUP;
	return tb45_sms_rx_init_start_step(ctx);
}

static uint32_t tb45_sms_rx_init_retry_delay_ms(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	struct tb45_sms_rx_init_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_init_ctx, retry);

	ARG_UNUSED(attempt);
	if (ctx == NULL) {
		return TB45_SMS_RX_INIT_RETRY_DELAY_MS;
	}

	if (ctx->last_ret == -EPERM) {
		return TB45_SMS_RX_INIT_PIPE_WAIT_DELAY_MS;
	}

	if (ctx->last_ret == -EBUSY) {
		return TB45_SMS_RX_INIT_BUSY_DELAY_MS;
	}

	return TB45_SMS_RX_INIT_RETRY_DELAY_MS;
}

static void tb45_sms_rx_init_attempt_failed(struct tb45_delayable_retry *retry, int ret, uint8_t attempt,
				    uint8_t max_attempts)
{
	struct tb45_sms_rx_init_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_init_ctx, retry);
	uint32_t retry_count;
	uint32_t retry_display;
	bool rate_limited;

	if (ctx != NULL) {
		ctx->last_ret = ret;
	}

	atomic_inc(&tb45_sms_rx_init_retry_count);
	retry_count = (uint32_t)atomic_get(&tb45_sms_rx_init_retry_count);
	retry_display = (max_attempts == 0U) ? retry_count : (uint32_t)attempt;
	rate_limited = ((ret == -EBUSY) || (ret == -EPERM)) &&
		((retry_count % TB45_SMS_RX_BUSY_LOG_INTERVAL) != 0U);
	if (rate_limited) {
		return;
	}

	if ((ctx != NULL) && (ctx->state == TB45_SMS_RX_INIT_STATE_CLEANUP)) {
		LOG_WRN("SMS_RCV startup cleanup pending (%d), retry %u%s", ret, retry_display,
			(max_attempts == 0U) ? "/inf" : "");
	} else {
		LOG_WRN("SMS_RCV setup pending (%d), retry %u%s", ret, retry_display,
			(max_attempts == 0U) ? "/inf" : "");
	}
}

static void tb45_sms_rx_init_complete(struct tb45_delayable_retry *retry, int ret)
{
	ARG_UNUSED(retry);

	if (ret == 0) {
		if (atomic_cas(&tb45_sms_rx_init_completed_logged, 0, 1)) {
			LOG_INF("SMS_RCV initialization...completed");
		}
		tb45_sms_rx_init_kick_pending_trigger();
		return;
	}

	LOG_ERR("SMS_RCV initialization failed (%d)", ret);
}

static const struct tb45_delayable_retry_ops tb45_sms_rx_init_retry_ops = {
	.run_attempt = tb45_sms_rx_init_run_attempt,
	.retry_delay_ms = tb45_sms_rx_init_retry_delay_ms,
	.attempt_failed = tb45_sms_rx_init_attempt_failed,
	.complete = tb45_sms_rx_init_complete,
};

static void tb45_sms_rx_delete_at_complete(int ret, void *user_data)
{
	struct tb45_sms_rx_delete_ctx *ctx = user_data;

	if (ctx == NULL) {
		return;
	}

	ctx->last_ret = ret;
	(void)tb45_delayable_retry_complete(&ctx->retry, ret);
}

static int tb45_sms_rx_delete_run_attempt(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	struct tb45_sms_rx_delete_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_delete_ctx, retry);
	int ret;

	ARG_UNUSED(attempt);

	if ((ctx == NULL) || (ctx->storage_index == 0U)) {
		return -EINVAL;
	}

	ret = snprintf(ctx->cmd, sizeof(ctx->cmd), "AT+CMGD=%u", ctx->storage_index);
	if ((ret < 0) || ((size_t)ret >= sizeof(ctx->cmd))) {
		return -EINVAL;
	}

	ret = tb45_sms_at_run_async_cb(NULL, ctx->cmd, "OK", TB45_SMS_INIT_AT_TIMEOUT_MS,
					tb45_sms_rx_delete_at_complete, ctx);
	if (ret < 0) {
		ctx->last_ret = ret;
		return ret;
	}

	return -EINPROGRESS;
}

static uint32_t tb45_sms_rx_delete_retry_delay_ms(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	ARG_UNUSED(retry);
	ARG_UNUSED(attempt);
	return TB45_SMS_RX_DELETE_RETRY_DELAY_MS;
}

static void tb45_sms_rx_delete_attempt_failed(struct tb45_delayable_retry *retry, int ret, uint8_t attempt,
				      uint8_t max_attempts)
{
	struct tb45_sms_rx_delete_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_delete_ctx, retry);

	if ((ctx == NULL) || (ctx->storage_index == 0U)) {
		return;
	}

	ctx->last_ret = ret;
	if ((max_attempts != 0U) && (attempt < max_attempts)) {
		return;
	}

	atomic_inc(&tb45_sms_rx_delete_fail_count);
	LOG_WRN("SMS_RCV delete failed idx=%u (%d)", ctx->storage_index, ret);
}

static void tb45_sms_rx_delete_complete(struct tb45_delayable_retry *retry, int ret)
{
	struct tb45_sms_rx_delete_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_sms_rx_delete_ctx, retry);

	ARG_UNUSED(ret);
	if (ctx == NULL) {
		return;
	}

	ctx->storage_index = 0U;
	tb45_sms_rx_resume_poll();
}

static const struct tb45_delayable_retry_ops tb45_sms_rx_delete_retry_ops = {
	.run_attempt = tb45_sms_rx_delete_run_attempt,
	.retry_delay_ms = tb45_sms_rx_delete_retry_delay_ms,
	.attempt_failed = tb45_sms_rx_delete_attempt_failed,
	.complete = tb45_sms_rx_delete_complete,
};

static void tb45_sms_rx_delete_kick(void)
{
	uint16_t storage_index;
	int ret;

	if (tb45_delayable_retry_is_pending(&tb45_sms_rx_delete_ctx.retry)) {
		return;
	}

	ret = k_msgq_get(&tb45_sms_rx_delete_msgq, &storage_index, K_NO_WAIT);
	if (ret == -ENOMSG) {
		return;
	}

	if (ret < 0) {
		LOG_ERR("SMS_RCV delete queue read failed (%d)", ret);
		return;
	}

	tb45_sms_rx_delete_ctx.storage_index = storage_index;
	(void)tb45_delayable_retry_start(&tb45_sms_rx_delete_ctx.retry,
					 TB45_SMS_RX_DELETE_MAX_ATTEMPTS);
}

static int tb45_sms_rx_enqueue_delete(uint16_t storage_index)
{
	int ret = k_msgq_put(&tb45_sms_rx_delete_msgq, &storage_index, K_NO_WAIT);

	if (ret == 0) {
		(void)tb45_sms_rx_submit_poll();
	}

	return ret;
}

static void tb45_sms_print_usage(const struct shell *sh)
{
#if defined(CONFIG_SHELL)
	if (sh != NULL) {
		shell_print(sh, "Usage: tb45 sms send <phone> <text>");
		shell_print(sh, "Example: tb45 sms send \"+886123456789\" \"hello\"");
		return;
	}
#endif

	LOG_WRN("Usage: tb45 sms send <phone> <text>\r\n"
		"Example: tb45 sms send \"+886123456789\" \"hello\"");
}

#if defined(CONFIG_SHELL)
static bool tb45_sms_send_cancel_pending(const struct shell *sh)
{
	if ((sh == NULL) || (sh->iface == NULL) || (sh->iface->api == NULL) ||
	    (sh->iface->api->read == NULL)) {
		return false;
	}

	uint8_t rx_buf[16];

	while (true) {
		size_t cnt = 0U;
		int read_ret = sh->iface->api->read(sh->iface, rx_buf, sizeof(rx_buf), &cnt);
		if ((read_ret < 0) || (cnt == 0U)) {
			break;
		}

		for (size_t i = 0U; i < cnt; i++) {
			if (rx_buf[i] == TB45_SMS_SHELL_ASCII_CTRL_C) {
				return true;
			}
		}
	}

	return false;
}

static int tb45_sms_send_sleep_interruptible(const struct shell *sh, int delay_ms)
{
	int elapsed_ms = 0;

	while (elapsed_ms < delay_ms) {
		if (tb45_sms_send_cancel_pending(sh)) {
			return -ECANCELED;
		}

		int sleep_ms = delay_ms - elapsed_ms;
		if (sleep_ms > TB45_SMS_SLEEP_SLICE_MS) {
			sleep_ms = TB45_SMS_SLEEP_SLICE_MS;
		}

		k_msleep(sleep_ms);
		elapsed_ms += sleep_ms;
	}

	if (tb45_sms_send_cancel_pending(sh)) {
		return -ECANCELED;
	}

	return 0;
}
#endif

static bool tb45_sms_phone_is_valid(const char *phone)
{
	size_t i = 0;
	size_t len;

	if (phone == NULL) {
		return false;
	}

	len = strlen(phone);
	if ((len < 3U) || (len > TB45_SMS_PHONE_MAX_LEN)) {
		return false;
	}

	if (phone[0] == '+') {
		if (len < 4U) {
			return false;
		}
		i = 1U;
	}

	for (; i < len; i++) {
		if (!isdigit((unsigned char)phone[i])) {
			return false;
		}
	}

	return true;
}

static bool tb45_sms_text_is_valid(const char *text, size_t *out_len)
{
	size_t i;
	size_t len;

	if (text == NULL) {
		return false;
	}

	len = strlen(text);
	if ((len == 0U) || (len > TB45_SMS_TEXT_MAX_LEN)) {
		return false;
	}

	for (i = 0U; i < len; i++) {
		unsigned char ch = (unsigned char)text[i];

		if ((ch < 0x20U) || (ch == 0x7FU)) {
			return false;
		}
	}

	if (out_len != NULL) {
		*out_len = len;
	}

	return true;
}

static uint32_t tb45_sms_next_request_id(void)
{
	uint32_t request_id = (uint32_t)atomic_inc(&tb45_sms_request_id_seed);

	if (request_id == 0U) {
		request_id = (uint32_t)atomic_inc(&tb45_sms_request_id_seed);
	}

	return request_id;
}

static void tb45_sms_store_async_result(uint32_t request_id, const char *phone, int result)
{
	struct tb45_sms_result entry;
	bool queue_was_full;

	entry.request_id = request_id;
	entry.result = result;
	(void)snprintf(entry.phone, sizeof(entry.phone), "%s", phone != NULL ? phone : "");

	k_mutex_lock(&tb45_sms_result_lock, K_FOREVER);
	queue_was_full = (tb45_sms_result_count == TB45_SMS_RESULT_QUEUE_DEPTH);

	if (queue_was_full) {
		tb45_sms_result_head = (uint16_t)((tb45_sms_result_head + 1U) %
					     TB45_SMS_RESULT_QUEUE_DEPTH);
		tb45_sms_result_count--;
	}

	tb45_sms_results[tb45_sms_result_tail] = entry;
	tb45_sms_result_tail = (uint16_t)((tb45_sms_result_tail + 1U) % TB45_SMS_RESULT_QUEUE_DEPTH);
	tb45_sms_result_count++;
	k_mutex_unlock(&tb45_sms_result_lock);

	if (!queue_was_full) {
		k_sem_give(&tb45_sms_result_sem);
	} else {
		atomic_inc(&tb45_sms_async_result_queue_drop_count);
		LOG_WRN("SMS async result queue full; dropped oldest result");
	}
}

static int tb45_sms_rx_enqueue_trigger(uint16_t storage_index, uint8_t full_scan)
{
	struct tb45_sms_rx_trigger trigger = {
		.storage_index = storage_index,
		.full_scan = full_scan,
	};
	struct tb45_sms_rx_trigger dropped;

	if (atomic_get(&tb45_sms_rx_start_allowed) == 0) {
		return -EAGAIN;
	}

	int ret = k_msgq_put(&tb45_sms_rx_trigger_msgq, &trigger, K_NO_WAIT);

	if (ret == 0) {
		(void)tb45_sms_rx_submit_poll();
		return 0;
	}

	if (ret != -ENOMSG) {
		return ret;
	}

	/* Keep newest trigger when queue is full. */
	atomic_inc(&tb45_sms_rx_trigger_queue_drop_count);
	(void)k_msgq_get(&tb45_sms_rx_trigger_msgq, &dropped, K_NO_WAIT);
	ret = k_msgq_put(&tb45_sms_rx_trigger_msgq, &trigger, K_NO_WAIT);
	if (ret == 0) {
		(void)tb45_sms_rx_submit_poll();
	}

	return ret;
}

static void tb45_sms_rx_publish_notice(uint16_t storage_index, int status)
{
	struct tb45_sms_rx_notice notice = {
		.storage_index = storage_index,
		.status = status,
	};
	struct tb45_sms_rx_notice dropped;
	int ret = k_msgq_put(&tb45_sms_rx_result_msgq, &notice, K_NO_WAIT);

	if (ret == 0) {
		tb45_sms_event_notify_rx_message(storage_index, status);
		return;
	}

	if (ret != -ENOMSG) {
		LOG_WRN("SMS_RCV queue put failed (%d)", ret);
		return;
	}

	atomic_inc(&tb45_sms_rx_result_queue_drop_count);
	(void)k_msgq_get(&tb45_sms_rx_result_msgq, &dropped, K_NO_WAIT);
	ret = k_msgq_put(&tb45_sms_rx_result_msgq, &notice, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN("SMS_RCV queue overflow; dropped notice idx=%u", storage_index);
		return;
	}

	tb45_sms_event_notify_rx_message(storage_index, status);
}

static bool tb45_sms_parse_u16(const char *str, uint16_t *out_value)
{
	char *endp;
	unsigned long parsed;

	if ((str == NULL) || (out_value == NULL)) {
		return false;
	}

	errno = 0;
	parsed = strtoul(str, &endp, 10);
	if ((errno != 0) || (endp == str) || (*endp != '\0') || (parsed > 0xFFFFUL)) {
		return false;
	}

	*out_value = (uint16_t)parsed;
	return true;
}

static bool tb45_sms_extract_first_u16(const char *str, uint16_t *out_value)
{
	const char *p = str;
	char digits[6];
	size_t n = 0U;

	if ((str == NULL) || (out_value == NULL)) {
		return false;
	}

	while (*p != '\0') {
		if (isdigit((unsigned char)*p) == 0) {
			p++;
			continue;
		}

		n = 0U;
		while ((*p != '\0') && (isdigit((unsigned char)*p) != 0) && (n < sizeof(digits) - 1U)) {
			digits[n++] = *p++;
		}
		digits[n] = '\0';
		return tb45_sms_parse_u16(digits, out_value);
	}

	return false;
}

static bool tb45_sms_extract_quoted_field(const char *line, uint8_t wanted_idx, char *out,
					  size_t out_size)
{
	const char *p = line;
	uint8_t qidx = 0U;

	if ((line == NULL) || (out == NULL) || (out_size == 0U)) {
		return false;
	}

	out[0] = '\0';

	while (*p != '\0') {
		const char *start;
		const char *end;
		size_t len;

		if (*p != '"') {
			p++;
			continue;
		}

		start = ++p;
		end = strchr(start, '"');
		if (end == NULL) {
			break;
		}

		if (qidx == wanted_idx) {
			len = (size_t)(end - start);
			if (len >= out_size) {
				len = out_size - 1U;
			}
			memcpy(out, start, len);
			out[len] = '\0';
			return true;
		}

		qidx++;
		p = end + 1;
	}

	return false;
}

static void tb45_sms_format_rx_timestamp(const char *cmgr_time, char *out, size_t out_size)
{
	int yy = 0;
	int mm = 0;
	int dd = 0;
	int hh = 0;
	int min = 0;
	int sec = 0;
	unsigned int year4;
	unsigned int mm2;
	unsigned int dd2;
	unsigned int hh2;
	unsigned int min2;
	unsigned int sec2;

	if ((out == NULL) || (out_size == 0U)) {
		return;
	}

	out[0] = '\0';
	if (cmgr_time == NULL) {
		return;
	}

	if (sscanf(cmgr_time, "%2d/%2d/%2d,%2d:%2d:%2d", &yy, &mm, &dd, &hh, &min, &sec) == 6 &&
	    (mm >= 1) && (mm <= 12)) {
		year4 = 2000U + (unsigned int)(yy & 0xFF);
		mm2 = (unsigned int)((mm < 0) ? 0 : (mm > 99 ? 99 : mm));
		dd2 = (unsigned int)((dd < 0) ? 0 : (dd > 99 ? 99 : dd));
		hh2 = (unsigned int)((hh < 0) ? 0 : (hh > 99 ? 99 : hh));
		min2 = (unsigned int)((min < 0) ? 0 : (min > 99 ? 99 : min));
		sec2 = (unsigned int)((sec < 0) ? 0 : (sec > 99 ? 99 : sec));
		(void)snprintf(out, out_size, "%04u/%02u/%02u %02u:%02u:%02u",
			       year4, mm2, dd2, hh2, min2, sec2);
		return;
	}

	(void)snprintf(out, out_size, "%.*s", (int)(out_size - 1U), cmgr_time);
	return;
}

static int tb45_sms_parse_cmgr_response(uint16_t storage_index, const char *response,
					struct tb45_sms_rx_message *out_message)
{
	const char *cmgr_line;
	const char *header_end;
	const char *msg_start;
	const char *msg_end;
	char header[192];
	char cmgr_timestamp[32];
	size_t header_len;
	size_t msg_len;

	if ((response == NULL) || (out_message == NULL)) {
		return -EINVAL;
	}

	cmgr_line = strstr(response, "+CMGR:");
	if (cmgr_line == NULL) {
		return -EIO;
	}

	header_end = strstr(cmgr_line, "\r\n");
	if (header_end == NULL) {
		header_end = strchr(cmgr_line, '\n');
	}
	if (header_end == NULL) {
		return -EIO;
	}

	header_len = (size_t)(header_end - cmgr_line);
	if (header_len >= sizeof(header)) {
		header_len = sizeof(header) - 1U;
	}
	memcpy(header, cmgr_line, header_len);
	header[header_len] = '\0';

	msg_start = header_end;
	while ((*msg_start == '\r') || (*msg_start == '\n')) {
		msg_start++;
	}
	msg_end = strstr(msg_start, "\r\n");
	if (msg_end == NULL) {
		msg_end = strchr(msg_start, '\n');
	}
	if (msg_end == NULL) {
		msg_end = msg_start + strlen(msg_start);
	}

	memset(out_message, 0, sizeof(*out_message));
	out_message->storage_index = storage_index;
	(void)tb45_sms_extract_quoted_field(header, 1U, out_message->phone, sizeof(out_message->phone));
	(void)tb45_sms_extract_quoted_field(header, 3U, cmgr_timestamp, sizeof(cmgr_timestamp));
	tb45_sms_format_rx_timestamp(cmgr_timestamp, out_message->timestamp, sizeof(out_message->timestamp));

	msg_len = (size_t)(msg_end - msg_start);
	if (msg_len >= sizeof(out_message->message)) {
		msg_len = sizeof(out_message->message) - 1U;
	}
	memcpy(out_message->message, msg_start, msg_len);
	out_message->message[msg_len] = '\0';

	return 0;
}

static size_t tb45_sms_parse_cmgl_unread_indexes(const char *response, uint16_t *indexes, size_t max_indexes)
{
	const char *p = response;
	size_t count = 0U;

	if ((response == NULL) || (indexes == NULL) || (max_indexes == 0U)) {
		return 0U;
	}

	while ((p = strstr(p, "+CMGL:")) != NULL) {
		uint16_t index_value = 0U;
		const char *line_end = strchr(p, '\n');
		char line[96];
		size_t line_len;

		if (line_end == NULL) {
			line_end = p + strlen(p);
		}
		line_len = (size_t)(line_end - p);
		if (line_len >= sizeof(line)) {
			line_len = sizeof(line) - 1U;
		}
		memcpy(line, p, line_len);
		line[line_len] = '\0';

		if (tb45_sms_extract_first_u16(line, &index_value)) {
			indexes[count++] = index_value;
			if (count >= max_indexes) {
				break;
			}
		}

		p = line_end;
	}

	return count;
}


/*
 * job: destination async job to populate
 * type: async SMS mode to enqueue
 * request: SMS payload and metadata to send
 * completion_ctx: optional wait context for TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT
 */
static int tb45_sms_build_enqueue_job(struct tb45_async_job *job,
				      enum tb45_async_job_type type,
				      const struct tb45_sms_request *request,
				      void *completion_ctx)
{
	struct tb45_sms_request *job_request;
	size_t phone_len;
	size_t text_len;

	if (job == NULL) {
		return -EINVAL;
	}

	if (request == NULL) {
		return -EINVAL;
	}

	if (!tb45_sms_phone_is_valid(request->phone_number)) {
		return -EINVAL;
	}

	if (!tb45_sms_text_is_valid(request->message, &text_len)) {
		return -EINVAL;
	}

	phone_len = strlen(request->phone_number);
	job->type = type;

	switch (type) {
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT:
		job_request = &job->payload.sms_send.request;
		break;
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT:
		job->payload.sms_send_wait.completion_ctx = completion_ctx;
		job_request = &job->payload.sms_send_wait.request;
		break;
	default:
		return -EINVAL;
	}

	memcpy(job_request->phone_number, request->phone_number, phone_len + 1);
	memcpy(job_request->message, request->message, text_len + 1);
	job_request->message_id = request->message_id;

	return 0;
}

static const struct tb45_sms_request *tb45_sms_async_job_request(const struct tb45_async_job *job)
{
	if (job == NULL) {
		return NULL;
	}

	switch (job->type) {
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT:
		return &job->payload.sms_send.request;
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT:
		return &job->payload.sms_send_wait.request;
	default:
		return NULL;
	}
}

static void *tb45_sms_async_job_completion_ctx(const struct tb45_async_job *job)
{
	if ((job == NULL) || (job->type != TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT)) {
		return NULL;
	}

	return job->payload.sms_send_wait.completion_ctx;
}

static void tb45_sms_async_dispatch_at_complete(int ret, void *user_data)
{
	ARG_UNUSED(user_data);
	tb45_async_dispatcher_complete_attempt(ret);
}

static int tb45_sms_async_dispatch(const struct tb45_async_job *job)
{
	char cmgs_cmd[48];
	const struct tb45_sms_request *request;
	int ret;

	request = tb45_sms_async_job_request(job);
	if (request == NULL) {
		return -EINVAL;
	}

	ret = snprintf(cmgs_cmd, sizeof(cmgs_cmd), "AT+CMGS=\"%s\"", request->phone_number);
	if ((ret < 0) || ((size_t)ret >= sizeof(cmgs_cmd))) {
		return -EINVAL;
	}

	LOG_DBG("SMS async dispatch: sending to %s", request->phone_number);
	ret = tb45_sms_at_send_text_raw_async_cb(cmgs_cmd,
		request->message,
		TB45_SMS_SEND_ATTEMPT_TIMEOUT_MS,
		tb45_sms_async_dispatch_at_complete, NULL);
	if (ret < 0) {
		return ret;
	}

	return -EINPROGRESS;
}

static void tb45_sms_async_complete(const struct tb45_async_job *job, int ret)
{
	const struct tb45_sms_request *request = tb45_sms_async_job_request(job);

	if (request == NULL) {
		return;
	}

	if (job->type == TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT) {
		tb45_sms_store_async_result(request->message_id, request->phone_number, ret);
	}

	tb45_async_job_complete_wait_ctx(tb45_sms_async_job_completion_ctx(job), ret);

	if (ret == 0) {
		LOG_DBG("SMS async dispatch: sent successfully");
	} else {
		LOG_ERR("SMS async dispatch: send failed (%d)", ret);
	}
}

static int tb45_sms_async_register_handler(void)
{
	int ret;

	ret = tb45_async_job_register_handler(TB45_ASYNC_JOB_TYPE_SMS_SEND,
					      tb45_sms_async_dispatch,
					      tb45_sms_async_complete);
	if (ret < 0) {
		LOG_ERR("SMS async register failed (%d)", ret);
		return 0;
	}

	ret = tb45_async_job_register_handler(TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT,
					      tb45_sms_async_dispatch,
					      tb45_sms_async_complete);
	if (ret < 0) {
		LOG_ERR("SMS async register failed (%d)", ret);
		return 0;
	}

	ret = tb45_async_job_register_handler(TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT,
					      tb45_sms_async_dispatch,
					      tb45_sms_async_complete);
	if (ret < 0) {
		LOG_ERR("SMS async register failed (%d)", ret);
	}

	return 0;
}

SYS_INIT(tb45_sms_async_register_handler, POST_KERNEL, 99);

static void tb45_sms_rx_capture_finish(void)
{
	struct tb45_sms_rx_capture_ctx *ctx = &tb45_sms_rx_capture_ctx;

	ctx->state = TB45_SMS_RX_CAPTURE_STATE_IDLE;
	ctx->last_ret = 0;
	ctx->active = false;
	tb45_sms_rx_resume_poll();
}

static void tb45_sms_rx_capture_at_complete(int ret, void *user_data)
{
	struct tb45_sms_rx_capture_ctx *ctx = user_data;
	int submit_ret;

	if (ctx == NULL) {
		return;
	}

	ctx->last_ret = ret;
	submit_ret = k_work_submit_to_queue(&low_priority_wq, &ctx->continue_work);
	if (submit_ret < 0) {
		if (ctx->state == TB45_SMS_RX_CAPTURE_STATE_CMGL) {
			atomic_inc(&tb45_sms_rx_scan_fail_count);
			tb45_sms_rx_publish_notice(0U, (ret < 0) ? ret : submit_ret);
		}
		tb45_sms_rx_capture_finish();
	}
}

static void tb45_sms_rx_capture_continue_work_handler(struct k_work *work)
{
	struct tb45_sms_rx_capture_ctx *ctx = CONTAINER_OF(work, struct tb45_sms_rx_capture_ctx,
							continue_work);
	uint16_t indexes[TB45_SMS_RX_SCAN_MAX_INDEXES];
	size_t index_count;
	size_t i;

	if (ctx == NULL) {
		return;
	}

	switch (ctx->state) {
	case TB45_SMS_RX_CAPTURE_STATE_CMGL:
		if (ctx->last_ret < 0) {
			atomic_inc(&tb45_sms_rx_scan_fail_count);
			tb45_sms_rx_publish_notice(0U, ctx->last_ret);
			tb45_sms_rx_capture_finish();
			return;
		}

		index_count = tb45_sms_parse_cmgl_unread_indexes(tb45_sms_rx_capture_buf,
			indexes, ARRAY_SIZE(indexes));
		for (i = 0U; i < index_count; i++) {
			atomic_inc(&tb45_sms_rx_processed_count);
			tb45_sms_rx_publish_notice(indexes[i], 0);
		}

		tb45_sms_rx_capture_finish();
		return;
	case TB45_SMS_RX_CAPTURE_STATE_IDLE:
	default:
		tb45_sms_rx_capture_finish();
		return;
	}
}

static int tb45_sms_rx_capture_start_cmgl(void)
{
	struct tb45_sms_rx_capture_ctx *ctx = &tb45_sms_rx_capture_ctx;
	int ret;

	if (ctx->active) {
		return -EBUSY;
	}

	ctx->state = TB45_SMS_RX_CAPTURE_STATE_CMGL;
	ctx->last_ret = 0;
	ret = tb45_sms_at_run_async_capture_cb(NULL, "AT+CMGL=\"REC UNREAD\"", "OK",
		TB45_SMS_INIT_AT_TIMEOUT_MS, tb45_sms_rx_capture_buf,
		sizeof(tb45_sms_rx_capture_buf), tb45_sms_rx_capture_at_complete, ctx);
	if (ret < 0) {
		ctx->state = TB45_SMS_RX_CAPTURE_STATE_IDLE;
		atomic_inc(&tb45_sms_rx_scan_fail_count);
		return ret;
	}

	ctx->active = true;
	return 0;
}

static int tb45_sms_rx_process_index(uint16_t storage_index, char *capture_buf, size_t capture_buf_size)
{
	ARG_UNUSED(capture_buf);
	ARG_UNUSED(capture_buf_size);

	if (storage_index == 0U) {
		return -EINVAL;
	}

	atomic_inc(&tb45_sms_rx_processed_count);
	tb45_sms_rx_publish_notice(storage_index, 0);
	return 0;
}

static int tb45_sms_rx_scan_unread(char *capture_buf, size_t capture_buf_size)
{
	ARG_UNUSED(capture_buf);
	ARG_UNUSED(capture_buf_size);

	return tb45_sms_rx_capture_start_cmgl();
}

static void tb45_sms_rx_process_trigger(const struct tb45_sms_rx_trigger *trigger)
{
	if (trigger == NULL) {
		return;
	}

	if ((atomic_get(&tb45_sms_rx_setup_done) == 0) &&
	    atomic_cas(&tb45_sms_rx_init_wait_logged, 0, 1)) {
		LOG_DBG("SMS_RCV initialization...please wait");
	}

	if (!tb45_sms_rx_init_is_ready()) {
		tb45_sms_rx_init_merge_trigger(trigger);
		if (!tb45_delayable_retry_is_pending(&tb45_sms_rx_init_ctx.retry)) {
			tb45_sms_rx_init_ctx.last_ret = 0;
			(void)tb45_delayable_retry_start(&tb45_sms_rx_init_ctx.retry, 0U);
		}
		return;
	}

	if (atomic_cas(&tb45_sms_rx_init_completed_logged, 0, 1)) {
		LOG_INF("SMS_RCV initialization...completed");
	}

	if ((trigger->full_scan != 0U) || (trigger->storage_index == 0U)) {
		(void)tb45_sms_rx_scan_unread(tb45_sms_rx_capture_buf,
					     sizeof(tb45_sms_rx_capture_buf));
	} else {
		(void)tb45_sms_rx_process_index(trigger->storage_index,
					      tb45_sms_rx_capture_buf,
					      sizeof(tb45_sms_rx_capture_buf));
	}
}

static void tb45_sms_rx_poll_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	while (true) {
		struct tb45_sms_rx_trigger trigger;
		int ret = k_msgq_get(&tb45_sms_rx_trigger_msgq, &trigger, K_NO_WAIT);

		if (ret == -ENOMSG) {
			break;
		}

		if (ret < 0) {
			LOG_ERR("SMS_RCV queue read failed (%d)", ret);
			break;
		}

		tb45_sms_rx_process_trigger(&trigger);
		if (tb45_sms_rx_capture_ctx.active) {
			break;
		}
	}

	if (tb45_sms_rx_capture_ctx.active) {
		return;
	}

	tb45_sms_rx_delete_kick();
	if (!tb45_delayable_retry_is_pending(&tb45_sms_rx_delete_ctx.retry)) {
		tb45_sms_rx_resume_poll();
	}
}

static int tb45_sms_rx_init(void)
{
	if (!atomic_cas(&tb45_sms_rx_started, 0, 1)) {
		return 0;
	}

	k_work_poll_init(&tb45_sms_rx_poll_work, tb45_sms_rx_poll_work_handler);
	k_work_init(&tb45_sms_rx_init_ctx.continue_work, tb45_sms_rx_init_continue_work_handler);
	k_work_init(&tb45_sms_rx_capture_ctx.continue_work, tb45_sms_rx_capture_continue_work_handler);
	tb45_delayable_retry_init(&tb45_sms_rx_init_ctx.retry,
				      &tb45_sms_rx_init_retry_ops);
	tb45_delayable_retry_init(&tb45_sms_rx_delete_ctx.retry,
				      &tb45_sms_rx_delete_retry_ops);
	tb45_sms_rx_init_ctx.state = TB45_SMS_RX_INIT_STATE_IDLE;
	tb45_sms_rx_init_ctx.last_ret = 0;
	tb45_sms_rx_init_ctx.pending_trigger_valid = false;
	tb45_sms_rx_delete_ctx.storage_index = 0U;
	tb45_sms_rx_delete_ctx.last_ret = 0;
	tb45_sms_rx_capture_ctx.state = TB45_SMS_RX_CAPTURE_STATE_IDLE;
	tb45_sms_rx_capture_ctx.last_ret = 0;
	tb45_sms_rx_capture_ctx.active = false;
	atomic_set(&tb45_sms_rx_start_allowed, 0);

	int ret = tb45_sms_rx_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("SMS_RCV init submit failed (%d)", ret);
	}

	return 0;
}

SYS_INIT(tb45_sms_rx_init, POST_KERNEL, 100);

int tb45_sms_send(const struct shell *sh, const struct tb45_sms_request *request)
{
	int ret;
	int attempt;
	int max_retries = 0;
	char cmgs_cmd[48];
	uint32_t message_id;

	if (request == NULL) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	if (!tb45_sms_phone_is_valid(request->phone_number)) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	if (!tb45_sms_text_is_valid(request->message, NULL)) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	ret = snprintf(cmgs_cmd, sizeof(cmgs_cmd), "AT+CMGS=\"%s\"", request->phone_number);
	if ((ret < 0) || ((size_t)ret >= sizeof(cmgs_cmd))) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	message_id = request->message_id;
	if (message_id == 0U) {
		message_id = tb45_sms_next_request_id();
	}

#if defined(CONFIG_SHELL)
	if (sh != NULL) {
		max_retries = TB45_SMS_SEND_MAX_RETRIES;
	}
#endif

	for (attempt = 0; attempt <= max_retries; attempt++) {
#if defined(CONFIG_SHELL)
		if ((sh != NULL) && tb45_sms_send_cancel_pending(sh)) {
			shell_warn(sh, "SMS send canceled by Ctrl+C");
			return -ECANCELED;
		}
#endif
		ret = tb45_sms_at_send_text_raw(sh, cmgs_cmd, request->message,
						TB45_SMS_SEND_ATTEMPT_TIMEOUT_MS);
		if (ret == 0) {
#if defined(CONFIG_SHELL)
			if (sh != NULL) {
				shell_print(sh, "[SMS_SND] send_ok id=%u phone=%s", message_id,
					    request->phone_number);
			}
#endif
			return 0;
		}

#if defined(CONFIG_SHELL)
		if ((ret == -ECANCELED) && (sh != NULL)) {
			shell_warn(sh, "SMS send canceled by Ctrl+C");
			return -ECANCELED;
		}
#endif

		if (sh != NULL) {
#if defined(CONFIG_SHELL)
			if (attempt < max_retries) {
				shell_error(sh, "SMS send failed (%d), retry %d/%d", ret, attempt + 1,
						    max_retries);
			} else {
				shell_error(sh, "[SMS_SND] send_fail id=%u rc=%d phone=%s", message_id,
				    ret, request->phone_number);
			}
#else
			LOG_ERR("SMS send failed (%d)", ret);
#endif
		}

#if defined(CONFIG_SHELL)
		if ((attempt < max_retries) &&
		    (tb45_sms_send_sleep_interruptible(sh, TB45_SMS_SEND_RETRY_DELAY_MS) ==
		     -ECANCELED)) {
			shell_warn(sh, "SMS send canceled by Ctrl+C");
			return -ECANCELED;
		}
#endif
	}

	return ret;
}

int tb45_sms_send_enqueue(const struct tb45_sms_request *request)
{
	struct tb45_async_job job;
	int ret = tb45_sms_build_enqueue_job(&job, TB45_ASYNC_JOB_TYPE_SMS_SEND,
					     request, NULL);
	if (ret < 0) {
		return ret;
	}

	return tb45_async_job_enqueue(&job);
}

int tb45_sms_send_enqueue_with_result_cb(const struct tb45_sms_request *request)
{
	struct tb45_sms_request request_with_id;

	if (request == NULL) {
		return -EINVAL;
	}

	request_with_id = *request;
	if (request_with_id.message_id == 0U) {
		request_with_id.message_id = tb45_sms_next_request_id();
	}

	return tb45_sms_send_enqueue_with_result_id(&request_with_id);
}

int tb45_sms_send_enqueue_with_result_id(const struct tb45_sms_request *request)
{
	struct tb45_async_job job;
	int ret;

	if ((request == NULL) || (request->message_id == 0U)) {
		return -EINVAL;
	}

	ret = tb45_sms_build_enqueue_job(&job, TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT,
					 request, NULL);
	if (ret < 0) {
		LOG_ERR("[SMS_ENQUEUE] enqueue_build_fail id=%u rc=%d phone=%s",
			request->message_id, ret, request->phone_number);
		return ret;
	}

	ret = tb45_async_job_enqueue(&job);
	if (ret == 0) {
		LOG_DBG("[SMS_ENQUEUE] enqueue_ok id=%u phone=%s", request->message_id,
			request->phone_number);
	} else {
		LOG_ERR("[SMS_ENQUEUE] enqueue_fail id=%u rc=%d phone=%s",
			request->message_id, ret, request->phone_number);
	}

	return ret;
}

int tb45_sms_send_enqueue_wait(const struct tb45_sms_request *request)
{
	struct tb45_async_job job;
	struct tb45_async_wait_ctx wait_ctx;
	int ret;

	tb45_async_wait_ctx_init(&wait_ctx, -EINPROGRESS);

	ret = tb45_sms_build_enqueue_job(&job, TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT,
					 request, &wait_ctx);
	if (ret < 0) {
		return ret;
	}

	ret = tb45_async_job_enqueue(&job);
	if (ret < 0) {
		return ret;
	}

	(void)k_sem_take(&wait_ctx.done, K_FOREVER);
	return wait_ctx.result;
}

int tb45_sms_send_global(const struct tb45_sms_request *request)
{
	return tb45_sms_send_enqueue_wait(request);
}

int tb45_sms_result_wait(struct tb45_sms_result *out_result, k_timeout_t timeout)
{
	int ret;

	if (out_result == NULL) {
		return -EINVAL;
	}

	ret = k_sem_take(&tb45_sms_result_sem, timeout);
	if (ret < 0) {
		return ret;
	}

	k_mutex_lock(&tb45_sms_result_lock, K_FOREVER);
	if (tb45_sms_result_count == 0U) {
		k_mutex_unlock(&tb45_sms_result_lock);
		return -EAGAIN;
	}

	*out_result = tb45_sms_results[tb45_sms_result_head];
	tb45_sms_result_head = (uint16_t)((tb45_sms_result_head + 1U) % TB45_SMS_RESULT_QUEUE_DEPTH);
	tb45_sms_result_count--;
	k_mutex_unlock(&tb45_sms_result_lock);

	return 0;
}

int tb45_sms_result_get(uint32_t request_id, struct tb45_sms_result *out_result)
{
	uint16_t i;
	uint16_t j;

	if ((request_id == 0U) || (out_result == NULL)) {
		return -EINVAL;
	}

	k_mutex_lock(&tb45_sms_result_lock, K_FOREVER);

	for (i = 0U; i < tb45_sms_result_count; i++) {
		uint16_t idx = (uint16_t)((tb45_sms_result_head + i) % TB45_SMS_RESULT_QUEUE_DEPTH);

		if (tb45_sms_results[idx].request_id != request_id) {
			continue;
		}

		*out_result = tb45_sms_results[idx];

		for (j = i; j + 1U < tb45_sms_result_count; j++) {
			uint16_t from =
				(uint16_t)((tb45_sms_result_head + j + 1U) % TB45_SMS_RESULT_QUEUE_DEPTH);
			uint16_t to = (uint16_t)((tb45_sms_result_head + j) % TB45_SMS_RESULT_QUEUE_DEPTH);
			tb45_sms_results[to] = tb45_sms_results[from];
		}

		tb45_sms_result_tail = (uint16_t)((tb45_sms_result_tail +
						  TB45_SMS_RESULT_QUEUE_DEPTH - 1U) %
						 TB45_SMS_RESULT_QUEUE_DEPTH);
		tb45_sms_result_count--;
		k_mutex_unlock(&tb45_sms_result_lock);
		(void)k_sem_take(&tb45_sms_result_sem, K_NO_WAIT);
		return 0;
	}

	k_mutex_unlock(&tb45_sms_result_lock);
	return -ENOENT;
}

int tb45_sms_receive_trigger_scan(void)
{
	return tb45_sms_rx_enqueue_trigger(0U, 1U);
}

int tb45_sms_receive_recover_stored_unread_messages(void)
{
	return tb45_sms_receive_trigger_scan();
}

int tb45_sms_receive_trigger_index(uint16_t storage_index)
{
	if (storage_index == 0U) {
		return -EINVAL;
	}

	return tb45_sms_rx_enqueue_trigger(storage_index, 0U);
}

int tb45_sms_receive_prepare_for_modem_reconnect(void)
{
	atomic_set(&tb45_sms_rx_start_allowed, 0);
	tb45_sms_rx_reset_runtime_state();
	return 0;
}

int tb45_sms_receive_recover_after_modem_reconnect(void)
{
	tb45_sms_rx_reset_runtime_state();
	atomic_set(&tb45_sms_rx_start_allowed, 1);
	return tb45_sms_receive_trigger_scan();
}

int tb45_sms_receive_notice_wait(struct tb45_sms_rx_notice *out_notice, k_timeout_t timeout)
{
	if (out_notice == NULL) {
		return -EINVAL;
	}

	return k_msgq_get(&tb45_sms_rx_result_msgq, out_notice, timeout);
}

int tb45_sms_receive_read_index(uint16_t storage_index, struct tb45_sms_rx_message *out_message)
{
	char cmd[24];
	char response[TB45_SMS_RX_CAPTURE_BUF_SIZE];
	int ret;

	if ((storage_index == 0U) || (out_message == NULL)) {
		return -EINVAL;
	}

	ret = snprintf(cmd, sizeof(cmd), "AT+CMGR=%u", storage_index);
	if ((ret < 0) || ((size_t)ret >= sizeof(cmd))) {
		return -EINVAL;
	}

	ret = tb45_sms_at_exec_capture(cmd, response, sizeof(response), TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_cmgr_fail_count);
		LOG_DBG("SMS_RCV CMGR idx=%u failed (%d)", storage_index, ret);
		return ret;
	}

	ret = tb45_sms_parse_cmgr_response(storage_index, response, out_message);
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_parse_fail_count);
		LOG_WRN("SMS_RCV parse failed idx=%u (%d)", storage_index, ret);
		return ret;
	}

	return 0;
}

int tb45_sms_receive_delete_index(uint16_t storage_index)
{
	if (storage_index == 0U) {
		return -EINVAL;
	}

	return tb45_sms_rx_enqueue_delete(storage_index);
}

int tb45_sms_receive_wait(struct tb45_sms_rx_message *out_message, k_timeout_t timeout)
{
	struct tb45_sms_rx_notice notice;
	int ret;

	if (out_message == NULL) {
		return -EINVAL;
	}

	ret = tb45_sms_receive_notice_wait(&notice, timeout);
	if (ret < 0) {
		return ret;
	}

	if (notice.status < 0) {
		memset(out_message, 0, sizeof(*out_message));
		out_message->storage_index = notice.storage_index;
		return notice.status;
	}

	return tb45_sms_receive_read_index(notice.storage_index, out_message);
}

int tb45_sms_get_stats(struct tb45_sms_stats *out_stats)
{
	if (out_stats == NULL) {
		return -EINVAL;
	}

	memset(out_stats, 0, sizeof(*out_stats));
	out_stats->rx_setup_done = (uint32_t)(atomic_get(&tb45_sms_rx_setup_done) != 0);
	out_stats->rx_cleanup_done = (uint32_t)(atomic_get(&tb45_sms_rx_cleanup_done) != 0);
	out_stats->rx_init_completed_logged =
		(uint32_t)(atomic_get(&tb45_sms_rx_init_completed_logged) != 0);
	out_stats->rx_init_retry_count = (uint32_t)atomic_get(&tb45_sms_rx_init_retry_count);
	out_stats->rx_trigger_queue_used = (uint32_t)k_msgq_num_used_get(&tb45_sms_rx_trigger_msgq);
	out_stats->rx_result_queue_used = (uint32_t)k_msgq_num_used_get(&tb45_sms_rx_result_msgq);
	out_stats->rx_processed_count = (uint32_t)atomic_get(&tb45_sms_rx_processed_count);
	out_stats->rx_scan_fail_count = (uint32_t)atomic_get(&tb45_sms_rx_scan_fail_count);
	out_stats->rx_cmgr_fail_count = (uint32_t)atomic_get(&tb45_sms_rx_cmgr_fail_count);
	out_stats->rx_parse_fail_count = (uint32_t)atomic_get(&tb45_sms_rx_parse_fail_count);
	out_stats->rx_delete_fail_count = (uint32_t)atomic_get(&tb45_sms_rx_delete_fail_count);
	out_stats->rx_trigger_queue_drop_count =
		(uint32_t)atomic_get(&tb45_sms_rx_trigger_queue_drop_count);
	out_stats->rx_result_queue_drop_count =
		(uint32_t)atomic_get(&tb45_sms_rx_result_queue_drop_count);
	out_stats->async_result_queue_drop_count =
		(uint32_t)atomic_get(&tb45_sms_async_result_queue_drop_count);

	k_mutex_lock(&tb45_sms_result_lock, K_FOREVER);
	out_stats->async_result_queue_used = tb45_sms_result_count;
	k_mutex_unlock(&tb45_sms_result_lock);

	return 0;
}
