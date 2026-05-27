#include "tb45_sms.h"
#include "tb45_async_job.h"
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
#define TB45_SMS_RX_AUTO_DELETE           CONFIG_APP_TB45_SMS_RX_AUTO_DELETE
#define TB45_SMS_RX_STARTUP_CLEANUP       CONFIG_APP_TB45_SMS_RX_STARTUP_CLEANUP
#define TB45_SMS_RX_STORAGE               CONFIG_APP_TB45_SMS_RX_STORAGE

#define TB45_SMS_RX_CAPTURE_BUF_SIZE      768
#define TB45_SMS_RX_INIT_RETRY_DELAY_MS   2000
#define TB45_SMS_SHELL_ASCII_CTRL_C       0x03U
#define TB45_SMS_SLEEP_SLICE_MS           50

struct tb45_sms_rx_trigger {
	uint16_t storage_index;
	uint8_t full_scan;
};

struct tb45_sms_wait_ctx {
	struct k_sem done;
	int result;
};

K_SEM_DEFINE(tb45_sms_result_sem, 0, TB45_SMS_RESULT_QUEUE_DEPTH);
K_MUTEX_DEFINE(tb45_sms_result_lock);
static struct tb45_sms_result tb45_sms_results[TB45_SMS_RESULT_QUEUE_DEPTH];
static uint16_t tb45_sms_result_head;
static uint16_t tb45_sms_result_tail;
static uint16_t tb45_sms_result_count;
static atomic_t tb45_sms_request_id_seed = ATOMIC_INIT(1);
static atomic_t tb45_sms_rx_started = ATOMIC_INIT(0);
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
static char tb45_sms_rx_capture_buf[TB45_SMS_RX_CAPTURE_BUF_SIZE];

K_MSGQ_DEFINE(tb45_sms_rx_trigger_msgq, sizeof(struct tb45_sms_rx_trigger),
	      TB45_SMS_RX_TRIGGER_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tb45_sms_rx_result_msgq, sizeof(struct tb45_sms_rx_message),
	      TB45_SMS_RX_RESULT_QUEUE_DEPTH, 4);

static int tb45_sms_rx_enqueue_trigger(uint16_t storage_index, uint8_t full_scan);

static struct k_work_poll tb45_sms_rx_poll_work;
static struct k_poll_event tb45_sms_rx_poll_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&tb45_sms_rx_trigger_msgq,
					0),
};
static struct k_work_delayable tb45_sms_rx_retry_work;
static atomic_t tb45_sms_rx_wq_not_ready_warned = ATOMIC_INIT(0);

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
	return k_work_poll_submit_to_queue(&low_priority_wq,
					   &tb45_sms_rx_poll_work,
					   tb45_sms_rx_poll_events,
					   ARRAY_SIZE(tb45_sms_rx_poll_events),
					   K_FOREVER);
}

static void tb45_sms_rx_retry_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)tb45_sms_rx_enqueue_trigger(0U, 1U);
}

static void tb45_sms_rx_schedule_retry(void)
{
	if (!tb45_sms_rx_wq_ready()) {
		(void)tb45_sms_rx_enqueue_trigger(0U, 1U);
		return;
	}

	int ret = k_work_reschedule_for_queue(&low_priority_wq, &tb45_sms_rx_retry_work,
					      K_MSEC(TB45_SMS_RX_INIT_RETRY_DELAY_MS));
	if (ret < 0) {
		LOG_WRN("SMS_RCV retry schedule failed (%d)", ret);
	}
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

	LOG_WRN("Usage: tb45 sms send <phone> <text>");
	LOG_WRN("Example: tb45 sms send \"+886123456789\" \"hello\"");
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

static void tb45_sms_rx_publish_message(const struct tb45_sms_rx_message *message)
{
	struct tb45_sms_rx_message dropped;
	int ret = k_msgq_put(&tb45_sms_rx_result_msgq, message, K_NO_WAIT);

	if (ret == 0) {
		return;
	}

	if (ret != -ENOMSG) {
		LOG_WRN("SMS_RCV queue put failed (%d)", ret);
		return;
	}

	atomic_inc(&tb45_sms_rx_result_queue_drop_count);
	(void)k_msgq_get(&tb45_sms_rx_result_msgq, &dropped, K_NO_WAIT);
	ret = k_msgq_put(&tb45_sms_rx_result_msgq, message, K_NO_WAIT);
	if (ret < 0) {
		LOG_WRN("SMS_RCV queue overflow; dropped message idx=%u", message->storage_index);
	}
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

static int tb45_sms_build_enqueue_job(struct tb45_async_job *job,
				      const struct tb45_sms_request *request, uint8_t capture_result,
				      void *completion_ctx)
{
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
	job->type = TB45_ASYNC_JOB_TYPE_SMS_SEND;
	memcpy(job->payload.sms_send.request.phone_number, request->phone_number, phone_len + 1);
	memcpy(job->payload.sms_send.request.message, request->message, text_len + 1);
	job->payload.sms_send.request.message_id = request->message_id;
	job->payload.sms_send.capture_result = capture_result;
	job->payload.sms_send.completion_ctx = completion_ctx;

	return 0;
}

static int tb45_sms_async_dispatch(const struct tb45_async_job *job)
{
	if ((job == NULL) || (job->type != TB45_ASYNC_JOB_TYPE_SMS_SEND)) {
		return -EINVAL;
	}

	LOG_DBG("SMS async dispatch: sending to %s", job->payload.sms_send.request.phone_number);
	int ret = tb45_sms_send(NULL, &job->payload.sms_send.request);
	struct tb45_sms_wait_ctx *wait_ctx = job->payload.sms_send.completion_ctx;

	if (job->payload.sms_send.capture_result != 0U) {
		tb45_sms_store_async_result(job->payload.sms_send.request.message_id,
					    job->payload.sms_send.request.phone_number, ret);
	}

	if (wait_ctx != NULL) {
		wait_ctx->result = ret;
		k_sem_give(&wait_ctx->done);
	}

	if (ret == 0) {
		LOG_DBG("SMS async dispatch: sent successfully");
	} else {
		LOG_ERR("SMS async dispatch: send failed (%d)", ret);
	}

	return ret;
}

static int tb45_sms_async_register_handler(void)
{
	int ret = tb45_async_job_register_handler(TB45_ASYNC_JOB_TYPE_SMS_SEND,
						  tb45_sms_async_dispatch);
	if (ret < 0) {
		LOG_ERR("SMS async register failed (%d)", ret);
	}

	return 0;
}

SYS_INIT(tb45_sms_async_register_handler, POST_KERNEL, 99);

static int tb45_sms_rx_configure_once(void)
{
	char cmd[48];
	int ret;

	if (atomic_get(&tb45_sms_rx_setup_done) != 0) {
		return 0;
	}

	ret = tb45_sms_at_run(NULL, "AT+CMGF=1", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	ret = tb45_sms_at_run(NULL, "AT+CSCS=\"IRA\"", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	ret = snprintf(cmd, sizeof(cmd), "AT+CPMS=\"%s\",\"%s\",\"%s\"",
		       TB45_SMS_RX_STORAGE, TB45_SMS_RX_STORAGE, TB45_SMS_RX_STORAGE);
	if ((ret < 0) || ((size_t)ret >= sizeof(cmd))) {
		return -EINVAL;
	}

	ret = tb45_sms_at_run(NULL, cmd, "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	ret = tb45_sms_at_run(NULL, "AT+CNMI=2,1,0,0,0", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	atomic_set(&tb45_sms_rx_setup_done, 1);
	LOG_DBG("SMS_RCV setup complete: CPMS=%s CNMI enabled", TB45_SMS_RX_STORAGE);
	return 0;
}

static int tb45_sms_rx_startup_cleanup_once(void)
{
	int ret;

	if (TB45_SMS_RX_STARTUP_CLEANUP == 0) {
		return 0;
	}

	if (atomic_get(&tb45_sms_rx_cleanup_done) != 0) {
		return 0;
	}

	ret = tb45_sms_at_run(NULL, "AT+CMGD=1,4", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		return ret;
	}

	atomic_set(&tb45_sms_rx_cleanup_done, 1);
	LOG_DBG("SMS_RCV startup cleanup completed");
	return 0;
}

static int tb45_sms_rx_delete_index(uint16_t storage_index)
{
	char cmd[24];
	int ret = snprintf(cmd, sizeof(cmd), "AT+CMGD=%u", storage_index);

	if ((ret < 0) || ((size_t)ret >= sizeof(cmd))) {
		return -EINVAL;
	}

	return tb45_sms_at_run(NULL, cmd, "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
}

static int tb45_sms_rx_process_index(uint16_t storage_index, char *capture_buf, size_t capture_buf_size)
{
	char cmd[24];
	struct tb45_sms_rx_message message;
	int ret = snprintf(cmd, sizeof(cmd), "AT+CMGR=%u", storage_index);

	if ((capture_buf == NULL) || (capture_buf_size == 0U)) {
		return -EINVAL;
	}

	if ((ret < 0) || ((size_t)ret >= sizeof(cmd))) {
		return -EINVAL;
	}

	ret = tb45_sms_at_exec_capture(cmd, capture_buf, capture_buf_size, TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_cmgr_fail_count);
		memset(&message, 0, sizeof(message));
		message.storage_index = storage_index;
		LOG_DBG("SMS_RCV CMGR idx=%u failed (%d)", storage_index, ret);
		tb45_sms_event_notify_rx_message(&message, ret);
		return ret;
	}

	ret = tb45_sms_parse_cmgr_response(storage_index, capture_buf, &message);
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_parse_fail_count);
		memset(&message, 0, sizeof(message));
		message.storage_index = storage_index;
		LOG_WRN("SMS_RCV parse failed idx=%u (%d)", storage_index, ret);
		tb45_sms_event_notify_rx_message(&message, ret);
		return ret;
	}

	atomic_inc(&tb45_sms_rx_processed_count);
	tb45_sms_rx_publish_message(&message);
	tb45_sms_event_notify_rx_message(&message, 0);

	if (TB45_SMS_RX_AUTO_DELETE != 0) {
		ret = tb45_sms_rx_delete_index(storage_index);
		if (ret < 0) {
			atomic_inc(&tb45_sms_rx_delete_fail_count);
			LOG_WRN("SMS_RCV delete failed idx=%u (%d)", storage_index, ret);
		}
	}

	return 0;
}

static int tb45_sms_rx_scan_unread(char *capture_buf, size_t capture_buf_size)
{
	uint16_t indexes[8];
	size_t count;
	size_t i;
	int ret;

	if ((capture_buf == NULL) || (capture_buf_size == 0U)) {
		return -EINVAL;
	}

	ret = tb45_sms_at_exec_capture("AT+CMGL=\"REC UNREAD\"", capture_buf, capture_buf_size,
				       TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_scan_fail_count);
		return ret;
	}

	count = tb45_sms_parse_cmgl_unread_indexes(capture_buf, indexes, ARRAY_SIZE(indexes));
	for (i = 0U; i < count; i++) {
		(void)tb45_sms_rx_process_index(indexes[i], capture_buf, capture_buf_size);
	}

	return 0;
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

	int ret = tb45_sms_rx_configure_once();
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_init_retry_count);
		LOG_DBG("SMS_RCV setup pending (%d)", ret);
		tb45_sms_rx_schedule_retry();
		return;
	}

	ret = tb45_sms_rx_startup_cleanup_once();
	if (ret < 0) {
		atomic_inc(&tb45_sms_rx_init_retry_count);
		LOG_DBG("SMS_RCV startup cleanup pending (%d)", ret);
		tb45_sms_rx_schedule_retry();
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
	}

	int ret = tb45_sms_rx_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("SMS_RCV poll submit failed (%d)", ret);
	}
}

static int tb45_sms_rx_init(void)
{
	if (!atomic_cas(&tb45_sms_rx_started, 0, 1)) {
		return 0;
	}

	k_work_poll_init(&tb45_sms_rx_poll_work, tb45_sms_rx_poll_work_handler);
	k_work_init_delayable(&tb45_sms_rx_retry_work, tb45_sms_rx_retry_work_handler);

	int ret = tb45_sms_rx_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("SMS_RCV init submit failed (%d)", ret);
	}

	(void)tb45_sms_receive_trigger_scan();
	return 0;
}

SYS_INIT(tb45_sms_rx_init, POST_KERNEL, 100);

int tb45_sms_send(const struct shell *sh, const struct tb45_sms_request *request)
{
	int ret;
	int attempt;
	char cmgs_cmd[48];

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

	for (attempt = 0; attempt <= TB45_SMS_SEND_MAX_RETRIES; attempt++) {
#if defined(CONFIG_SHELL)
		if ((sh != NULL) && tb45_sms_send_cancel_pending(sh)) {
			shell_warn(sh, "SMS send canceled by Ctrl+C");
			return -ECANCELED;
		}
#endif
		ret = tb45_sms_at_send_text_raw(sh, cmgs_cmd, request->message,
						TB45_SMS_SEND_ATTEMPT_TIMEOUT_MS);
		if (ret == 0) {
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
			if (attempt < TB45_SMS_SEND_MAX_RETRIES) {
				shell_error(sh, "SMS send failed (%d), retry %d/%d", ret, attempt + 1,
						    TB45_SMS_SEND_MAX_RETRIES);
			} else {
				shell_error(sh, "SMS send failed (%d), retries exhausted", ret);
			}
#else
			if (attempt < TB45_SMS_SEND_MAX_RETRIES) {
				LOG_ERR("SMS send failed (%d), retry %d/%d", ret, attempt + 1,
					TB45_SMS_SEND_MAX_RETRIES);
			} else {
				LOG_ERR("SMS send failed (%d), retries exhausted", ret);
			}
#endif
		}

		if (attempt < TB45_SMS_SEND_MAX_RETRIES) {
#if defined(CONFIG_SHELL)
			if ((sh != NULL) &&
			    (tb45_sms_send_sleep_interruptible(sh, TB45_SMS_SEND_RETRY_DELAY_MS) ==
			     -ECANCELED)) {
				shell_warn(sh, "SMS send canceled by Ctrl+C");
				return -ECANCELED;
			}
#else
			k_msleep(TB45_SMS_SEND_RETRY_DELAY_MS);
#endif
		}
	}

	return ret;
}

int tb45_sms_send_enqueue(const struct tb45_sms_request *request)
{
	struct tb45_async_job job;
	int ret = tb45_sms_build_enqueue_job(&job, request, 0U, NULL);
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
		tb45_sms_event_notify_enqueue(request, -EINVAL);
		return -EINVAL;
	}

	ret = tb45_sms_build_enqueue_job(&job, request, 1U, NULL);
	if (ret < 0) {
		tb45_sms_event_notify_enqueue(request, ret);
		return ret;
	}

	ret = tb45_async_job_enqueue(&job);
	tb45_sms_event_notify_enqueue(request, ret);
	return ret;
}

int tb45_sms_send_enqueue_wait(const struct tb45_sms_request *request)
{
	struct tb45_async_job job;
	struct tb45_sms_wait_ctx wait_ctx = {
		.result = -EINPROGRESS,
	};
	int ret;

	k_sem_init(&wait_ctx.done, 0, 1);

	ret = tb45_sms_build_enqueue_job(&job, request, 0U, &wait_ctx);
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

int tb45_sms_receive_trigger_index(uint16_t storage_index)
{
	if (storage_index == 0U) {
		return -EINVAL;
	}

	return tb45_sms_rx_enqueue_trigger(storage_index, 0U);
}

int tb45_sms_receive_recover_after_modem_reconnect(void)
{
	atomic_set(&tb45_sms_rx_setup_done, 0);
	atomic_set(&tb45_sms_rx_init_wait_logged, 0);
	atomic_set(&tb45_sms_rx_init_completed_logged, 0);
	return tb45_sms_receive_trigger_scan();
}

int tb45_sms_receive_wait(struct tb45_sms_rx_message *out_message, k_timeout_t timeout)
{
	if (out_message == NULL) {
		return -EINVAL;
	}

	return k_msgq_get(&tb45_sms_rx_result_msgq, out_message, timeout);
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
