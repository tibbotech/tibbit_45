#include "tb45_async_job_internal.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tb45_async_job, CONFIG_LOG_DEFAULT_LEVEL);

#define TB45_ASYNC_JOB_QUEUE_DEPTH  CONFIG_APP_TB45_ASYNC_JOB_QUEUE_DEPTH

K_MSGQ_DEFINE(tb45_async_job_msgq, sizeof(struct tb45_async_job), TB45_ASYNC_JOB_QUEUE_DEPTH, 4);

static tb45_async_job_handler_t tb45_async_handlers[TB45_ASYNC_JOB_TYPE_COUNT] = {0};
static tb45_async_job_complete_cb_t tb45_async_complete_cbs[TB45_ASYNC_JOB_TYPE_COUNT] = {0};

static const char *tb45_async_job_type_name(enum tb45_async_job_type type)
{
	switch (type) {
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
		return "SMS_SEND";
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT:
		return "SMS_SEND_CAPTURE_RESULT";
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT:
		return "SMS_SEND_WAIT";
#endif
	case TB45_ASYNC_JOB_TYPE_PING_RUN:
		return "PING_RUN";
	default:
		return "UNKNOWN";
	}
}

struct tb45_async_job_policy tb45_async_job_policy_get(enum tb45_async_job_type type)
{
	struct tb45_async_job_policy policy = {
		.max_attempts = 1U,
	};

	switch (type) {
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT:
		policy.max_attempts = (uint8_t)(1U + CONFIG_APP_TB45_SMS_SEND_MAX_RETRIES);
		break;
#endif
	case TB45_ASYNC_JOB_TYPE_PING_RUN:
		policy.max_attempts = (uint8_t)(1U + CONFIG_APP_TB45_ASYNC_PING_JOB_EXTRA_RETRIES);
		break;
	default:
		break;
	}

	if (policy.max_attempts == 0U) {
		policy.max_attempts = 1U;
	}

	return policy;
}

uint32_t tb45_async_job_retry_delay_ms(enum tb45_async_job_type type)
{
	switch (type) {
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT:
	case TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT:
		return CONFIG_APP_TB45_SMS_SEND_RETRY_DELAY_MS;
#endif
	case TB45_ASYNC_JOB_TYPE_PING_RUN:
		return CONFIG_APP_TB45_ASYNC_PING_JOB_RETRY_DELAY_MS;
	default:
		return 0U;
	}
}

int tb45_async_job_execute(const struct tb45_async_job *job)
{
	if ((job == NULL) || ((unsigned int)job->type >= TB45_ASYNC_JOB_TYPE_COUNT)) {
		return -EINVAL;
	}

	tb45_async_job_handler_t handler = tb45_async_handlers[job->type];
	if (handler == NULL) {
		LOG_ERR("async job dropped: no handler for type=%s",
			tb45_async_job_type_name(job->type));
		return -ENODEV;
	}

	return handler(job);
}

int tb45_async_job_register_handler(enum tb45_async_job_type type,
				    tb45_async_job_handler_t handler,
				    tb45_async_job_complete_cb_t complete_cb)
{
	if (((unsigned int)type >= TB45_ASYNC_JOB_TYPE_COUNT) || (handler == NULL)) {
		return -EINVAL;
	}

	tb45_async_handlers[type] = handler;
	tb45_async_complete_cbs[type] = complete_cb;
	LOG_INF("async handler registered for %s", tb45_async_job_type_name(type));
	return 0;
}

void tb45_async_job_complete(const struct tb45_async_job *job, int ret)
{
	if ((job == NULL) || ((unsigned int)job->type >= TB45_ASYNC_JOB_TYPE_COUNT)) {
		return;
	}

	tb45_async_job_complete_cb_t complete_cb = tb45_async_complete_cbs[job->type];
	if (complete_cb != NULL) {
		complete_cb(job, ret);
	}
}

void tb45_async_job_complete_wait_ctx(struct tb45_async_wait_ctx *wait_ctx, int ret)
{
	if (wait_ctx == NULL) {
		return;
	}

	wait_ctx->result = ret;
	k_sem_give(&wait_ctx->done);
}

int tb45_async_job_enqueue(const struct tb45_async_job *job)
{
	if ((job == NULL) || ((unsigned int)job->type >= TB45_ASYNC_JOB_TYPE_COUNT)) {
		return -EINVAL;
	}

	int ret = k_msgq_put(&tb45_async_job_msgq, job, K_NO_WAIT);
	if (ret == -ENOMSG) {
		LOG_WRN("async queue full for %s", tb45_async_job_type_name(job->type));
		return -EAGAIN;
	}

	if (ret != 0) {
		LOG_ERR("async queue put failed for %s (%d)",
			tb45_async_job_type_name(job->type), ret);
	}

	return ret;
}
