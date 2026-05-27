#include "tb45_async_job.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tb45_async_job, CONFIG_LOG_DEFAULT_LEVEL);

#define TB45_ASYNC_JOB_QUEUE_DEPTH  CONFIG_APP_TB45_ASYNC_JOB_QUEUE_DEPTH
struct tb45_async_job_policy {
	uint8_t max_attempts;
	uint32_t retry_delay_ms;
};

K_MSGQ_DEFINE(tb45_async_job_msgq, sizeof(struct tb45_async_job), TB45_ASYNC_JOB_QUEUE_DEPTH, 4);

static tb45_async_job_handler_t tb45_async_handlers[TB45_ASYNC_JOB_TYPE_COUNT];

static const char *tb45_async_job_type_name(enum tb45_async_job_type type)
{
	switch (type) {
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
		return "SMS_SEND";
	case TB45_ASYNC_JOB_TYPE_PING_RUN:
		return "PING_RUN";
	default:
		return "UNKNOWN";
	}
}

static struct tb45_async_job_policy tb45_async_job_policy_get(enum tb45_async_job_type type)
{
	struct tb45_async_job_policy policy = {
		.max_attempts = 1U,
		.retry_delay_ms = 0U,
	};

	switch (type) {
	case TB45_ASYNC_JOB_TYPE_SMS_SEND:
		policy.max_attempts = (uint8_t)(1U + CONFIG_APP_TB45_ASYNC_SMS_JOB_EXTRA_RETRIES);
		policy.retry_delay_ms = CONFIG_APP_TB45_ASYNC_SMS_JOB_RETRY_DELAY_MS;
		break;
	case TB45_ASYNC_JOB_TYPE_PING_RUN:
		policy.max_attempts = (uint8_t)(1U + CONFIG_APP_TB45_ASYNC_PING_JOB_EXTRA_RETRIES);
		policy.retry_delay_ms = CONFIG_APP_TB45_ASYNC_PING_JOB_RETRY_DELAY_MS;
		break;
	default:
		break;
	}

	if (policy.max_attempts == 0U) {
		policy.max_attempts = 1U;
	}

	return policy;
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

	struct tb45_async_job_policy policy = tb45_async_job_policy_get(job->type);
	int ret = -EIO;

	for (uint8_t attempt = 1U; attempt <= policy.max_attempts; attempt++) {
		ret = handler(job);
		if (ret == 0) {
			return 0;
		}

		LOG_WRN("async job %s failed (%d), attempt %u/%u",
			tb45_async_job_type_name(job->type), ret,
			attempt, policy.max_attempts);

		if ((attempt < policy.max_attempts) && (policy.retry_delay_ms > 0U)) {
			k_msleep(policy.retry_delay_ms);
		}
	}

	return ret;
}

int tb45_async_job_register_handler(enum tb45_async_job_type type,
				    tb45_async_job_handler_t handler)
{
	if (((unsigned int)type >= TB45_ASYNC_JOB_TYPE_COUNT) || (handler == NULL)) {
		return -EINVAL;
	}

	tb45_async_handlers[type] = handler;
	LOG_INF("async handler registered for %s", tb45_async_job_type_name(type));
	return 0;
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
