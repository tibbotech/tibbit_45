#include "tb45_async_job_internal.h"
#include "tb45_delayable_retry.h"

#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#if __has_include("work_queues.h")
#include "work_queues.h"
#else
extern struct k_work_q low_priority_wq;
#endif

LOG_MODULE_REGISTER(tb45_async_dispatcher, CONFIG_LOG_DEFAULT_LEVEL);

static struct k_work_poll tb45_async_dispatcher_poll_work = {0};
static struct k_poll_event tb45_async_dispatcher_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(
		K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
		K_POLL_MODE_NOTIFY_ONLY,
		&tb45_async_job_msgq,
		0),
};
static atomic_t tb45_async_dispatcher_wq_not_ready_warned = ATOMIC_INIT(0);

struct tb45_async_dispatcher_retry_ctx {
	struct tb45_delayable_retry retry;
	struct tb45_async_job job;
};

static struct tb45_async_dispatcher_retry_ctx tb45_async_dispatcher_retry_ctx = {0};

static bool tb45_async_dispatcher_wq_ready(void)
{
	bool ready = k_work_queue_thread_get(&low_priority_wq) != NULL;

	if (ready) {
		atomic_set(&tb45_async_dispatcher_wq_not_ready_warned, 0);
	}

	return ready;
}

static int tb45_async_dispatcher_submit_poll(void)
{
	if (!tb45_async_dispatcher_wq_ready()) {
		if (atomic_cas(&tb45_async_dispatcher_wq_not_ready_warned, 0, 1)) {
			LOG_WRN("async dispatcher queue not ready; waiting for work_queues_init()");
		}
		return -EAGAIN;
	}

	tb45_async_dispatcher_events[0].state = K_POLL_STATE_NOT_READY;
	return k_work_poll_submit_to_queue(&low_priority_wq,
					   &tb45_async_dispatcher_poll_work,
					   tb45_async_dispatcher_events,
					   ARRAY_SIZE(tb45_async_dispatcher_events),
					   K_FOREVER);
}

static void tb45_async_dispatcher_resume_poll(void)
{
	int ret = tb45_async_dispatcher_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("async dispatcher poll submit failed (%d)", ret);
	}
}

static int tb45_async_dispatcher_run_attempt(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	struct tb45_async_dispatcher_retry_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_async_dispatcher_retry_ctx, retry);

	ARG_UNUSED(attempt);
	return tb45_async_job_execute(&ctx->job);
}

static uint32_t tb45_async_dispatcher_retry_delay_ms(struct tb45_delayable_retry *retry, uint8_t attempt)
{
	struct tb45_async_dispatcher_retry_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_async_dispatcher_retry_ctx, retry);

	ARG_UNUSED(attempt);
	return tb45_async_job_retry_delay_ms(ctx->job.type);
}

static void tb45_async_dispatcher_attempt_failed(struct tb45_delayable_retry *retry, int ret, uint8_t attempt,
						  uint8_t max_attempts)
{
	struct tb45_async_dispatcher_retry_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_async_dispatcher_retry_ctx, retry);

	LOG_WRN("async job type=%d failed (%d), attempt %u/%u", ctx->job.type, ret, attempt,
		max_attempts);
}

static void tb45_async_dispatcher_retry_complete(struct tb45_delayable_retry *retry, int ret)
{
	struct tb45_async_dispatcher_retry_ctx *ctx =
		CONTAINER_OF(retry, struct tb45_async_dispatcher_retry_ctx, retry);

	tb45_async_job_complete(&ctx->job, ret);
	tb45_async_dispatcher_resume_poll();
}

static const struct tb45_delayable_retry_ops tb45_async_dispatcher_retry_ops = {
	.run_attempt = tb45_async_dispatcher_run_attempt,
	.retry_delay_ms = tb45_async_dispatcher_retry_delay_ms,
	.attempt_failed = tb45_async_dispatcher_attempt_failed,
	.complete = tb45_async_dispatcher_retry_complete,
};

void tb45_async_dispatcher_complete_job(const struct tb45_async_job *job, int ret)
{
	tb45_async_job_complete(job, ret);
	tb45_async_dispatcher_resume_poll();
}

void tb45_async_dispatcher_complete_attempt(int ret)
{
	(void)tb45_delayable_retry_complete(&tb45_async_dispatcher_retry_ctx.retry, ret);
}

static bool tb45_async_dispatcher_run_job(const struct tb45_async_job *job,
				  const struct tb45_async_job_policy *policy)
{
	if (job->type == TB45_ASYNC_JOB_TYPE_PING_RUN) {
		int ret = tb45_async_job_execute(job);

		if (ret == -EINPROGRESS) {
			return true;
		}

		tb45_async_dispatcher_complete_job(job, ret);
		return false;
	}

	tb45_async_dispatcher_retry_ctx.job = *job;
	return tb45_delayable_retry_start(&tb45_async_dispatcher_retry_ctx.retry, policy->max_attempts);
}

static void tb45_async_dispatcher_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	while (true) {
		struct tb45_async_job job;
		int ret = k_msgq_get(&tb45_async_job_msgq, &job, K_NO_WAIT);

		if (ret == -ENOMSG) {
			break;
		}

		if (ret != 0) {
			LOG_ERR("async queue read failed (%d)", ret);
			break;
		}

		struct tb45_async_job_policy policy = tb45_async_job_policy_get(job.type);
		(void)tb45_async_dispatcher_run_job(&job, &policy);
		return;
	}

	tb45_async_dispatcher_resume_poll();
}

static int tb45_async_dispatcher_init(void)
{
	k_work_poll_init(&tb45_async_dispatcher_poll_work,
			 tb45_async_dispatcher_work_handler);
	tb45_delayable_retry_init(&tb45_async_dispatcher_retry_ctx.retry,
				      &tb45_async_dispatcher_retry_ops);

	int ret = tb45_async_dispatcher_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("async dispatcher init submit failed (%d)", ret);
	}

	return 0;
}

SYS_INIT(tb45_async_dispatcher_init, POST_KERNEL, 98);
