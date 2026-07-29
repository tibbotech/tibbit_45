#include "tb45_delayable_retry.h"

#include <errno.h>

#include <zephyr/sys/util.h>
#if __has_include("work_queues.h")
#include "work_queues.h"
#else
extern struct k_work_q low_priority_wq;
#endif

static bool tb45_delayable_retry_finish_attempt(struct tb45_delayable_retry *retry, int ret)
{
	retry->attempt_in_progress = false;

	if (ret == 0) {
		retry->pending = false;
		retry->ops->complete(retry, 0);
		return false;
	}

	if (retry->ops->attempt_failed != NULL) {
		retry->ops->attempt_failed(retry, ret, retry->attempt, retry->max_attempts);
	}

	if ((retry->max_attempts == 0U) || (retry->attempt < retry->max_attempts)) {
		uint32_t delay_ms = 0U;
		int sched_ret;

		if (retry->ops->retry_delay_ms != NULL) {
			delay_ms = retry->ops->retry_delay_ms(retry, retry->attempt);
		}

		sched_ret = k_work_reschedule_for_queue(&low_priority_wq, &retry->dwork, K_MSEC(delay_ms));
		if (sched_ret >= 0) {
			if ((retry->max_attempts != 0U) || (retry->attempt < UINT8_MAX)) {
				retry->attempt++;
			}
			retry->pending = true;
			return true;
		}

		ret = sched_ret;
	}

	retry->pending = false;
	retry->ops->complete(retry, ret);
	return false;
}

static bool tb45_delayable_retry_step(struct tb45_delayable_retry *retry)
{
	int ret;

	if ((retry == NULL) || (retry->ops == NULL) || (retry->ops->run_attempt == NULL) ||
	    (retry->ops->complete == NULL)) {
		return false;
	}

	retry->pending = true;
	retry->attempt_in_progress = true;
	ret = retry->ops->run_attempt(retry, retry->attempt);
	if (ret == -EINPROGRESS) {
		return true;
	}

	return tb45_delayable_retry_finish_attempt(retry, ret);
}

static void tb45_delayable_retry_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tb45_delayable_retry *retry = CONTAINER_OF(dwork, struct tb45_delayable_retry, dwork);

	(void)tb45_delayable_retry_step(retry);
}

void tb45_delayable_retry_init(struct tb45_delayable_retry *retry,
			       const struct tb45_delayable_retry_ops *ops)
{
	if (retry == NULL) {
		return;
	}

	retry->ops = ops;
	retry->attempt = 0U;
	retry->max_attempts = 0U;
	retry->pending = false;
	retry->attempt_in_progress = false;
	k_work_init_delayable(&retry->dwork, tb45_delayable_retry_work_handler);
}

bool tb45_delayable_retry_start(struct tb45_delayable_retry *retry, uint8_t max_attempts)
{
	if (retry == NULL) {
		return false;
	}

	retry->attempt = 1U;
	retry->max_attempts = max_attempts;
	retry->pending = false;
	retry->attempt_in_progress = false;
	return tb45_delayable_retry_step(retry);
}

bool tb45_delayable_retry_complete(struct tb45_delayable_retry *retry, int ret)
{
	if ((retry == NULL) || !retry->attempt_in_progress) {
		return false;
	}

	return tb45_delayable_retry_finish_attempt(retry, ret);
}

bool tb45_delayable_retry_is_pending(const struct tb45_delayable_retry *retry)
{
	return (retry != NULL) && retry->pending;
}
