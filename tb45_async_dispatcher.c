#include "tb45_async_job_internal.h"

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

static struct k_work_poll tb45_async_dispatcher_poll_work;
static struct k_poll_event tb45_async_dispatcher_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(
		K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
		K_POLL_MODE_NOTIFY_ONLY,
		&tb45_async_job_msgq,
		0),
};
static atomic_t tb45_async_dispatcher_wq_not_ready_warned = ATOMIC_INIT(0);

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

		ret = tb45_async_job_execute(&job);
		if ((ret != 0) && (ret != -ENODEV) && (ret != -EINVAL)) {
			LOG_WRN("async dispatcher: job type=%d ended with %d", job.type, ret);
		}
	}

	int ret = tb45_async_dispatcher_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("async dispatcher poll submit failed (%d)", ret);
	}
}

static int tb45_async_dispatcher_init(void)
{
	k_work_poll_init(&tb45_async_dispatcher_poll_work,
			 tb45_async_dispatcher_work_handler);

	int ret = tb45_async_dispatcher_submit_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("async dispatcher init submit failed (%d)", ret);
	}

	return 0;
}

SYS_INIT(tb45_async_dispatcher_init, POST_KERNEL, 98);
