#include "tb45_sms_event.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#if __has_include("work_queues.h")
#include "work_queues.h"
#else
extern struct k_work_q low_priority_wq;
#endif

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(tb45_sms_event, CONFIG_LOG_DEFAULT_LEVEL);

#define TB45_SMS_EVENT_QUEUE_DEPTH CONFIG_APP_TB45_SMS_EVENT_QUEUE_DEPTH
#define TB45_SMS_SERIAL_RX_DEBOUNCE_MS CONFIG_APP_TB45_SMS_SERIAL_RX_DEBOUNCE_MS

K_MSGQ_DEFINE(tb45_sms_event_msgq, sizeof(struct tb45_sms_event), TB45_SMS_EVENT_QUEUE_DEPTH, 4);

static struct k_work_poll tb45_sms_event_dispatch_poll_work;
static struct k_poll_event tb45_sms_event_dispatch_poll_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&tb45_sms_event_msgq,
					0),
};

#if defined(CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE) && CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE
extern struct k_sem tb45_sms_result_sem;
static struct k_work_poll tb45_sms_result_poll_work;
static struct k_poll_event tb45_sms_result_poll_events[] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&tb45_sms_result_sem,
					0),
};
#endif

K_MUTEX_DEFINE(tb45_sms_event_cb_lock);
static tb45_sms_event_cb_t tb45_sms_event_cb;
static void *tb45_sms_event_cb_user_data;
static atomic_t tb45_sms_event_started = ATOMIC_INIT(0);
static atomic_t tb45_sms_last_serial_rx_ms = ATOMIC_INIT(0);
static atomic_t tb45_sms_event_wq_not_ready_warned = ATOMIC_INIT(0);

static void tb45_sms_event_dispatch_send(const struct tb45_sms_event *event)
{
	switch (event->type) {
	case TB45_SMS_EVENT_TYPE_ENQUEUE_MSG_STATUS:
		if (event->status == 0) {
			LOG_DBG("[SMS_ENQUEUE] enqueue_ok id=%u phone=%s", event->send.message_id,
				event->send.phone);
		} else {
			LOG_ERR("[SMS_ENQUEUE] enqueue_fail id=%u rc=%d phone=%s",
				event->send.message_id, event->status, event->send.phone);
		}
		break;
	default:
		break;
	}
}

static void tb45_sms_event_dispatch_receive(const struct tb45_sms_event *event)
{
	switch (event->type) {
	case TB45_SMS_EVENT_TYPE_RECEIVE_SERIAL_STATUS:
		LOG_DBG("[SMS_RCV]: serial-rx ts=%u", event->timestamp_ms);
		break;
	default:
		break;
	}
}

static void tb45_sms_event_dispatch_combined(const struct tb45_sms_event *event)
{
	tb45_sms_event_cb_t cb;
	void *user_data;

	k_mutex_lock(&tb45_sms_event_cb_lock, K_FOREVER);
	cb = tb45_sms_event_cb;
	user_data = tb45_sms_event_cb_user_data;
	k_mutex_unlock(&tb45_sms_event_cb_lock);

	if (cb != NULL) {
		cb(event, user_data);
		return;
	}

	tb45_sms_event_dispatch_send(event);
	tb45_sms_event_dispatch_receive(event);
}

static void tb45_sms_event_publish(const struct tb45_sms_event *event)
{
	int ret = k_msgq_put(&tb45_sms_event_msgq, event, K_NO_WAIT);
	if (ret != 0) {
		LOG_WRN("SMS_SND queue full; dropped type=%d", (int)event->type);
	}
}

static bool tb45_sms_event_wq_ready(void)
{
	bool ready = k_work_queue_thread_get(&low_priority_wq) != NULL;

	if (ready) {
		atomic_set(&tb45_sms_event_wq_not_ready_warned, 0);
	}

	return ready;
}

static int tb45_sms_event_submit_dispatch_poll(void)
{
	if (!tb45_sms_event_wq_ready()) {
		if (atomic_cas(&tb45_sms_event_wq_not_ready_warned, 0, 1)) {
			LOG_WRN("SMS_EVT queue not ready; waiting for work_queues_init()");
		}
		return -EAGAIN;
	}

	tb45_sms_event_dispatch_poll_events[0].state = K_POLL_STATE_NOT_READY;
	return k_work_poll_submit_to_queue(&low_priority_wq,
					   &tb45_sms_event_dispatch_poll_work,
					   tb45_sms_event_dispatch_poll_events,
					   ARRAY_SIZE(tb45_sms_event_dispatch_poll_events),
					   K_FOREVER);
}

static void tb45_sms_event_dispatch_worker(struct k_work *work)
{
	ARG_UNUSED(work);

	while (true) {
		struct tb45_sms_event event;
		int ret = k_msgq_get(&tb45_sms_event_msgq, &event, K_NO_WAIT);
		if (ret == -ENOMSG) {
			break;
		}

		if (ret != 0) {
			LOG_ERR("SMS_SND queue read failed (%d)", ret);
			break;
		}

		tb45_sms_event_dispatch_combined(&event);
	}

	int ret = tb45_sms_event_submit_dispatch_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("SMS_EVT dispatch poll submit failed (%d)", ret);
	}
}

#if defined(CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE) && CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE
static int tb45_sms_event_submit_result_poll(void)
{
	if (!tb45_sms_event_wq_ready()) {
		if (atomic_cas(&tb45_sms_event_wq_not_ready_warned, 0, 1)) {
			LOG_WRN("SMS_EVT queue not ready; waiting for work_queues_init()");
		}
		return -EAGAIN;
	}

	tb45_sms_result_poll_events[0].state = K_POLL_STATE_NOT_READY;
	return k_work_poll_submit_to_queue(&low_priority_wq,
					   &tb45_sms_result_poll_work,
					   tb45_sms_result_poll_events,
					   ARRAY_SIZE(tb45_sms_result_poll_events),
					   K_FOREVER);
}

static void tb45_sms_result_worker(struct k_work *work)
{
	ARG_UNUSED(work);

	while (true) {
		struct tb45_sms_result result;
		int ret = tb45_sms_result_wait(&result, K_NO_WAIT);
		if ((ret == -EAGAIN) || (ret == -EBUSY)) {
			break;
		}

		if (ret != 0) {
			LOG_ERR("SMS_SND result wait failed (%d)", ret);
			break;
		}

		struct tb45_sms_event event = {
			.type = TB45_SMS_EVENT_TYPE_SEND_MSG_STATUS,
			.status = result.result,
			.timestamp_ms = (uint32_t)k_uptime_get_32(),
		};
		event.send.message_id = result.request_id;
		(void)snprintf(event.send.phone, sizeof(event.send.phone), "%s", result.phone);
		tb45_sms_event_publish(&event);
	}

	int ret = tb45_sms_event_submit_result_poll();
	if ((ret != 0) && (ret != -EAGAIN) && (ret != -EADDRINUSE)) {
		LOG_ERR("SMS_EVT result poll submit failed (%d)", ret);
	}
}
#endif

int tb45_sms_event_init(void)
{
	if (!atomic_cas(&tb45_sms_event_started, 0, 1)) {
		return 0;
	}

	k_work_poll_init(&tb45_sms_event_dispatch_poll_work, tb45_sms_event_dispatch_worker);

	int ret = tb45_sms_event_submit_dispatch_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("SMS_EVT init dispatch submit failed (%d)", ret);
	}

#if defined(CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE) && CONFIG_APP_TB45_SMS_RESULT_WORKER_ENABLE
	k_work_poll_init(&tb45_sms_result_poll_work, tb45_sms_result_worker);

	ret = tb45_sms_event_submit_result_poll();
	if ((ret != 0) && (ret != -EAGAIN)) {
		LOG_ERR("SMS_EVT init result submit failed (%d)", ret);
	}
#endif

	return 0;
}

int tb45_sms_event_set_callback(tb45_sms_event_cb_t cb, void *user_data)
{
	k_mutex_lock(&tb45_sms_event_cb_lock, K_FOREVER);
	tb45_sms_event_cb = cb;
	tb45_sms_event_cb_user_data = user_data;
	k_mutex_unlock(&tb45_sms_event_cb_lock);
	return 0;
}

void tb45_sms_event_on_serial_rx(void)
{
#if defined(CONFIG_APP_TB45_SMS_TRIGGER_EVENT_PATH) && CONFIG_APP_TB45_SMS_TRIGGER_EVENT_PATH
	(void)tb45_sms_receive_trigger_scan();

	const uint32_t now = (uint32_t)k_uptime_get_32();
	const uint32_t last = (uint32_t)atomic_get(&tb45_sms_last_serial_rx_ms);

	if ((now - last) < TB45_SMS_SERIAL_RX_DEBOUNCE_MS) {
		return;
	}

	atomic_set(&tb45_sms_last_serial_rx_ms, (atomic_val_t)now);

	struct tb45_sms_event event = {
		.type = TB45_SMS_EVENT_TYPE_RECEIVE_SERIAL_STATUS,
		.timestamp_ms = now,
	};
	tb45_sms_event_publish(&event);
#endif
}

void tb45_sms_event_on_cmti(uint16_t storage_index)
{
	if (storage_index == 0U) {
		return;
	}

	(void)tb45_sms_receive_trigger_index(storage_index);
}

void tb45_sms_event_notify_enqueue(const struct tb45_sms_request *request, int status)
{
	struct tb45_sms_event event = {
		.type = TB45_SMS_EVENT_TYPE_ENQUEUE_MSG_STATUS,
		.status = status,
		.timestamp_ms = (uint32_t)k_uptime_get_32(),
	};

	if (request != NULL) {
		event.send.message_id = request->message_id;
		(void)snprintf(event.send.phone, sizeof(event.send.phone), "%s",
			       request->phone_number);
	}

	tb45_sms_event_publish(&event);
}

void tb45_sms_event_notify_rx_message(const struct tb45_sms_rx_message *message, int status)
{
	struct tb45_sms_event event = {
		.type = TB45_SMS_EVENT_TYPE_RECEIVE_MSG_OUTPUT,
		.status = status,
		.timestamp_ms = (uint32_t)k_uptime_get_32(),
	};

	if (message != NULL) {
		event.receive.storage_index = message->storage_index;
		(void)snprintf(event.receive.phone, sizeof(event.receive.phone), "%s", message->phone);
		(void)snprintf(event.receive.date_time, sizeof(event.receive.date_time), "%s",
			       message->timestamp);
		(void)snprintf(event.receive.message, sizeof(event.receive.message), "%s",
			       message->message);
	}

	tb45_sms_event_publish(&event);
}
