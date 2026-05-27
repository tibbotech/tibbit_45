#ifndef TB45_SMS_EVENT_H_
#define TB45_SMS_EVENT_H_

#include <stdint.h>

#include "tb45_sms.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tb45_sms_event_type {
	TB45_SMS_EVENT_TYPE_RECEIVE_SERIAL_STATUS = 0,
	TB45_SMS_EVENT_TYPE_RECEIVE_MSG_OUTPUT = 1,
	TB45_SMS_EVENT_TYPE_ENQUEUE_MSG_STATUS = 2,
	TB45_SMS_EVENT_TYPE_SEND_MSG_STATUS = 3,
};

struct tb45_sms_event {
	enum tb45_sms_event_type type;
	uint32_t timestamp_ms;
	int status;
	struct {
		uint32_t message_id;
		char phone[CONFIG_APP_TB45_SMS_PHONE_MAX_LEN + 1];
	} send;
	struct {
		uint16_t storage_index;
		char phone[CONFIG_APP_TB45_SMS_PHONE_MAX_LEN + 1];
		char date_time[24];
		char message[CONFIG_APP_TB45_SMS_TEXT_MAX_LEN + 1];
	} receive;
};


typedef void (*tb45_sms_event_cb_t)(const struct tb45_sms_event *event, void *user_data);

int tb45_sms_event_init(void);
int tb45_sms_event_set_callback(tb45_sms_event_cb_t cb, void *user_data);

void tb45_sms_event_on_serial_rx(void);
void tb45_sms_event_on_cmti(uint16_t storage_index);
void tb45_sms_event_notify_enqueue(const struct tb45_sms_request *request, int status);
void tb45_sms_event_notify_rx_message(const struct tb45_sms_rx_message *message, int status);

#ifdef __cplusplus
}
#endif

#endif /* TB45_SMS_EVENT_H_ */
