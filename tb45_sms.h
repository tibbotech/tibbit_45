#ifndef TB45_SMS_H_
#define TB45_SMS_H_

#include <stdint.h>

#include <zephyr/kernel.h>

struct shell;

#ifdef __cplusplus
extern "C" {
#endif

struct tb45_sms_request {
	char phone_number[CONFIG_APP_TB45_SMS_PHONE_MAX_LEN + 1];
	char message[CONFIG_APP_TB45_SMS_TEXT_MAX_LEN + 1];
	uint32_t message_id;
};

struct tb45_sms_result {
	uint32_t request_id;
	int result;
	char phone[CONFIG_APP_TB45_SMS_PHONE_MAX_LEN + 1];
};

struct tb45_sms_rx_message {
	uint16_t storage_index;
	char timestamp[24];
	char phone[CONFIG_APP_TB45_SMS_PHONE_MAX_LEN + 1];
	char message[CONFIG_APP_TB45_SMS_TEXT_MAX_LEN + 1];
};

struct tb45_sms_stats {
	uint32_t rx_setup_done;
	uint32_t rx_cleanup_done;
	uint32_t rx_init_completed_logged;
	uint32_t rx_init_retry_count;
	uint32_t rx_trigger_queue_used;
	uint32_t rx_result_queue_used;
	uint32_t async_result_queue_used;
	uint32_t rx_processed_count;
	uint32_t rx_scan_fail_count;
	uint32_t rx_cmgr_fail_count;
	uint32_t rx_parse_fail_count;
	uint32_t rx_delete_fail_count;
	uint32_t rx_trigger_queue_drop_count;
	uint32_t rx_result_queue_drop_count;
	uint32_t async_result_queue_drop_count;
};

/**
 * @brief Send an SMS message via SIM7500.
 * * @param sh Shell context for logging (can be NULL).
 * @param request Canonical SMS request payload.
 * @return 0 on success, negative error code on failure.
 */
int tb45_sms_send(const struct shell *sh, const struct tb45_sms_request *request);

/**
 * @brief Queue SMS sending on TB45 SMS internal worker thread.
 *
 * This is intended for callers outside shell context (e.g. main.cpp callbacks).
 * The request is copied into an internal queue and sent asynchronously.
 *
 * @param request Canonical SMS request payload.
 * @return 0 when queued successfully, negative error code on failure.
 */
int tb45_sms_send_enqueue(const struct tb45_sms_request *request);

/**
 * @brief Queue SMS send asynchronously (non-blocking caller path).
 *
 * This is non-blocking for the caller and suitable for PPP/event callbacks.
 * Final send result is handled internally by the SMS worker logs.
 *
 * @param request Canonical SMS request payload.
 * @return 0 when queued successfully, negative error code on failure.
 */
int tb45_sms_send_enqueue_with_result_cb(const struct tb45_sms_request *request);

/**
 * @brief Queue SMS send asynchronously with caller-provided request id.
 *
 * This is non-blocking for the caller and suitable for PPP/event callbacks.
 * Final send result can be collected later via tb45_sms_result_wait/get APIs.
 *
 * @param request Canonical SMS request payload.
 *                `request->message_id` must be non-zero.
 * @return 0 when queued successfully, negative error code on failure.
 */
int tb45_sms_send_enqueue_with_result_id(const struct tb45_sms_request *request);

/**
 * @brief Wait for the next completed SMS async result.
 *
 * @param out_result Output structure for completed result.
 * @param timeout Wait timeout.
 * @return 0 on success, -EAGAIN on timeout, negative error code otherwise.
 */
int tb45_sms_result_wait(struct tb45_sms_result *out_result, k_timeout_t timeout);

/**
 * @brief Get and consume a completed SMS async result by request id.
 *
 * @param request_id Request id to retrieve.
 * @param out_result Output structure for completed result.
 * @return 0 on success, -ENOENT when result for id is not available yet.
 */
int tb45_sms_result_get(uint32_t request_id, struct tb45_sms_result *out_result);

/**
 * @brief Queue SMS send and wait for final send result.
 *
 * The send is still executed on the async worker thread, but this call blocks
 * until the worker finishes the SMS flow (including retries).
 *
 * @param request Canonical SMS request payload.
 * @return 0 on final send success, negative error code on final send failure.
 */
int tb45_sms_send_enqueue_wait(const struct tb45_sms_request *request);

/**
 * @brief Global convenience API that returns the real SMS send result.
 *
 * Intended for non-shell callers that need end-to-end success/failure status.
 * Internally this uses the async worker path and waits for completion.
 *
 * @param request Canonical SMS request payload.
 * @return 0 on final send success, negative error code on final send failure.
 */
int tb45_sms_send_global(const struct tb45_sms_request *request);

/**
 * @brief Trigger SMS receive scan from modem storage.
 *
 * This is non-blocking. The receive worker will fetch unread messages and
 * queue parsed results for tb45_sms_receive_wait() and event callbacks.
 *
 * @return 0 when trigger accepted, negative error code otherwise.
 */
int tb45_sms_receive_trigger_scan(void);

/**
 * @brief Trigger SMS receive for one known modem storage index.
 *
 * This is non-blocking. The receive worker will run `AT+CMGR=<index>` and
 * process the message if present.
 *
 * @param storage_index Modem storage index from `+CMTI`.
 * @return 0 when trigger accepted, negative error code otherwise.
 */
int tb45_sms_receive_trigger_index(uint16_t storage_index);

/**
 * @brief Re-arm SMS receive configuration after modem reconnect.
 *
 * This clears one-time RX setup state so the worker reapplies modem-side SMS
 * receive config (for example `AT+CPMS` and `AT+CNMI`) and then performs an
 * unread scan.
 *
 * @return 0 when trigger accepted, negative error code otherwise.
 */
int tb45_sms_receive_recover_after_modem_reconnect(void);

/**
 * @brief Wait for next received SMS message.
 *
 * @param out_message Output receive message.
 * @param timeout Wait timeout.
 * @return 0 on success, -EAGAIN on timeout, negative error code otherwise.
 */
int tb45_sms_receive_wait(struct tb45_sms_rx_message *out_message, k_timeout_t timeout);

/**
 * @brief Read SMS runtime counters and queue usage.
 *
 * @param out_stats Output statistics snapshot.
 * @return 0 on success, negative error code on failure.
 */
int tb45_sms_get_stats(struct tb45_sms_stats *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* TB45_SMS_H_ */
