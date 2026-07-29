#ifndef TB45_SMS_AT_HELPER_H_
#define TB45_SMS_AT_HELPER_H_

#include <stddef.h>

struct shell;
typedef void (*tb45_sms_at_complete_cb_t)(int ret, void *user_data);

/**
 * @brief Run one modem AT command synchronously through the modem user pipe.
 *
 * @param sh Shell context used for response printing. May be NULL.
 * @param request AT request string.
 * @param expected_response Expected response terminator. Defaults to "OK" when NULL.
 * @param timeout_ms Max time to wait for completion in milliseconds.
 *
 * @retval 0 On success.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe script is running.
 * @retval -ETIMEDOUT Command timed out.
 * @retval -EIO Command aborted (for example modem returned ERROR).
 */
int tb45_sms_at_run(const struct shell *sh, const char *request, const char *expected_response,
		    int timeout_ms);

/**
 * @brief Run one modem AT command asynchronously through the modem user pipe.
 *
 * Completion is reported through @p complete_cb when provided. Only one helper
 * operation may be active at a time because the modem user pipe is shared.
 *
 * @param sh Shell context used for response printing. May be NULL.
 * @param request AT request string.
 * @param expected_response Expected response terminator. Defaults to "OK" when NULL.
 * @param timeout_ms Max time to wait for completion in milliseconds.
 * @param complete_cb Optional completion callback invoked when the script finishes.
 * @param user_data Opaque user data passed to @p complete_cb.
 *
 * @retval 0 On successful start.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe script is running.
 * @retval -EIO Failed to start the script.
 */
int tb45_sms_at_run_async_cb(const struct shell *sh, const char *request,
			     const char *expected_response, int timeout_ms,
			     tb45_sms_at_complete_cb_t complete_cb, void *user_data);

/**
 * @brief Run one modem AT command asynchronously and capture response lines.
 *
 * Captured output is line-oriented, null-terminated when @p out_buf is
 * provided, and completed before @p complete_cb is invoked.
 *
 * @param sh Shell context used for response printing. May be NULL.
 * @param request AT request string.
 * @param expected_response Expected response terminator. Defaults to "OK" when NULL.
 * @param timeout_ms Max time to wait for completion in milliseconds.
 * @param out_buf Optional output buffer for response capture.
 * @param out_buf_size Output buffer size in bytes.
 * @param complete_cb Optional completion callback invoked when the script finishes.
 * @param user_data Opaque user data passed to @p complete_cb.
 *
 * @retval 0 On successful start.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe script is running.
 * @retval -EIO Failed to start the script.
 */
int tb45_sms_at_run_async_capture_cb(const struct shell *sh, const char *request,
				     const char *expected_response, int timeout_ms,
				     char *out_buf, size_t out_buf_size,
				     tb45_sms_at_complete_cb_t complete_cb, void *user_data);

/**
 * @brief Send one text SMS asynchronously on the low-priority worker path.
 *
 * This is intended for non-shell async dispatch. The helper copies `cmgs_cmd`
 * and `text`, performs the raw pipe exchange without blocking the shared work
 * queue between polls, and reports final completion through @p complete_cb.
 *
 * @param cmgs_cmd CMGS command string, e.g. `AT+CMGS="+123456789"`.
 * @param text SMS message body.
 * @param submit_timeout_ms Final SMS submit timeout in milliseconds.
 * @param complete_cb Optional completion callback invoked when the flow finishes.
 * @param user_data Opaque user data passed to @p complete_cb.
 *
 * @retval 0 On successful start.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe raw send is already active.
 * @retval -EAGAIN Low-priority workqueue is not ready.
 * @retval -EIO Failed to start the raw send flow.
 */
int tb45_sms_at_send_text_raw_async_cb(const char *cmgs_cmd, const char *text,
				       int submit_timeout_ms,
				       tb45_sms_at_complete_cb_t complete_cb,
				       void *user_data);

/**
 * @brief Send one text SMS in a single raw user-pipe session.
 *
 * This API performs:
 * 1) send `AT+CMGF=1` and wait `OK`
 * 2) send `AT+CSCS="GSM"` and wait `OK`
 * 3) send `cmgs_cmd` plus CR and wait `> ` prompt
 * 4) send `text` plus Ctrl+Z (0x1A) with no trailing CR
 * 5) wait for modem final `+CMGS:` and `OK`
 *
 * @param sh Shell context used for response printing. May be NULL.
 * @param cmgs_cmd CMGS command string, e.g. `AT+CMGS="+123456789"`.
 * @param text SMS message body.
 * @param submit_timeout_ms Final SMS submit timeout in milliseconds.
 *
 * @retval 0 On success.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe script is running.
 * @retval -ETIMEDOUT Prompt/final response timed out.
 * @retval -EIO Modem returned ERROR or protocol state was invalid.
 */
int tb45_sms_at_send_text_raw(const struct shell *sh, const char *cmgs_cmd, const char *text,
			      int submit_timeout_ms);

/**
 * @brief Execute one AT command and capture response until final OK.
 *
 * Captured output contains the raw modem response bytes received before
 * completion and is always null-terminated when @p out_buf is provided.
 *
 * @param request AT request string.
 * @param out_buf Optional output buffer for response capture.
 * @param out_buf_size Output buffer size in bytes.
 * @param timeout_ms Command timeout in milliseconds.
 *
 * @retval 0 On success.
 * @retval -EINVAL Invalid arguments.
 * @retval -EPERM Modem user pipe is not ready.
 * @retval -EBUSY Another user-pipe script is running.
 * @retval -ETIMEDOUT Command timed out.
 * @retval -EIO Modem returned ERROR.
 */
int tb45_sms_at_exec_capture(const char *request, char *out_buf, size_t out_buf_size, int timeout_ms);

#endif /* TB45_SMS_AT_HELPER_H_ */
