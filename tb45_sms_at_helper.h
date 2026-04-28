#ifndef TB45_SMS_AT_HELPER_H_
#define TB45_SMS_AT_HELPER_H_

struct shell;

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

#endif /* TB45_SMS_AT_HELPER_H_ */
