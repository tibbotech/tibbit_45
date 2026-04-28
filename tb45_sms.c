#include "tb45_sms.h"
#include "tb45_sms_at_helper.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tb45_sms, CONFIG_LOG_DEFAULT_LEVEL);

#define TB45_SMS_PHONE_MAX_LEN          24
#define TB45_SMS_TEXT_MAX_LEN           CONFIG_APP_TB45_SMS_TEXT_MAX_LEN
#define TB45_SMS_TIMEOUT_SEND_FINAL_MS  90000
#define TB45_SMS_INIT_WAIT_MS           10000
#define TB45_SMS_INIT_AT_TIMEOUT_MS     10000

static void tb45_sms_print_usage(const struct shell *sh)
{
	if (sh == NULL) {
		return;
	}

	shell_print(sh, "Usage: tb45 sms send <phone> <text>");
	shell_print(sh, "Example: tb45 sms send \"+886123456789\" \"hello\"");
}

static bool tb45_sms_phone_is_valid(const char *phone)
{
	size_t i = 0;
	size_t len;

	if (phone == NULL) {
		return false;
	}

	len = strlen(phone);
	if ((len < 3U) || (len > TB45_SMS_PHONE_MAX_LEN)) {
		return false;
	}

	if (phone[0] == '+') {
		if (len < 4U) {
			return false;
		}
		i = 1U;
	}

	for (; i < len; i++) {
		if (!isdigit((unsigned char)phone[i])) {
			return false;
		}
	}

	return true;
}

static bool tb45_sms_text_is_valid(const char *text, size_t *out_len)
{
	size_t i;
	size_t len;

	if (text == NULL) {
		return false;
	}

	len = strlen(text);
	if ((len == 0U) || (len > TB45_SMS_TEXT_MAX_LEN)) {
		return false;
	}

	for (i = 0U; i < len; i++) {
		unsigned char ch = (unsigned char)text[i];

		if ((ch < 0x20U) || (ch == 0x7FU)) {
			return false;
		}
	}

	if (out_len != NULL) {
		*out_len = len;
	}

	return true;
}

int tb45_sms_send(const struct shell *sh, const char *phone, const char *text)
{
	int ret;
	char cmgs_cmd[48];

	if (!tb45_sms_phone_is_valid(phone)) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	if (!tb45_sms_text_is_valid(text, NULL)) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	ret = snprintf(cmgs_cmd, sizeof(cmgs_cmd), "AT+CMGS=\"%s\"", phone);
	if ((ret < 0) || ((size_t)ret >= sizeof(cmgs_cmd))) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	ret = tb45_sms_at_send_text_raw(sh, cmgs_cmd, text, TB45_SMS_TIMEOUT_SEND_FINAL_MS);
	if (ret < 0) {
		if (sh != NULL) {
			shell_error(sh, "SMS send failed (%d)", ret);
		}
		return ret;
	}

	return 0;
}

/* --- Shell Command Handlers --- */

static int cmd_tb45_sms_send(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3U) {
		tb45_sms_print_usage(sh);
		return -EINVAL;
	}

	/* argv[1] = phone, argv[2] = text */
	int ret = tb45_sms_send(sh, argv[1], argv[2]);

	if (ret == 0) {
		shell_print(sh, "SMS sent successfully to %s", argv[1]);
	}

	return ret;
}

static int cmd_tb45_sms_init(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argv);

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 sms init");
		return -EINVAL;
	}

	shell_print(sh, "SMS init: step 1/7: tb45 ppp down");
	ret = shell_execute_cmd(sh, "tb45 ppp down");
	if (ret < 0) {
		shell_error(sh, "SMS init failed at 'tb45 ppp down' (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init: step 2/7: wait %d seconds", TB45_SMS_INIT_WAIT_MS / 1000);
	k_msleep(TB45_SMS_INIT_WAIT_MS);

	shell_print(sh, "SMS init: step 3/7: AT");
	ret = tb45_sms_at_run(sh, "AT", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "SMS init failed at AT (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init: step 4/7: AT+CPIN?");
	ret = tb45_sms_at_run(sh, "AT+CPIN?", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "SMS init failed at AT+CPIN? (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init: step 5/7: AT+CGSMS?");
	ret = tb45_sms_at_run(sh, "AT+CGSMS?", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "SMS init failed at AT+CGSMS? (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init: step 6/7: AT+CGSMS=3");
	ret = tb45_sms_at_run(sh, "AT+CGSMS=3", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "SMS init failed at AT+CGSMS=3 (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init: step 7/7: AT+CNMP=2");
	ret = tb45_sms_at_run(sh, "AT+CNMP=2", "OK", TB45_SMS_INIT_AT_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "SMS init failed at AT+CNMP=2 (%d)", ret);
		return ret;
	}

	shell_print(sh, "SMS init complete");
	return 0;
}

static const struct shell_static_entry tb45_sms_subcmds[] = {
	SHELL_CMD_ARG(send, NULL, "Send SMS: tb45 sms send <phone> <text>",
		      cmd_tb45_sms_send, 1, 255),
	SHELL_CMD_ARG(init, NULL,
		      "Temporary SMS init: ppp down + wait + AT/CPIN/CGSMS/CNMP setup",
		      cmd_tb45_sms_init, 1, 0),
	SHELL_SUBCMD_SET_END,
};

const union shell_cmd_entry tb45_sms_cmds = {
	.entry = tb45_sms_subcmds,
};
