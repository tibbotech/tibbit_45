#include "tb45_sms_at_helper.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/modem/at/user_pipe.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/pipelink.h>
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#endif
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(tb45_sms_at_helper, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_SHELL
#define TB45_SHELL_PRINT(_sh, ...) shell_print((_sh), __VA_ARGS__)
#define TB45_SHELL_ERROR(_sh, ...) shell_error((_sh), __VA_ARGS__)
#else
#define TB45_SHELL_PRINT(_sh, ...) \
	do {                       \
		ARG_UNUSED(_sh);   \
	} while (0)
#define TB45_SHELL_ERROR(_sh, ...) \
	do {                       \
		ARG_UNUSED(_sh);   \
	} while (0)
#endif

#define TB45_SMS_AT_CHAT_RX_BUF_SIZE    256
#define TB45_SMS_AT_POLL_INTERVAL_MS    10
#define TB45_SMS_AT_DEFAULT_TIMEOUT_MS  5000
#define TB45_SMS_AT_MIN_TIMEOUT_MS      100
#define TB45_SMS_AT_MAX_TIMEOUT_MS      600000
#define TB45_SMS_AT_SETUP_TIMEOUT_MS    10000
#define TB45_SMS_AT_PROMPT_TIMEOUT_MS   10000
#define TB45_SMS_AT_RX_BUF_SIZE         768
#define TB45_SMS_AT_RX_CHUNK_SIZE       128

#define TB45_SMS_AT_MODEM_NODE      DT_ALIAS(modem)
#define TB45_SMS_AT_PIPELINK_NAME   _CONCAT(user_pipe_, CONFIG_MODEM_AT_USER_PIPE_IDX)

MODEM_PIPELINK_DT_DECLARE(TB45_SMS_AT_MODEM_NODE, TB45_SMS_AT_PIPELINK_NAME);
static struct modem_pipelink *tb45_sms_at_pipelink =
	MODEM_PIPELINK_DT_GET(TB45_SMS_AT_MODEM_NODE, TB45_SMS_AT_PIPELINK_NAME);

static struct modem_chat tb45_sms_at_chat;
static uint8_t tb45_sms_at_chat_receive_buf[TB45_SMS_AT_CHAT_RX_BUF_SIZE];
static uint8_t *tb45_sms_at_chat_argv_buf[2];
static struct modem_chat_script_chat tb45_sms_at_script_chat[1];
static struct modem_chat_match tb45_sms_at_script_chat_matches[2];
static struct modem_chat_script tb45_sms_at_script;
static const struct shell *tb45_sms_at_active_shell;
static enum modem_chat_script_result tb45_sms_at_last_result;

static void tb45_sms_at_print_any_match(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	if ((tb45_sms_at_active_shell == NULL) || (argc != 2)) {
		return;
	}

	TB45_SHELL_PRINT(tb45_sms_at_active_shell, "%s", argv[1]);
}

static void tb45_sms_at_print_match(struct modem_chat *chat, char **argv, uint16_t argc,
				    void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	if ((tb45_sms_at_active_shell == NULL) || (argc != 1)) {
		return;
	}

	TB45_SHELL_PRINT(tb45_sms_at_active_shell, "%s", argv[0]);
}

MODEM_CHAT_MATCHES_DEFINE(
	tb45_sms_at_abort_matches,
	MODEM_CHAT_MATCH("ERROR", "", tb45_sms_at_print_match),
	MODEM_CHAT_MATCH_WILDCARD("+CME ERROR:*", "", tb45_sms_at_print_match),
	MODEM_CHAT_MATCH_WILDCARD("+CMS ERROR:*", "", tb45_sms_at_print_match),
);

static void tb45_sms_at_script_callback(struct modem_chat *chat,
					enum modem_chat_script_result result,
					void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	tb45_sms_at_last_result = result;
	tb45_sms_at_active_shell = NULL;
	modem_at_user_pipe_release();
}

static void tb45_sms_at_init_chat(void)
{
	const struct modem_chat_config chat_cfg = {
		.receive_buf = tb45_sms_at_chat_receive_buf,
		.receive_buf_size = sizeof(tb45_sms_at_chat_receive_buf),
		.delimiter = "\r",
		.delimiter_size = sizeof("\r") - 1,
		.filter = "\n",
		.filter_size = sizeof("\n") - 1,
		.argv = tb45_sms_at_chat_argv_buf,
		.argv_size = ARRAY_SIZE(tb45_sms_at_chat_argv_buf),
	};

	modem_chat_init(&tb45_sms_at_chat, &chat_cfg);
}

static void tb45_sms_at_init_script(void)
{
	modem_chat_match_init(&tb45_sms_at_script_chat_matches[0]);
	(void)modem_chat_match_set_match(&tb45_sms_at_script_chat_matches[0], "");
	(void)modem_chat_match_set_separators(&tb45_sms_at_script_chat_matches[0], "");
	modem_chat_match_set_callback(&tb45_sms_at_script_chat_matches[0], tb45_sms_at_print_any_match);
	modem_chat_match_set_partial(&tb45_sms_at_script_chat_matches[0], true);
	modem_chat_match_enable_wildcards(&tb45_sms_at_script_chat_matches[0], false);

	modem_chat_match_init(&tb45_sms_at_script_chat_matches[1]);
	(void)modem_chat_match_set_match(&tb45_sms_at_script_chat_matches[1], "OK");
	(void)modem_chat_match_set_separators(&tb45_sms_at_script_chat_matches[1], "");
	modem_chat_match_set_callback(&tb45_sms_at_script_chat_matches[1], tb45_sms_at_print_match);
	modem_chat_match_set_partial(&tb45_sms_at_script_chat_matches[1], false);
	modem_chat_match_enable_wildcards(&tb45_sms_at_script_chat_matches[1], false);

	modem_chat_script_chat_init(tb45_sms_at_script_chat);
	(void)modem_chat_script_chat_set_response_matches(tb45_sms_at_script_chat,
							  tb45_sms_at_script_chat_matches,
							  ARRAY_SIZE(tb45_sms_at_script_chat_matches));
	modem_chat_script_chat_set_timeout(tb45_sms_at_script_chat,
					   TB45_SMS_AT_DEFAULT_TIMEOUT_MS);

	modem_chat_script_init(&tb45_sms_at_script);
	modem_chat_script_set_name(&tb45_sms_at_script, "tb45_sms_at_script");
	(void)modem_chat_script_set_script_chats(&tb45_sms_at_script, tb45_sms_at_script_chat,
						 ARRAY_SIZE(tb45_sms_at_script_chat));
	(void)modem_chat_script_set_abort_matches(&tb45_sms_at_script, tb45_sms_at_abort_matches,
						  ARRAY_SIZE(tb45_sms_at_abort_matches));
	modem_chat_script_set_callback(&tb45_sms_at_script, tb45_sms_at_script_callback);
	modem_chat_script_set_timeout(&tb45_sms_at_script,
				      (TB45_SMS_AT_DEFAULT_TIMEOUT_MS + 999) / 1000);
}

static int tb45_sms_at_init(void)
{
	tb45_sms_at_init_chat();
	tb45_sms_at_init_script();
	tb45_sms_at_last_result = MODEM_CHAT_SCRIPT_RESULT_ABORT;
	modem_at_user_pipe_init(&tb45_sms_at_chat);
	return 0;
}

SYS_INIT(tb45_sms_at_init, POST_KERNEL, 99);

static int tb45_sms_at_run_async(const struct shell *sh, const char *request,
				 const char *expected_response, int timeout_ms)
{
	int ret;
	const char *expected = (expected_response != NULL) ? expected_response : "OK";
	uint16_t chat_timeout_ms;
	uint32_t script_timeout_s;

	if ((request == NULL) || (timeout_ms < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
	    (timeout_ms > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
		return -EINVAL;
	}

	ret = modem_at_user_pipe_claim();
	if (ret < 0) {
		if (sh != NULL) {
			if (ret == -EPERM) {
				TB45_SHELL_ERROR(sh, "modem is not ready");
			} else if (ret == -EBUSY) {
				TB45_SHELL_ERROR(sh, "AT channel is busy");
			} else {
				TB45_SHELL_ERROR(sh, "AT channel unavailable (%d)", ret);
			}
		}
		return ret;
	}

	ret = modem_chat_script_chat_set_request(tb45_sms_at_script_chat, request);
	if (ret < 0) {
		modem_at_user_pipe_release();
		return -EINVAL;
	}

	ret = modem_chat_match_set_match(&tb45_sms_at_script_chat_matches[1], expected);
	if (ret < 0) {
		modem_at_user_pipe_release();
		return -EINVAL;
	}

	tb45_sms_at_active_shell = sh;
	tb45_sms_at_last_result = MODEM_CHAT_SCRIPT_RESULT_ABORT;
	chat_timeout_ms = (timeout_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)timeout_ms;
	script_timeout_s = (timeout_ms + 999U) / 1000U;
	modem_chat_script_chat_set_timeout(tb45_sms_at_script_chat, chat_timeout_ms);
	modem_chat_script_set_timeout(&tb45_sms_at_script, script_timeout_s);

	ret = modem_chat_run_script_async(&tb45_sms_at_chat, &tb45_sms_at_script);
	if (ret < 0) {
		tb45_sms_at_active_shell = NULL;
		modem_at_user_pipe_release();
		if (sh != NULL) {
			TB45_SHELL_ERROR(sh, "failed to start AT script (%d)", ret);
		}
	}

	return ret;
}

int tb45_sms_at_run(const struct shell *sh, const char *request, const char *expected_response,
		    int timeout_ms)
{
	int ret;
	int waited_ms = 0;

	ret = tb45_sms_at_run_async(sh, request, expected_response, timeout_ms);
	if (ret < 0) {
		return ret;
	}

	while (waited_ms < timeout_ms) {
		ret = modem_at_user_pipe_claim();
		if (ret == -EBUSY) {
			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			waited_ms += TB45_SMS_AT_POLL_INTERVAL_MS;
			continue;
		}

		if (ret < 0) {
			return ret;
		}

		modem_at_user_pipe_release();

		if (tb45_sms_at_last_result == MODEM_CHAT_SCRIPT_RESULT_SUCCESS) {
			return 0;
		}

		if (tb45_sms_at_last_result == MODEM_CHAT_SCRIPT_RESULT_TIMEOUT) {
			return -ETIMEDOUT;
		}

		return -EIO;
	}

	return -ETIMEDOUT;
}

static bool tb45_sms_at_buf_contains(const uint8_t *buf, size_t len, const char *needle)
{
	size_t needle_len = strlen(needle);
	size_t i;

	if ((needle_len == 0U) || (len < needle_len)) {
		return false;
	}

	for (i = 0U; i <= (len - needle_len); i++) {
		if (memcmp(&buf[i], needle, needle_len) == 0) {
			return true;
		}
	}

	return false;
}

static void tb45_sms_at_buf_append(uint8_t *dst, size_t dst_cap, size_t *dst_len,
				   const uint8_t *src, size_t src_len)
{
	size_t keep;

	if (src_len >= dst_cap) {
		memcpy(dst, &src[src_len - dst_cap], dst_cap);
		*dst_len = dst_cap;
		return;
	}

	if ((*dst_len + src_len) > dst_cap) {
		keep = dst_cap - src_len;
		memmove(dst, &dst[*dst_len - keep], keep);
		*dst_len = keep;
	}

	memcpy(&dst[*dst_len], src, src_len);
	*dst_len += src_len;
}

static bool tb45_sms_at_has_error(const uint8_t *buf, size_t len)
{
	return tb45_sms_at_buf_contains(buf, len, "ERROR") ||
	       tb45_sms_at_buf_contains(buf, len, "+CME ERROR") ||
	       tb45_sms_at_buf_contains(buf, len, "+CMS ERROR");
}

static int tb45_sms_pipe_transmit_all(struct modem_pipe *pipe, const uint8_t *buf, size_t len,
				      int timeout_ms)
{
	int ret;
	size_t sent = 0U;
	int64_t deadline_ms;

	deadline_ms = k_uptime_get() + timeout_ms;

	while (sent < len) {
		ret = modem_pipe_transmit(pipe, &buf[sent], len - sent);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			if (k_uptime_get() >= deadline_ms) {
				return -ETIMEDOUT;
			}

			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			continue;
		}

		sent += (size_t)ret;
	}

	return 0;
}

static int tb45_sms_pipe_drain_rx(struct modem_pipe *pipe)
{
	int ret;
	int loops = 0;
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	while (loops++ < 64) {
		ret = modem_pipe_receive(pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return 0;
		}
	}

	return 0;
}

static int tb45_sms_pipe_send_line(struct modem_pipe *pipe, const char *line, int timeout_ms)
{
	int ret;
	uint8_t cr = '\r';

	if (line == NULL) {
		return -EINVAL;
	}

	ret = tb45_sms_pipe_transmit_all(pipe, (const uint8_t *)line, strlen(line), timeout_ms);
	if (ret < 0) {
		return ret;
	}

	return tb45_sms_pipe_transmit_all(pipe, &cr, 1U, timeout_ms);
}

static int tb45_sms_pipe_wait_for_ok(struct modem_pipe *pipe, const char *required_token,
				     int timeout_ms)
{
	int ret;
	int64_t deadline_ms;
	size_t rx_len = 0U;
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	deadline_ms = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline_ms) {
		ret = modem_pipe_receive(pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			continue;
		}

		tb45_sms_at_buf_append(rx_buf, sizeof(rx_buf), &rx_len, chunk, (size_t)ret);

		if (tb45_sms_at_has_error(rx_buf, rx_len)) {
			return -EIO;
		}

		if (tb45_sms_at_buf_contains(rx_buf, rx_len, "\r\nOK\r\n")) {
			if ((required_token != NULL) &&
			    !tb45_sms_at_buf_contains(rx_buf, rx_len, required_token)) {
				return -EIO;
			}
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int tb45_sms_pipe_wait_for_prompt(struct modem_pipe *pipe, int timeout_ms)
{
	int ret;
	int64_t deadline_ms;
	size_t rx_len = 0U;
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	deadline_ms = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline_ms) {
		ret = modem_pipe_receive(pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			continue;
		}

		tb45_sms_at_buf_append(rx_buf, sizeof(rx_buf), &rx_len, chunk, (size_t)ret);

		if (tb45_sms_at_has_error(rx_buf, rx_len)) {
			return -EIO;
		}

		if (tb45_sms_at_buf_contains(rx_buf, rx_len, "> ")) {
			return 0;
		}

		if (tb45_sms_at_buf_contains(rx_buf, rx_len, ">")) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int tb45_sms_pipe_wait_for_sms_result(struct modem_pipe *pipe, int timeout_ms)
{
	int ret;
	int64_t deadline_ms;
	bool seen_cmgs = false;
	bool seen_ok = false;
	size_t rx_len = 0U;
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	deadline_ms = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline_ms) {
		ret = modem_pipe_receive(pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			continue;
		}

		tb45_sms_at_buf_append(rx_buf, sizeof(rx_buf), &rx_len, chunk, (size_t)ret);

		if (tb45_sms_at_has_error(rx_buf, rx_len)) {
			return -EIO;
		}

		if (!seen_cmgs && tb45_sms_at_buf_contains(rx_buf, rx_len, "+CMGS:")) {
			seen_cmgs = true;
		}

		if (!seen_ok && tb45_sms_at_buf_contains(rx_buf, rx_len, "\r\nOK\r\n")) {
			seen_ok = true;
		}

		if (seen_cmgs && seen_ok) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

int tb45_sms_at_send_text_raw(const struct shell *sh, const char *cmgs_cmd, const char *text,
			      int submit_timeout_ms)
{
	int ret;
	int attach_ret = 0;
	size_t text_len;
	uint8_t ctrl_z = 0x1A;
	struct modem_pipe *pipe;

	if ((cmgs_cmd == NULL) || (text == NULL) ||
	    (submit_timeout_ms < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
	    (submit_timeout_ms > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
		return -EINVAL;
	}

	pipe = modem_pipelink_get_pipe(tb45_sms_at_pipelink);
	if (pipe == NULL) {
		return -ENODEV;
	}

	ret = modem_at_user_pipe_claim();
	if (ret < 0) {
		if (sh != NULL) {
			if (ret == -EPERM) {
				TB45_SHELL_ERROR(sh, "modem is not ready");
			} else if (ret == -EBUSY) {
				TB45_SHELL_ERROR(sh, "AT channel is busy");
			} else {
				TB45_SHELL_ERROR(sh, "AT channel unavailable (%d)", ret);
			}
		}
		return ret;
	}

	modem_chat_release(&tb45_sms_at_chat);

	ret = modem_pipe_open_async(pipe);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_drain_rx(pipe);
	if (ret < 0) {
		goto out;
	}

	if (sh != NULL) {
		TB45_SHELL_PRINT(sh, "SMS: checking SIM readiness...");
	}

	ret = tb45_sms_pipe_send_line(pipe, "AT+CPIN?", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, "+CPIN: READY", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		if (sh != NULL) {
			TB45_SHELL_ERROR(sh, "SMS: SIM not ready (AT+CPIN?)");
		}
		goto out;
	}

	if (sh != NULL) {
		TB45_SHELL_PRINT(sh, "SMS: configuring modem for text mode...");
	}

	ret = tb45_sms_pipe_send_line(pipe, "AT+CMGF=1", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, NULL, TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_send_line(pipe, "AT+CSCS=\"GSM\"", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, NULL, TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	if (sh != NULL) {
		TB45_SHELL_PRINT(sh, "SMS: requesting CMGS prompt...");
	}

	ret = tb45_sms_pipe_send_line(pipe, cmgs_cmd, TB45_SMS_AT_PROMPT_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_prompt(pipe, TB45_SMS_AT_PROMPT_TIMEOUT_MS);
	if (ret < 0) {
		if (sh != NULL) {
			TB45_SHELL_ERROR(sh, "SMS: modem did not provide CMGS prompt");
		}
		goto out;
	}

	if (sh != NULL) {
		TB45_SHELL_PRINT(sh, "SMS: submitting payload (timeout %d s)...", submit_timeout_ms / 1000);
	}

	text_len = strlen(text);
	ret = tb45_sms_pipe_transmit_all(pipe, (const uint8_t *)text, text_len, submit_timeout_ms);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_transmit_all(pipe, &ctrl_z, 1U, submit_timeout_ms);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_sms_result(pipe, submit_timeout_ms);
	if ((ret == -ETIMEDOUT) && (sh != NULL)) {
		TB45_SHELL_ERROR(sh, "SMS: submit timed out after %d s", submit_timeout_ms / 1000);
	}

out:
	attach_ret = modem_chat_attach(&tb45_sms_at_chat, pipe);
	if ((attach_ret < 0) && (ret == 0)) {
		ret = attach_ret;
	}

	modem_at_user_pipe_release();
	return ret;
}

#ifdef CONFIG_SHELL
static int cmd_tb45_modem_at(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	return tb45_sms_at_run(sh, argv[1], (argc >= 3) ? argv[2] : "OK",
			       TB45_SMS_AT_DEFAULT_TIMEOUT_MS);
}

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_modem_cmds,
	SHELL_CMD_ARG(at, NULL,
		      SHELL_HELP("Send AT command", "<command> [expected_response]"),
		      cmd_tb45_modem_at, 2, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(modem, &tb45_modem_cmds, "Modem commands", NULL);
#endif /* CONFIG_SHELL */
