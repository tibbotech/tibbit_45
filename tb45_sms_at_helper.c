#include "tb45_sms_at_helper.h"

#include <errno.h>
#include <stdlib.h>
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
#if __has_include("work_queues.h")
#include "work_queues.h"
#else
extern struct k_work_q low_priority_wq;
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
/* Sliding token-detection window, not an AT response capture buffer. */
#define TB45_SMS_AT_RX_BUF_SIZE         64
#define TB45_SMS_AT_RX_CHUNK_SIZE       64
#define TB45_SHELL_ASCII_CTRL_C         0x03U
#define TB45_SMS_AT_RAW_CMD_BUF_SIZE    64

#define TB45_SMS_AT_MODEM_NODE      DT_ALIAS(modem)
#define TB45_SMS_AT_PIPELINK_NAME   _CONCAT(user_pipe_, CONFIG_APP_TB45_MODEM_AT_USER_PIPE_IDX)

MODEM_PIPELINK_DT_DECLARE(TB45_SMS_AT_MODEM_NODE, TB45_SMS_AT_PIPELINK_NAME);
static struct modem_pipelink *tb45_sms_at_pipelink =
	MODEM_PIPELINK_DT_GET(TB45_SMS_AT_MODEM_NODE, TB45_SMS_AT_PIPELINK_NAME);

static struct modem_chat tb45_sms_at_chat = {0};
static uint8_t tb45_sms_at_chat_receive_buf[TB45_SMS_AT_CHAT_RX_BUF_SIZE] = {0};
static uint8_t *tb45_sms_at_chat_argv_buf[2] = {0};
static struct modem_chat_script_chat tb45_sms_at_script_chat[1] = {0};
static struct modem_chat_match tb45_sms_at_script_chat_matches[2] = {0};
static struct modem_chat_script tb45_sms_at_script = {0};
static const struct shell *tb45_sms_at_active_shell = NULL;
static enum modem_chat_script_result tb45_sms_at_last_result = 0;
static tb45_sms_at_complete_cb_t tb45_sms_at_complete_cb = NULL;
static void *tb45_sms_at_complete_user_data = NULL;
static char *tb45_sms_at_capture_buf = NULL;
static size_t tb45_sms_at_capture_buf_size = 0U;
static size_t tb45_sms_at_capture_len = 0U;

enum tb45_sms_at_raw_send_state {
	TB45_SMS_AT_RAW_SEND_STATE_IDLE = 0,
	TB45_SMS_AT_RAW_SEND_STATE_CPIN_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_CPIN_WAIT,
	TB45_SMS_AT_RAW_SEND_STATE_CMGF_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_CMGF_WAIT,
	TB45_SMS_AT_RAW_SEND_STATE_CSCS_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_CSCS_WAIT,
	TB45_SMS_AT_RAW_SEND_STATE_CMGS_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_PROMPT_WAIT,
	TB45_SMS_AT_RAW_SEND_STATE_TEXT_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_CTRL_Z_SEND,
	TB45_SMS_AT_RAW_SEND_STATE_RESULT_WAIT,
};

struct tb45_sms_at_raw_send_ctx {
	struct k_work_delayable dwork;
	struct modem_pipe *pipe;
	tb45_sms_at_complete_cb_t complete_cb;
	void *complete_user_data;
	enum tb45_sms_at_raw_send_state state;
	const uint8_t *tx_buf;
	size_t tx_len;
	size_t tx_offset;
	size_t rx_len;
	int64_t deadline_ms;
	int submit_timeout_ms;
	char cmgs_cmd[TB45_SMS_AT_RAW_CMD_BUF_SIZE];
	char line_buf[TB45_SMS_AT_RAW_CMD_BUF_SIZE + 1];
	char text[CONFIG_APP_TB45_SMS_TEXT_MAX_LEN + 1];
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	bool seen_cmgs;
	bool seen_ok;
	bool active;
	bool user_pipe_claimed;
};

static struct tb45_sms_at_raw_send_ctx tb45_sms_at_raw_send_ctx = {0};
static void tb45_sms_at_raw_send_work_handler(struct k_work *work);

static int tb45_sms_at_script_result_to_errno(enum modem_chat_script_result result)
{
	switch (result) {
	case MODEM_CHAT_SCRIPT_RESULT_SUCCESS:
		return 0;
	case MODEM_CHAT_SCRIPT_RESULT_TIMEOUT:
		return -ETIMEDOUT;
	case MODEM_CHAT_SCRIPT_RESULT_ABORT:
	default:
		return -EIO;
	}
}


static void tb45_sms_at_capture_reset(char *out_buf, size_t out_buf_size)
{
	tb45_sms_at_capture_buf = out_buf;
	tb45_sms_at_capture_buf_size = out_buf_size;
	tb45_sms_at_capture_len = 0U;
	if ((out_buf != NULL) && (out_buf_size > 0U)) {
		out_buf[0] = '\0';
	}
}

static void tb45_sms_at_capture_append(const char *line)
{
	size_t line_len;
	size_t remaining;
	size_t copy_len;
	const char crlf[3] = "\r\n";

	if ((tb45_sms_at_capture_buf == NULL) || (tb45_sms_at_capture_buf_size <= 1U) ||
	    (line == NULL)) {
		return;
	}

	line_len = strlen(line);
	remaining = (tb45_sms_at_capture_buf_size - 1U) - tb45_sms_at_capture_len;
	if (remaining == 0U) {
		return;
	}

	copy_len = (line_len < remaining) ? line_len : remaining;
	memcpy(&tb45_sms_at_capture_buf[tb45_sms_at_capture_len], line, copy_len);
	tb45_sms_at_capture_len += copy_len;
	remaining = (tb45_sms_at_capture_buf_size - 1U) - tb45_sms_at_capture_len;
	if (remaining > 0U) {
		copy_len = (sizeof(crlf) - 1U < remaining) ? (sizeof(crlf) - 1U) : remaining;
		memcpy(&tb45_sms_at_capture_buf[tb45_sms_at_capture_len], crlf, copy_len);
		tb45_sms_at_capture_len += copy_len;
	}
	tb45_sms_at_capture_buf[tb45_sms_at_capture_len] = '\0';
}

static void tb45_sms_at_print_any_match(struct modem_chat *chat, char **argv, uint16_t argc,
					void *user_data)
{
	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	if (argc == 2) {
		tb45_sms_at_capture_append(argv[1]);
	}

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

	if (argc == 1) {
		tb45_sms_at_capture_append(argv[0]);
	}

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
	tb45_sms_at_complete_cb_t complete_cb;
	void *complete_user_data;
	int ret;

	ARG_UNUSED(chat);
	ARG_UNUSED(user_data);

	tb45_sms_at_last_result = result;
	complete_cb = tb45_sms_at_complete_cb;
	complete_user_data = tb45_sms_at_complete_user_data;
	ret = tb45_sms_at_script_result_to_errno(result);
	tb45_sms_at_complete_cb = NULL;
	tb45_sms_at_complete_user_data = NULL;
	tb45_sms_at_capture_buf = NULL;
	tb45_sms_at_capture_buf_size = 0U;
	tb45_sms_at_capture_len = 0U;
	tb45_sms_at_active_shell = NULL;
	modem_at_user_pipe_release();
	if (complete_cb != NULL) {
		complete_cb(ret, complete_user_data);
	}
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
	tb45_sms_at_raw_send_ctx.state = TB45_SMS_AT_RAW_SEND_STATE_IDLE;
	tb45_sms_at_raw_send_ctx.pipe = NULL;
	tb45_sms_at_raw_send_ctx.complete_cb = NULL;
	tb45_sms_at_raw_send_ctx.complete_user_data = NULL;
	tb45_sms_at_raw_send_ctx.tx_buf = NULL;
	tb45_sms_at_raw_send_ctx.tx_len = 0U;
	tb45_sms_at_raw_send_ctx.tx_offset = 0U;
	tb45_sms_at_raw_send_ctx.rx_len = 0U;
	tb45_sms_at_raw_send_ctx.deadline_ms = 0;
	tb45_sms_at_raw_send_ctx.submit_timeout_ms = 0;
	tb45_sms_at_raw_send_ctx.cmgs_cmd[0] = '\0';
	tb45_sms_at_raw_send_ctx.line_buf[0] = '\0';
	tb45_sms_at_raw_send_ctx.text[0] = '\0';
	tb45_sms_at_raw_send_ctx.seen_cmgs = false;
	tb45_sms_at_raw_send_ctx.seen_ok = false;
	tb45_sms_at_raw_send_ctx.active = false;
	tb45_sms_at_raw_send_ctx.user_pipe_claimed = false;
	k_work_init_delayable(&tb45_sms_at_raw_send_ctx.dwork, tb45_sms_at_raw_send_work_handler);
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
	modem_chat_script_set_name(&tb45_sms_at_script,
				       (sh != NULL) ? "modem_shell_at_script" : "tb45_sms_at_script");
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

int tb45_sms_at_run_async_cb(const struct shell *sh, const char *request,
			     const char *expected_response, int timeout_ms,
			     tb45_sms_at_complete_cb_t complete_cb, void *user_data)
{
	int ret;

	tb45_sms_at_capture_reset(NULL, 0U);
	tb45_sms_at_complete_cb = complete_cb;
	tb45_sms_at_complete_user_data = user_data;
	ret = tb45_sms_at_run_async(sh, request, expected_response, timeout_ms);
	if (ret < 0) {
		tb45_sms_at_capture_buf = NULL;
		tb45_sms_at_capture_buf_size = 0U;
		tb45_sms_at_capture_len = 0U;
		tb45_sms_at_complete_cb = NULL;
		tb45_sms_at_complete_user_data = NULL;
	}

	return ret;
}

int tb45_sms_at_run_async_capture_cb(const struct shell *sh, const char *request,
				     const char *expected_response, int timeout_ms,
				     char *out_buf, size_t out_buf_size,
				     tb45_sms_at_complete_cb_t complete_cb, void *user_data)
{
	int ret;

	tb45_sms_at_capture_reset(out_buf, out_buf_size);
	tb45_sms_at_complete_cb = complete_cb;
	tb45_sms_at_complete_user_data = user_data;
	ret = tb45_sms_at_run_async(sh, request, expected_response, timeout_ms);
	if (ret < 0) {
		tb45_sms_at_capture_buf = NULL;
		tb45_sms_at_capture_buf_size = 0U;
		tb45_sms_at_capture_len = 0U;
		tb45_sms_at_complete_cb = NULL;
		tb45_sms_at_complete_user_data = NULL;
	}

	return ret;
}

int tb45_sms_at_run(const struct shell *sh, const char *request, const char *expected_response,
		    int timeout_ms)
{
	int ret;
	int waited_ms = 0;

	ret = tb45_sms_at_run_async_cb(sh, request, expected_response, timeout_ms, NULL, NULL);
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

static bool tb45_sms_shell_cancel_pending(const struct shell *sh)
{
#if defined(CONFIG_SHELL)
	if ((sh == NULL) || (sh->iface == NULL) || (sh->iface->api == NULL) ||
	    (sh->iface->api->read == NULL)) {
		return false;
	}

	uint8_t rx_buf[16];

	while (true) {
		size_t cnt = 0U;
		int ret = sh->iface->api->read(sh->iface, rx_buf, sizeof(rx_buf), &cnt);
		if ((ret < 0) || (cnt == 0U)) {
			break;
		}

		for (size_t i = 0U; i < cnt; i++) {
			if (rx_buf[i] == TB45_SHELL_ASCII_CTRL_C) {
				return true;
			}
		}
	}
#else
	ARG_UNUSED(sh);
#endif

	return false;
}

static int tb45_sms_pipe_transmit_all(struct modem_pipe *pipe, const uint8_t *buf, size_t len,
			      int timeout_ms, const struct shell *sh)
{
	int ret;
	size_t sent = 0U;
	int64_t deadline_ms;

	deadline_ms = k_uptime_get() + timeout_ms;

	while (sent < len) {
		if (tb45_sms_shell_cancel_pending(sh)) {
			return -ECANCELED;
		}

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

	ret = tb45_sms_pipe_transmit_all(pipe, (const uint8_t *)line, strlen(line), timeout_ms, NULL);
	if (ret < 0) {
		return ret;
	}

	return tb45_sms_pipe_transmit_all(pipe, &cr, 1U, timeout_ms, NULL);
}

static int tb45_sms_pipe_wait_for_ok_capture(struct modem_pipe *pipe, const char *required_token,
				     int timeout_ms, char *out_buf, size_t out_buf_size,
				     const struct shell *sh)
{
	int ret;
	int64_t deadline_ms;
	size_t rx_len = 0U;
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	deadline_ms = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline_ms) {
		if (tb45_sms_shell_cancel_pending(sh)) {
			return -ECANCELED;
		}

		ret = modem_pipe_receive(pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			k_msleep(TB45_SMS_AT_POLL_INTERVAL_MS);
			continue;
		}

		tb45_sms_at_buf_append(rx_buf, sizeof(rx_buf), &rx_len, chunk, (size_t)ret);
		if ((out_buf != NULL) && (out_buf_size > 1U)) {
			size_t out_len = strlen(out_buf);
			tb45_sms_at_buf_append((uint8_t *)out_buf, out_buf_size - 1U, &out_len, chunk,
					       (size_t)ret);
			out_buf[out_len] = '\0';
		}

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

static int tb45_sms_pipe_wait_for_ok(struct modem_pipe *pipe, const char *required_token,
			     int timeout_ms, const struct shell *sh)
{
	return tb45_sms_pipe_wait_for_ok_capture(pipe, required_token, timeout_ms, NULL, 0U, sh);
}

static int tb45_sms_pipe_wait_for_prompt(struct modem_pipe *pipe, int timeout_ms,
				 const struct shell *sh)
{
	int ret;
	int64_t deadline_ms;
	size_t rx_len = 0U;
	uint8_t rx_buf[TB45_SMS_AT_RX_BUF_SIZE];
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	deadline_ms = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline_ms) {
		if (tb45_sms_shell_cancel_pending(sh)) {
			return -ECANCELED;
		}

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

static int tb45_sms_pipe_wait_for_sms_result(struct modem_pipe *pipe, int timeout_ms,
				      const struct shell *sh)
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
		if (tb45_sms_shell_cancel_pending(sh)) {
			return -ECANCELED;
		}

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


static int tb45_sms_at_raw_send_schedule(uint32_t delay_ms)
{
	return k_work_reschedule_for_queue(&low_priority_wq, &tb45_sms_at_raw_send_ctx.dwork,
						 K_MSEC(delay_ms));
}

static void tb45_sms_at_raw_send_prepare_wait(struct tb45_sms_at_raw_send_ctx *ctx, int timeout_ms)
{
	ctx->rx_len = 0U;
	ctx->deadline_ms = k_uptime_get() + timeout_ms;
	ctx->seen_cmgs = false;
	ctx->seen_ok = false;
}

static void tb45_sms_at_raw_send_prepare_transmit(struct tb45_sms_at_raw_send_ctx *ctx,
					      const uint8_t *buf, size_t len, int timeout_ms)
{
	ctx->tx_buf = buf;
	ctx->tx_len = len;
	ctx->tx_offset = 0U;
	ctx->deadline_ms = k_uptime_get() + timeout_ms;
}

static int tb45_sms_at_raw_send_prepare_line(struct tb45_sms_at_raw_send_ctx *ctx, const char *line,
					     int timeout_ms)
{
	size_t len;

	if ((ctx == NULL) || (line == NULL)) {
		return -EINVAL;
	}

	len = strlen(line);
	if (len >= (sizeof(ctx->line_buf) - 1U)) {
		return -EINVAL;
	}

	memcpy(ctx->line_buf, line, len);
	ctx->line_buf[len] = '\r';
	ctx->line_buf[len + 1U] = '\0';
	tb45_sms_at_raw_send_prepare_transmit(ctx, (const uint8_t *)ctx->line_buf, len + 1U,
					      timeout_ms);
	return 0;
}

static int tb45_sms_at_raw_send_transmit_step(struct tb45_sms_at_raw_send_ctx *ctx)
{
	while (ctx->tx_offset < ctx->tx_len) {
		int ret = modem_pipe_transmit(ctx->pipe, &ctx->tx_buf[ctx->tx_offset],
					     ctx->tx_len - ctx->tx_offset);
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return (k_uptime_get() >= ctx->deadline_ms) ? -ETIMEDOUT : -EAGAIN;
		}

		ctx->tx_offset += (size_t)ret;
	}

	return 0;
}

static int tb45_sms_at_raw_send_wait_for_ok_step(struct tb45_sms_at_raw_send_ctx *ctx,
					 const char *required_token)
{
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	while (true) {
		int ret = modem_pipe_receive(ctx->pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return (k_uptime_get() >= ctx->deadline_ms) ? -ETIMEDOUT : -EAGAIN;
		}

		tb45_sms_at_buf_append(ctx->rx_buf, sizeof(ctx->rx_buf), &ctx->rx_len, chunk,
				       (size_t)ret);
		if (tb45_sms_at_has_error(ctx->rx_buf, ctx->rx_len)) {
			return -EIO;
		}

		if (tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, "\r\nOK\r\n")) {
			if ((required_token != NULL) &&
			    !tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, required_token)) {
				return -EIO;
			}
			return 0;
		}
	}
}

static int tb45_sms_at_raw_send_wait_for_prompt_step(struct tb45_sms_at_raw_send_ctx *ctx)
{
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	while (true) {
		int ret = modem_pipe_receive(ctx->pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return (k_uptime_get() >= ctx->deadline_ms) ? -ETIMEDOUT : -EAGAIN;
		}

		tb45_sms_at_buf_append(ctx->rx_buf, sizeof(ctx->rx_buf), &ctx->rx_len, chunk,
				       (size_t)ret);
		if (tb45_sms_at_has_error(ctx->rx_buf, ctx->rx_len)) {
			return -EIO;
		}

		if (tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, "> ") ||
		    tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, ">")) {
			return 0;
		}
	}
}

static int tb45_sms_at_raw_send_wait_for_sms_result_step(struct tb45_sms_at_raw_send_ctx *ctx)
{
	uint8_t chunk[TB45_SMS_AT_RX_CHUNK_SIZE];

	while (true) {
		int ret = modem_pipe_receive(ctx->pipe, chunk, sizeof(chunk));
		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return (k_uptime_get() >= ctx->deadline_ms) ? -ETIMEDOUT : -EAGAIN;
		}

		tb45_sms_at_buf_append(ctx->rx_buf, sizeof(ctx->rx_buf), &ctx->rx_len, chunk,
				       (size_t)ret);
		if (tb45_sms_at_has_error(ctx->rx_buf, ctx->rx_len)) {
			return -EIO;
		}

		if (!ctx->seen_cmgs && tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, "+CMGS:")) {
			ctx->seen_cmgs = true;
		}

		if (!ctx->seen_ok && tb45_sms_at_buf_contains(ctx->rx_buf, ctx->rx_len, "\r\nOK\r\n")) {
			ctx->seen_ok = true;
		}

		if (ctx->seen_cmgs && ctx->seen_ok) {
			return 0;
		}
	}
}

static void tb45_sms_at_raw_send_finish(int ret)
{
	struct tb45_sms_at_raw_send_ctx *ctx = &tb45_sms_at_raw_send_ctx;
	tb45_sms_at_complete_cb_t complete_cb = ctx->complete_cb;
	void *complete_user_data = ctx->complete_user_data;
	int attach_ret = 0;

	if (ctx->pipe != NULL) {
		attach_ret = modem_chat_attach(&tb45_sms_at_chat, ctx->pipe);
		if ((attach_ret < 0) && (ret == 0)) {
			ret = attach_ret;
		}
	}

	if (ctx->user_pipe_claimed) {
		modem_at_user_pipe_release();
	}

	ctx->pipe = NULL;
	ctx->complete_cb = NULL;
	ctx->complete_user_data = NULL;
	ctx->state = TB45_SMS_AT_RAW_SEND_STATE_IDLE;
	ctx->tx_buf = NULL;
	ctx->tx_len = 0U;
	ctx->tx_offset = 0U;
	ctx->rx_len = 0U;
	ctx->deadline_ms = 0;
	ctx->submit_timeout_ms = 0;
	ctx->cmgs_cmd[0] = '\0';
	ctx->line_buf[0] = '\0';
	ctx->text[0] = '\0';
	ctx->seen_cmgs = false;
	ctx->seen_ok = false;
	ctx->active = false;
	ctx->user_pipe_claimed = false;

	if (complete_cb != NULL) {
		complete_cb(ret, complete_user_data);
	}
}

static void tb45_sms_at_raw_send_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tb45_sms_at_raw_send_ctx *ctx = CONTAINER_OF(dwork, struct tb45_sms_at_raw_send_ctx,
						       dwork);
	int ret = 0;

	if ((ctx == NULL) || !ctx->active) {
		return;
	}

	while (ctx->active) {
		switch (ctx->state) {
		case TB45_SMS_AT_RAW_SEND_STATE_CPIN_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CPIN_WAIT;
				tb45_sms_at_raw_send_prepare_wait(ctx, TB45_SMS_AT_SETUP_TIMEOUT_MS);
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CPIN_WAIT:
			ret = tb45_sms_at_raw_send_wait_for_ok_step(ctx, "+CPIN: READY");
			if (ret == 0) {
				ret = tb45_sms_at_raw_send_prepare_line(ctx, "AT+CMGF=1",
							       TB45_SMS_AT_SETUP_TIMEOUT_MS);
				if (ret < 0) {
					break;
				}
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CMGF_SEND;
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CMGF_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CMGF_WAIT;
				tb45_sms_at_raw_send_prepare_wait(ctx, TB45_SMS_AT_SETUP_TIMEOUT_MS);
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CMGF_WAIT:
			ret = tb45_sms_at_raw_send_wait_for_ok_step(ctx, NULL);
			if (ret == 0) {
				ret = tb45_sms_at_raw_send_prepare_line(ctx, "AT+CSCS=\"IRA\"",
							       TB45_SMS_AT_SETUP_TIMEOUT_MS);
				if (ret < 0) {
					break;
				}
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CSCS_SEND;
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CSCS_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CSCS_WAIT;
				tb45_sms_at_raw_send_prepare_wait(ctx, TB45_SMS_AT_SETUP_TIMEOUT_MS);
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CSCS_WAIT:
			ret = tb45_sms_at_raw_send_wait_for_ok_step(ctx, NULL);
			if (ret == 0) {
				ret = tb45_sms_at_raw_send_prepare_line(ctx, ctx->cmgs_cmd,
							       TB45_SMS_AT_PROMPT_TIMEOUT_MS);
				if (ret < 0) {
					break;
				}
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CMGS_SEND;
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CMGS_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_PROMPT_WAIT;
				tb45_sms_at_raw_send_prepare_wait(ctx, TB45_SMS_AT_PROMPT_TIMEOUT_MS);
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_PROMPT_WAIT:
			ret = tb45_sms_at_raw_send_wait_for_prompt_step(ctx);
			if (ret == 0) {
				tb45_sms_at_raw_send_prepare_transmit(ctx, (const uint8_t *)ctx->text,
							      strlen(ctx->text),
							      ctx->submit_timeout_ms);
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_TEXT_SEND;
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_TEXT_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->line_buf[0] = 0x1AU;
				ctx->line_buf[1] = '\0';
				tb45_sms_at_raw_send_prepare_transmit(ctx, (const uint8_t *)ctx->line_buf,
							      1U, ctx->submit_timeout_ms);
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CTRL_Z_SEND;
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_CTRL_Z_SEND:
			ret = tb45_sms_at_raw_send_transmit_step(ctx);
			if (ret == 0) {
				ctx->state = TB45_SMS_AT_RAW_SEND_STATE_RESULT_WAIT;
				tb45_sms_at_raw_send_prepare_wait(ctx, ctx->submit_timeout_ms);
				continue;
			}
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_RESULT_WAIT:
			ret = tb45_sms_at_raw_send_wait_for_sms_result_step(ctx);
			break;
		case TB45_SMS_AT_RAW_SEND_STATE_IDLE:
		default:
			ret = -EINVAL;
			break;
		}

		if (ret == -EAGAIN) {
			ret = tb45_sms_at_raw_send_schedule(TB45_SMS_AT_POLL_INTERVAL_MS);
			if (ret >= 0) {
				return;
			}
		}

		tb45_sms_at_raw_send_finish(ret);
		return;
	}
}

int tb45_sms_at_send_text_raw_async_cb(const char *cmgs_cmd, const char *text,
				       int submit_timeout_ms,
				       tb45_sms_at_complete_cb_t complete_cb,
				       void *user_data)
{
	struct tb45_sms_at_raw_send_ctx *ctx = &tb45_sms_at_raw_send_ctx;
	struct modem_pipe *pipe;
	size_t cmgs_len;
	size_t text_len;
	int ret;
	int attach_ret = 0;

	if ((cmgs_cmd == NULL) || (text == NULL) ||
	    (submit_timeout_ms < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
	    (submit_timeout_ms > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
		return -EINVAL;
	}

	if (k_work_queue_thread_get(&low_priority_wq) == NULL) {
		return -EAGAIN;
	}

	if (ctx->active) {
		return -EBUSY;
	}

	cmgs_len = strlen(cmgs_cmd);
	text_len = strlen(text);
	if ((cmgs_len == 0U) || (cmgs_len >= sizeof(ctx->cmgs_cmd)) ||
	    (text_len == 0U) || (text_len >= sizeof(ctx->text))) {
		return -EINVAL;
	}

	pipe = modem_pipelink_get_pipe(tb45_sms_at_pipelink);
	if (pipe == NULL) {
		return -ENODEV;
	}

	ret = modem_at_user_pipe_claim();
	if (ret < 0) {
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

	ctx->pipe = pipe;
	ctx->complete_cb = complete_cb;
	ctx->complete_user_data = user_data;
	ctx->state = TB45_SMS_AT_RAW_SEND_STATE_CPIN_SEND;
	ctx->submit_timeout_ms = submit_timeout_ms;
	memcpy(ctx->cmgs_cmd, cmgs_cmd, cmgs_len + 1U);
	memcpy(ctx->text, text, text_len + 1U);
	ctx->seen_cmgs = false;
	ctx->seen_ok = false;
	ctx->active = true;
	ctx->user_pipe_claimed = true;
	ret = tb45_sms_at_raw_send_prepare_line(ctx, "AT+CPIN?", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out_active;
	}

	ret = tb45_sms_at_raw_send_schedule(0U);
	if (ret >= 0) {
		return 0;
	}

out_active:
	ctx->active = false;
	ctx->user_pipe_claimed = false;
	ctx->pipe = pipe;
out:
	attach_ret = modem_chat_attach(&tb45_sms_at_chat, pipe);
	if ((attach_ret < 0) && (ret == 0)) {
		ret = attach_ret;
	}
	modem_at_user_pipe_release();
	ctx->pipe = NULL;
	ctx->complete_cb = NULL;
	ctx->complete_user_data = NULL;
	ctx->state = TB45_SMS_AT_RAW_SEND_STATE_IDLE;
	ctx->submit_timeout_ms = 0;
	ctx->cmgs_cmd[0] = '\0';
	ctx->line_buf[0] = '\0';
	ctx->text[0] = '\0';
	ctx->seen_cmgs = false;
	ctx->seen_ok = false;
	ctx->tx_buf = NULL;
	ctx->tx_len = 0U;
	ctx->tx_offset = 0U;
	ctx->rx_len = 0U;
	ctx->deadline_ms = 0;
	return ret;
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
		LOG_DBG("SMS: checking SIM readiness...");
	}

	ret = tb45_sms_pipe_send_line(pipe, "AT+CPIN?", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, "+CPIN: READY", TB45_SMS_AT_SETUP_TIMEOUT_MS, sh);
	if (ret < 0) {
		if (sh != NULL) {
			TB45_SHELL_ERROR(sh, "SMS: SIM not ready (AT+CPIN?)");
		}
		goto out;
	}

	if (sh != NULL) {
		LOG_DBG("SMS: configuring modem for text mode...");
	}

	ret = tb45_sms_pipe_send_line(pipe, "AT+CMGF=1", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, NULL, TB45_SMS_AT_SETUP_TIMEOUT_MS, sh);
	if (ret < 0) {
		goto out;
	}

	/* Use IRA so ASCII characters (e.g. '_') are preserved in SMS text mode. */
	ret = tb45_sms_pipe_send_line(pipe, "AT+CSCS=\"IRA\"", TB45_SMS_AT_SETUP_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok(pipe, NULL, TB45_SMS_AT_SETUP_TIMEOUT_MS, sh);
	if (ret < 0) {
		goto out;
	}

	if (sh != NULL) {
		LOG_DBG("SMS: requesting CMGS prompt...");
	}

	ret = tb45_sms_pipe_send_line(pipe, cmgs_cmd, TB45_SMS_AT_PROMPT_TIMEOUT_MS);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_prompt(pipe, TB45_SMS_AT_PROMPT_TIMEOUT_MS, sh);
	if (ret < 0) {
		if (sh != NULL) {
			TB45_SHELL_ERROR(sh, "SMS: modem did not provide CMGS prompt");
		}
		goto out;
	}

	if (sh != NULL) {
		LOG_DBG("SMS: submitting payload (timeout %d s)...", submit_timeout_ms / 1000);
	}

	text_len = strlen(text);
	ret = tb45_sms_pipe_transmit_all(pipe, (const uint8_t *)text, text_len, submit_timeout_ms, sh);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_transmit_all(pipe, &ctrl_z, 1U, submit_timeout_ms, sh);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_sms_result(pipe, submit_timeout_ms, sh);
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

int tb45_sms_at_exec_capture(const char *request, char *out_buf, size_t out_buf_size, int timeout_ms)
{
	int ret;
	int attach_ret = 0;
	struct modem_pipe *pipe;

	if ((request == NULL) || (timeout_ms < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
	    (timeout_ms > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
		return -EINVAL;
	}

	if ((out_buf != NULL) && (out_buf_size > 0U)) {
		out_buf[0] = '\0';
	}

	pipe = modem_pipelink_get_pipe(tb45_sms_at_pipelink);
	if (pipe == NULL) {
		return -ENODEV;
	}

	ret = modem_at_user_pipe_claim();
	if (ret < 0) {
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

	ret = tb45_sms_pipe_send_line(pipe, request, timeout_ms);
	if (ret < 0) {
		goto out;
	}

	ret = tb45_sms_pipe_wait_for_ok_capture(pipe, NULL, timeout_ms, out_buf, out_buf_size, NULL);

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
	const char *expected = "OK";
	int timeout_ms = TB45_SMS_AT_DEFAULT_TIMEOUT_MS;
	char *endptr = NULL;
	long parsed_timeout;

	if (argc >= 3) {
		parsed_timeout = strtol(argv[2], &endptr, 10);
		if ((endptr != NULL) && (*argv[2] != '\0') && (*endptr == '\0')) {
			if ((parsed_timeout < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
			    (parsed_timeout > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
				shell_error(sh, "timeout_ms must be %d..%d",
					    TB45_SMS_AT_MIN_TIMEOUT_MS,
					    TB45_SMS_AT_MAX_TIMEOUT_MS);
				return -EINVAL;
			}

			timeout_ms = (int)parsed_timeout;
		} else {
			expected = argv[2];
		}
	}

	if (argc >= 4) {
		parsed_timeout = strtol(argv[3], &endptr, 10);
		if ((endptr == NULL) || (*argv[3] == '\0') || (*endptr != '\0')) {
			shell_error(sh, "invalid timeout_ms: %s", argv[3]);
			return -EINVAL;
		}

		if ((parsed_timeout < TB45_SMS_AT_MIN_TIMEOUT_MS) ||
		    (parsed_timeout > TB45_SMS_AT_MAX_TIMEOUT_MS)) {
			shell_error(sh, "timeout_ms must be %d..%d",
				    TB45_SMS_AT_MIN_TIMEOUT_MS,
				    TB45_SMS_AT_MAX_TIMEOUT_MS);
			return -EINVAL;
		}

		timeout_ms = (int)parsed_timeout;
	}

	return tb45_sms_at_run(sh, argv[1], expected, timeout_ms);
}

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_modem_cmds,
	SHELL_CMD_ARG(at, NULL,
		      SHELL_HELP("Send AT command",
			 "<command> [expected_response|timeout_ms] [timeout_ms]"),
		      cmd_tb45_modem_at, 2, 2),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(modem, &tb45_modem_cmds, "Modem commands", NULL);
#endif /* CONFIG_SHELL */
