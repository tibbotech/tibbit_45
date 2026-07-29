#include "tb45_shell.h"

#include "tb45_cellular.h"
#include "tb45_ping.h"
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
#include "tb45_sms.h"
#endif
#include "tb45_sms_at_helper.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/cellular.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/socket.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/reboot.h>

#define TB45_SHELL_SIM_PUK_LEN     8U
#define TB45_SHELL_SIM_PIN_MIN_LEN 4U
#define TB45_SHELL_SIM_PIN_MAX_LEN 8U
#define TB45_SHELL_ISP_QUERY_TIMEOUT_MS 60000
#define TB45_SHELL_ISP_CAPTURE_BUF_SIZE 4096
#define TB45_SHELL_ISP_MAX_OPERATORS 32U
#define TB45_SHELL_ISP_MAX_TOKENS 8U

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
#define TB45_SHELL_SHOW_COMMANDS_TEXT "Read-only status/info commands: tb45 show <ppp_info|modem_info|summary|network_modes|isp_list|isp_current|restart_info|probe_info|sms_stat>"
#else
#define TB45_SHELL_SHOW_COMMANDS_TEXT "Read-only status/info commands: tb45 show <ppp_info|modem_info|summary|network_modes|isp_list|isp_current|restart_info|probe_info>"
#endif
#define TB45_SHELL_ISP_TOKEN_BUF_SIZE 96U
#define TB45_SHELL_ISP_GROUP_BUF_SIZE 256U
#define TB45_SHELL_PPP_AT_QUERY_TIMEOUT_MS 5000
#define TB45_SHELL_PPP_AT_CAPTURE_BUF_SIZE 256

struct tb45_shell_ping_progress_ctx {
	const struct shell *sh;
};

static int tb45_shell_current_network_mode_code = 2; /* AUTO default */

struct tb45_shell_network_mode_item {
	int code;
	const char *meaning;
};

static const struct tb45_shell_network_mode_item tb45_shell_network_modes[] = {
	{2, "AUTO (automatic mode selection)"},
	{9, "GSM + LTE (module/firmware dependent)"},
	{10, "GSM + WCDMA + LTE (module/firmware dependent)"},
	{13, "GSM only"},
	{14, "WCDMA only"},
	{19, "GSM + WCDMA (module/firmware dependent)"},
	{22, "LTE + WCDMA (module/firmware dependent)"},
	{38, "LTE only"},
	{39, "GSM + WCDMA + LTE (module/firmware dependent)"},
	{48, "LTE + legacy RAT mix (module/firmware dependent)"},
	{51, "NR5G/LTE auto on some SIMCOM families"},
	{54, "LTE + WCDMA preference profile (module/firmware dependent)"},
	{59, "LTE profile variant (module/firmware dependent)"},
	{60, "LTE profile variant (module/firmware dependent)"},
	{63, "LTE profile variant (module/firmware dependent)"},
	{67, "LTE profile variant (module/firmware dependent)"},
};

struct tb45_shell_isp_item {
	int status;
	char long_name[64];
	char short_name[32];
	char mccmnc[24];
};

static char tb45_shell_isp_response_buf[TB45_SHELL_ISP_CAPTURE_BUF_SIZE] = {0};
static struct tb45_shell_isp_item tb45_shell_isp_items[TB45_SHELL_ISP_MAX_OPERATORS] = {0};
static char tb45_shell_isp_group_buf[TB45_SHELL_ISP_GROUP_BUF_SIZE] = {0};
static char tb45_shell_isp_tokens[TB45_SHELL_ISP_MAX_TOKENS][TB45_SHELL_ISP_TOKEN_BUF_SIZE] = {0};

static const char *tb45_shell_isp_status_to_str(int status)
{
	switch (status) {
	case 1:
		return "available";
	case 2:
		return "current";
	case 3:
		return "forbidden";
	default:
		return "unknown";
	}
}

static void tb45_shell_isp_copy_trim_unquote(char *dst, size_t dst_size, const char *src)
{
	const char *start = src;
	const char *end;
	size_t out_len = 0U;

	if ((dst == NULL) || (dst_size == 0U)) {
		return;
	}

	dst[0] = '\0';
	if (src == NULL) {
		return;
	}

	while ((*start == ' ') || (*start == '\t') || (*start == '\r') || (*start == '\n')) {
		start++;
	}

	end = start + strlen(start);
	while ((end > start) && ((end[-1] == ' ') || (end[-1] == '\t') ||
				   (end[-1] == '\r') || (end[-1] == '\n'))) {
		end--;
	}

	if ((end > start + 1) && (start[0] == '"') && (end[-1] == '"')) {
		start++;
		end--;
	}

	while ((start < end) && (out_len + 1U < dst_size)) {
		dst[out_len++] = *start++;
	}

	dst[out_len] = '\0';
}

static int tb45_shell_isp_parse_group(const char *group, struct tb45_shell_isp_item *out_item)
{
	size_t token_idx = 0U;
	size_t char_idx = 0U;
	bool in_quotes = false;
	const char *p = group;
	char *endp = NULL;
	long status = 0;

	if ((group == NULL) || (out_item == NULL)) {
		return -EINVAL;
	}

	memset(tb45_shell_isp_tokens, 0, sizeof(tb45_shell_isp_tokens));

	while ((*p != '\0') && (token_idx < TB45_SHELL_ISP_MAX_TOKENS)) {
		char c = *p++;

		if (c == '"') {
			in_quotes = !in_quotes;
		}

		if ((c == ',') && !in_quotes) {
			tb45_shell_isp_tokens[token_idx][char_idx] = '\0';
			token_idx++;
			char_idx = 0U;
			continue;
		}

		if (char_idx + 1U < TB45_SHELL_ISP_TOKEN_BUF_SIZE) {
			tb45_shell_isp_tokens[token_idx][char_idx++] = c;
		}
	}

	if ((token_idx < TB45_SHELL_ISP_MAX_TOKENS) && (char_idx < TB45_SHELL_ISP_TOKEN_BUF_SIZE)) {
		tb45_shell_isp_tokens[token_idx][char_idx] = '\0';
		token_idx++;
	}

	if (token_idx < 4U) {
		return -EINVAL;
	}

	if ((tb45_shell_isp_tokens[1][0] != '"') ||
	    (tb45_shell_isp_tokens[2][0] != '"') ||
	    (tb45_shell_isp_tokens[3][0] != '"')) {
		return -EINVAL;
	}

	errno = 0;
	status = strtol(tb45_shell_isp_tokens[0], &endp, 10);
	if ((errno != 0) || (endp == tb45_shell_isp_tokens[0]) || (*endp != '\0')) {
		return -EINVAL;
	}

	out_item->status = (int)status;
	tb45_shell_isp_copy_trim_unquote(out_item->long_name, sizeof(out_item->long_name),
					 tb45_shell_isp_tokens[1]);
	tb45_shell_isp_copy_trim_unquote(out_item->short_name, sizeof(out_item->short_name),
					 tb45_shell_isp_tokens[2]);
	tb45_shell_isp_copy_trim_unquote(out_item->mccmnc, sizeof(out_item->mccmnc),
					 tb45_shell_isp_tokens[3]);

	return 0;
}

static size_t tb45_shell_isp_parse_cops(const char *response, struct tb45_shell_isp_item *items,
					 size_t max_items)
{
	const char *cops = strstr(response, "+COPS:");
	size_t found = 0U;

	if ((cops == NULL) || (items == NULL) || (max_items == 0U)) {
		return 0U;
	}

	for (const char *p = cops; *p != '\0'; p++) {
		const char *end = NULL;
		size_t len = 0U;

		if (*p != '(') {
			continue;
		}

		end = strchr(p + 1, ')');
		if (end == NULL) {
			break;
		}

		len = (size_t)(end - (p + 1));
		if (len >= sizeof(tb45_shell_isp_group_buf)) {
			p = end;
			continue;
		}

		memcpy(tb45_shell_isp_group_buf, p + 1, len);
		tb45_shell_isp_group_buf[len] = '\0';

		if (tb45_shell_isp_parse_group(tb45_shell_isp_group_buf, &items[found]) == 0) {
			found++;
			if (found >= max_items) {
				break;
			}
		}

		p = end;
	}

	return found;
}

static int tb45_shell_parse_cops_provider(const char *response, char *provider, size_t provider_size)
{
	const char *cops;
	char token[64];

	if ((response == NULL) || (provider == NULL) || (provider_size == 0U)) {
		return -EINVAL;
	}

	provider[0] = '\0';
	cops = strstr(response, "+COPS:");
	if (cops == NULL) {
		return -ENOENT;
	}

	if (sscanf(cops, "+COPS: %*d,%*d,\"%63[^\"]\"", token) == 1) {
		tb45_shell_isp_copy_trim_unquote(provider, provider_size, token);
		return (provider[0] == '\0') ? -ENOENT : 0;
	}

	if (sscanf(cops, "+COPS: %*d,%*d,%63[^,\r\n]", token) == 1) {
		tb45_shell_isp_copy_trim_unquote(provider, provider_size, token);
		return (provider[0] == '\0') ? -ENOENT : 0;
	}

	return -ENOENT;
}

static int tb45_shell_parse_cesq(const char *response, int *rsrq_code, int *rsrp_code)
{
	const char *cesq_line;
	int parsed_rsrq = 255;
	int parsed_rsrp = 255;

	if ((response == NULL) || (rsrq_code == NULL) || (rsrp_code == NULL)) {
		return -EINVAL;
	}

	cesq_line = strstr(response, "+CESQ:");
	if (cesq_line == NULL) {
		return -ENOENT;
	}

	if (sscanf(cesq_line, "+CESQ: %*d,%*d,%*d,%*d,%d,%d", &parsed_rsrq, &parsed_rsrp) != 2) {
		return -ENOENT;
	}

	*rsrq_code = parsed_rsrq;
	*rsrp_code = parsed_rsrp;
	return 0;
}

static int tb45_shell_cesq_rsrq_code_to_db10(int rsrq_code, int *rsrq_db10)
{
	if (rsrq_db10 == NULL) {
		return -EINVAL;
	}

	if (rsrq_code == 255) {
		return -ENODATA;
	}

	if ((rsrq_code < 0) || (rsrq_code > 34)) {
		return -ERANGE;
	}

	*rsrq_db10 = -195 + (5 * rsrq_code);
	return 0;
}

static int tb45_shell_cesq_rsrp_code_to_dbm(int rsrp_code, int *rsrp_dbm)
{
	if (rsrp_dbm == NULL) {
		return -EINVAL;
	}

	if (rsrp_code == 255) {
		return -ENODATA;
	}

	if ((rsrp_code < 0) || (rsrp_code > 97)) {
		return -ERANGE;
	}

	*rsrp_dbm = -140 + rsrp_code;
	return 0;
}

int tb45_shell_cmd_show_isp_list(const struct shell *sh, size_t argc, char **argv)
{
	size_t found = 0U;
	int ret;
	struct net_if *ppp_iface = NULL;
	bool ppp_active = false;

	ARG_UNUSED(argv);

	ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show isp_list");
		return -EINVAL;
	}

	ret = tb45_cellular_shell_ppp_get_iface(sh, &ppp_iface);
	if (ret == 0) {
		ppp_active = net_if_is_up(ppp_iface) || (net_if_get_default() == ppp_iface);
	}

	if (ppp_active) {
		shell_warn(sh, "Service provider scan (AT+COPS=?) can disrupt active PPP.");
		shell_warn(sh, "Please run 'tb45 ppp down' first.");
		shell_warn(sh, "Then retry 'tb45 show isp_list'.");
		return -EBUSY;
	}

	shell_print(sh, "Scanning for available Service Providers...Please wait");

	memset(tb45_shell_isp_response_buf, 0, sizeof(tb45_shell_isp_response_buf));
	ret = tb45_sms_at_exec_capture("AT+COPS=?", tb45_shell_isp_response_buf,
				       sizeof(tb45_shell_isp_response_buf),
				       TB45_SHELL_ISP_QUERY_TIMEOUT_MS);

	if (ret < 0) {
		shell_error(sh, "AT+COPS=? failed (%d)", ret);
		return ret;
	}

	found = tb45_shell_isp_parse_cops(tb45_shell_isp_response_buf, tb45_shell_isp_items,
					  TB45_SHELL_ISP_MAX_OPERATORS);

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
		      "%-15s %-24s %-14s %s\n",
		      "Status", "Long_Name", "Short_Name", "MCC+MNC");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
		      "--------------- ------------------------ -------------- ----------\n");

	for (size_t i = 0U; i < found; i++) {
		char status_text[48];
		enum shell_vt100_color row_color;

		(void)snprintf(status_text, sizeof(status_text), "%d (%s)",
			       tb45_shell_isp_items[i].status,
			       tb45_shell_isp_status_to_str(tb45_shell_isp_items[i].status));

		row_color = (tb45_shell_isp_items[i].status == 2) ?
			SHELL_VT100_COLOR_YELLOW : SHELL_VT100_COLOR_DEFAULT;

		shell_fprintf(sh, row_color,
			      "%-15s %-24s %-14s %s\n",
			      status_text,
			      tb45_shell_isp_items[i].long_name,
			      tb45_shell_isp_items[i].short_name,
			      tb45_shell_isp_items[i].mccmnc);
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_YELLOW, "Scan complete. If needed, bring data back with 'tb45 ppp up'.\n");
	return 0;
}

int tb45_shell_cmd_show_isp_current(const struct shell *sh, size_t argc, char **argv)
{
	char at_capture[TB45_SHELL_PPP_AT_CAPTURE_BUF_SIZE];
	char provider[64];
	int ret;
	int parse_ret;

	ARG_UNUSED(argv);

	ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show isp_current");
		return -EINVAL;
	}

	memset(at_capture, 0, sizeof(at_capture));
	ret = tb45_sms_at_exec_capture("AT+COPS?", at_capture, sizeof(at_capture),
				       TB45_SHELL_PPP_AT_QUERY_TIMEOUT_MS);
	if (ret < 0) {
		shell_error(sh, "AT+COPS? failed (%d)", ret);
		return ret;
	}

	parse_ret = tb45_shell_parse_cops_provider(at_capture, provider, sizeof(provider));

	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "\nPROVIDER:");
	if (parse_ret == 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n\n", provider);
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown>\n\n");
	}

	return 0;
}

static void tb45_shell_ping_print_usage(const struct shell *sh)
{
	if (sh == NULL) {
		return;
	}

	shell_print(sh,
		    "Usage: tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]");
	shell_print(sh,
		    "  Optionals: count, timeout_ms, payload_bytes");
	shell_print(sh,
		    "Example 1: tb45 net ping google.com");
	shell_print(sh,
		    "Example 2: tb45 net ping google.com 4 2000 1252");
}

int tb45_shell_cmd_help(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 help");
		return -EINVAL;
	}

	tb45_cellular_shell_print_available_commands_menu(sh);
	return 0;
}

static int tb45_shell_parse_positive_u32(const char *text, uint32_t min_value,
					 uint32_t max_value, uint32_t *out_value)
{
	if ((text == NULL) || (out_value == NULL)) {
		return -EINVAL;
	}

	char *endp = NULL;
	errno = 0;
	unsigned long parsed = strtoul(text, &endp, 10);
	if ((errno != 0) || (endp == text) || (*endp != '\0') ||
	    (parsed < min_value) || (parsed > max_value)) {
		return -EINVAL;
	}

	*out_value = (uint32_t)parsed;
	return 0;
}

static bool tb45_shell_value_is_numeric(const char *value, size_t min_len, size_t max_len)
{
	size_t len;

	if (value == NULL) {
		return false;
	}

	len = strlen(value);
	if ((len < min_len) || (len > max_len)) {
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		if (!isdigit((unsigned char)value[i])) {
			return false;
		}
	}

	return true;
}

static void tb45_shell_modem_unlock_pin_print_usage(const struct shell *sh, char **argv,
						    int argv_index, const char *command)
{
	if ((argv_index >= 0) && (argv != NULL) && (argv[argv_index] != NULL)) {
		shell_fprintf(sh, SHELL_NORMAL, "Invalid input: ");
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " %s\n", argv[argv_index]);
	}

	if (command == NULL) {
		command = "unlock_pin";
	}

	shell_fprintf(sh, SHELL_NORMAL, "Usage: tb45 modem %s <", command);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "pukcode");
	shell_fprintf(sh, SHELL_NORMAL, "> <");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "pincode");
	shell_fprintf(sh, SHELL_NORMAL, ">\n");

	shell_fprintf(sh, SHELL_NORMAL, "Example: tb45 modem %s ", command);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "12345678");
	shell_fprintf(sh, SHELL_NORMAL, " ");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "0000\n");
}

static int tb45_shell_modem_info_print_info(const struct shell *sh, const struct device *cell_dev,
					   const char *label, enum cellular_modem_info_type info,
					   bool *use_blue)
{
	char buf[96] = {0};
	int ret = cellular_get_modem_info(cell_dev, info, buf, sizeof(buf));
	enum shell_vt100_color label_color = *use_blue
		? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN;

	shell_fprintf(sh, label_color, "%s:", label);
	if (ret == 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n", buf);
		*use_blue = !(*use_blue);
		return 0;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", ret);
	*use_blue = !(*use_blue);
	return ret;
}

static void tb45_shell_modem_info_print_on_off(const struct shell *sh, const char *label,
					       bool is_on, bool *use_blue)
{
	enum shell_vt100_color label_color = *use_blue
		? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN;
	enum shell_vt100_color value_color = is_on
		? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED;

	shell_fprintf(sh, label_color, "%s:", label);
	shell_fprintf(sh, value_color, " %s\n", is_on ? "On" : "Off");
	*use_blue = !(*use_blue);
}

static void tb45_shell_ping_progress_cb(const struct tb45_ping_progress *progress,
					void *user_data)
{
	if ((progress == NULL) || (user_data == NULL)) {
		return;
	}

	struct tb45_shell_ping_progress_ctx *ctx =
		(struct tb45_shell_ping_progress_ctx *)user_data;
	if (ctx->sh == NULL) {
		return;
	}

	if (progress->success) {
		char addr_buf[NET_IPV4_ADDR_LEN];
		const char *src = net_addr_ntop(AF_INET, &progress->reply_addr, addr_buf,
						sizeof(addr_buf));
		shell_print(ctx->sh, "Reply from %s: icmp_seq=%u time=%u ms",
			    (src != NULL) ? src : "<invalid>",
			    progress->sequence, progress->elapsed_ms);
		return;
	}

	if (progress->status == -ETIMEDOUT) {
		shell_print(ctx->sh, "Request timeout for icmp_seq=%u", progress->sequence);
		return;
	}

	shell_print(ctx->sh, "Request failed for icmp_seq=%u (%d)",
		    progress->sequence, progress->status);
}

static bool tb45_shell_ping_cancel_cb(void *user_data)
{
	const struct shell *sh = (const struct shell *)user_data;

	return tb45_cellular_shell_interrupt_cancel_pending(sh);
}

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
static void tb45_shell_sms_print_usage(const struct shell *sh)
{
	if (sh == NULL) {
		return;
	}

	shell_print(sh, "Usage: tb45 sms send <phone> <text>");
	shell_print(sh, "Example: tb45 sms send \"+886123456789\" \"hello\"");
}

int tb45_shell_cmd_sms_send(const struct shell *sh, size_t argc, char **argv)
{
	struct tb45_sms_request request = {
		.message_id = 0U,
	};

	if (argc != 3U) {
		tb45_shell_sms_print_usage(sh);
		return -EINVAL;
	}

	/* argv[1] = phone, argv[2] = text */
	(void)snprintf(request.phone_number, sizeof(request.phone_number), "%s", argv[1]);
	(void)snprintf(request.message, sizeof(request.message), "%s", argv[2]);
	return tb45_sms_send(sh, &request);
}

int tb45_shell_cmd_sms_init(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 sms init");
		return -EINVAL;
	}

	shell_print(sh, "SMS init: delegated to tb45 modem restart");
	return shell_execute_cmd(sh, "tb45 modem restart");
}

#endif

int tb45_shell_cmd_net_ping(const struct shell *sh, size_t argc, char **argv)
{
	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if ((argc < 2U) || (argc > 5U)) {
		tb45_shell_ping_print_usage(sh);
		return -EINVAL;
	}

	uint16_t count = 0U;
	if (argc >= 3U) {
		uint32_t parsed_count = 0U;
		if (tb45_shell_parse_positive_u32(argv[2], 1U, UINT16_MAX, &parsed_count) < 0) {
			shell_error(sh, "Invalid ping count: %s", argv[2]);
			return -EINVAL;
		}
		count = (uint16_t)parsed_count;
	}

	uint32_t timeout_ms = 0U;
	if (argc >= 4U) {
		if (tb45_shell_parse_positive_u32(argv[3], 1U, UINT32_MAX, &timeout_ms) < 0) {
			shell_error(sh, "Invalid ping timeout_ms: %s", argv[3]);
			return -EINVAL;
		}
	}

	size_t payload_size = TB45_PING_DEFAULT_PAYLOAD_SIZE;
	if (argc >= 5U) {
		uint32_t parsed_payload = 0U;
		if (tb45_shell_parse_positive_u32(argv[4], 0U,
						  TB45_PING_MAX_PAYLOAD_SIZE,
						  &parsed_payload) < 0) {
			shell_error(sh, "Invalid ping payload_bytes: %s (max=%u)",
				    argv[4], (unsigned int)TB45_PING_MAX_PAYLOAD_SIZE);
			return -EINVAL;
		}
		payload_size = (size_t)parsed_payload;
	}

	uint32_t shown_count = (count == 0U) ? CONFIG_APP_TB45_ASYNC_PING_DEFAULT_COUNT : count;
	uint32_t shown_timeout = (timeout_ms == 0U)
		? CONFIG_APP_TB45_ASYNC_PING_DEFAULT_TIMEOUT_MS
		: timeout_ms;
	shell_print(sh, "Running ping: host=%s count=%u timeout=%u ms payload=%u bytes",
		    argv[1], shown_count, shown_timeout, (unsigned int)payload_size);
	struct net_if *iface = net_if_get_default();
	if (iface != NULL) {
		const char *iface_l2 = "OTHER";
		if (net_if_l2(iface) == &NET_L2_GET_NAME(PPP)) {
			iface_l2 = "PPP";
		} else if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
			iface_l2 = "ETH";
		}
		shell_print(sh, "Ping interface: %s (idx=%d)", iface_l2, net_if_get_by_iface(iface));
	} else {
		shell_print(sh, "Ping interface: <none>");
	}

	struct tb45_shell_ping_progress_ctx progress_ctx = {
		.sh = sh,
	};

	(void)tb45_cellular_shell_interrupt_begin();

	ret = tb45_ping_run_ex_cancelable(argv[1], count, timeout_ms, payload_size,
					tb45_shell_ping_progress_cb, tb45_shell_ping_cancel_cb,
					&progress_ctx, (void *)sh);
	tb45_cellular_shell_interrupt_end();
	if (ret < 0) {
		if (ret == -ECANCELED) {
			shell_warn(sh, "Ping canceled by Ctrl+C");
		} else {
			shell_error(sh, "Ping failed (%d)", ret);
		}
		return ret;
	}

	return 0;
}

int tb45_shell_cmd_modem_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show modem_info");
		return -EINVAL;
	}

	const struct device *cell_dev = tb45_cellular_get_device();
	if (cell_dev == NULL) {
		shell_error(sh, "No DT alias 'modem' found");
		return -ENODEV;
	}

	if (!device_is_ready(cell_dev)) {
		shell_error(sh, "Cellular device not ready: %s", cell_dev->name);
		return -ENODEV;
	}

	int overall_ret = 0;
	bool use_blue = true;

	enum pm_device_state pm_state = PM_DEVICE_STATE_ACTIVE;
	int pm_ret = pm_device_state_get(cell_dev, &pm_state);
	bool pm_on = (pm_ret == 0) && (pm_state == PM_DEVICE_STATE_ACTIVE);

	shell_fprintf(sh, SHELL_VT100_COLOR_MAGENTA, "\nCELL_INFO:");
	shell_fprintf(sh, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "\n");

	ret = tb45_shell_modem_info_print_info(sh, cell_dev, "IMEI", CELLULAR_MODEM_INFO_IMEI, &use_blue);
	if (ret != 0) {
		overall_ret = ret;
	}

	ret = tb45_shell_modem_info_print_info(sh, cell_dev, "MODEL", CELLULAR_MODEM_INFO_MODEL_ID, &use_blue);
	if (ret != 0) {
		overall_ret = ret;
	}

	ret = tb45_shell_modem_info_print_info(sh, cell_dev, "MFR",
					      CELLULAR_MODEM_INFO_MANUFACTURER, &use_blue);
	if (ret != 0) {
		overall_ret = ret;
	}

	ret = tb45_shell_modem_info_print_info(sh, cell_dev, "FW", CELLULAR_MODEM_INFO_FW_VERSION, &use_blue);
	if (ret != 0) {
		overall_ret = ret;
	}

	enum cellular_registration_status reg = CELLULAR_REGISTRATION_NOT_REGISTERED;
	ret = cellular_get_registration_status(cell_dev, CELLULAR_ACCESS_TECHNOLOGY_LTE, &reg);
	shell_fprintf(sh, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "REG_STATUS:");
	if (ret == 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", (int)reg);
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", ret);
		overall_ret = ret;
	}
	use_blue = !use_blue;

	tb45_shell_modem_info_print_on_off(sh, "PPP", pm_on, &use_blue);
	tb45_shell_modem_info_print_on_off(sh, "Power-Management", pm_on, &use_blue);
	if (pm_ret != 0) {
		shell_fprintf(sh, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "PM_STATE:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", pm_ret);
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	return overall_ret;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), modem_info, NULL,
                  "Print modem info via cellular API",
                  tb45_shell_cmd_modem_info, 0, 0);

int tb45_shell_cmd_modem_on(const struct shell *sh, size_t argc, char **argv)
{
	return tb45_cellular_shell_cmd_modem_on(sh, argc, argv);
}

int tb45_shell_cmd_modem_off(const struct shell *sh, size_t argc, char **argv)
{
	return tb45_cellular_shell_cmd_modem_off(sh, argc, argv);
}

int tb45_shell_cmd_modem_restart(const struct shell *sh, size_t argc, char **argv)
{
    return tb45_cellular_shell_cmd_modem_restart(sh, argc, argv);
}

int tb45_shell_cmd_modem_shutdown(const struct shell *sh, size_t argc, char **argv)
{
	return tb45_cellular_shell_cmd_modem_shutdown(sh, argc, argv);
}

int tb45_shell_cmd_modem_unlock_pin(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3U) {
        tb45_shell_modem_unlock_pin_print_usage(sh, argv, -1, "unlock_pin");
        return -EINVAL;
    }

    int ret = tb45_cellular_shell_check_menu_loaded(sh);
    if (ret < 0) {
        return ret;
    }

    int cpin_state = TB45_CELLULAR_CPIN_STATE_UNKNOWN;
    ret = tb45_cellular_get_cpin_state(&cpin_state);
    if (ret < 0) {
        shell_error(sh, "Failed to read CPIN state (%d)", ret);
        return ret;
    }

    if (cpin_state == TB45_CELLULAR_CPIN_STATE_READY) {
        shell_print(sh, "Sim already Unlocked...No action required");
        return 0;
    }

    if (!tb45_shell_value_is_numeric(argv[1], TB45_SHELL_SIM_PUK_LEN,
                                     TB45_SHELL_SIM_PUK_LEN)) {
        tb45_shell_modem_unlock_pin_print_usage(sh, argv, 1, "unlock_pin");
        return -EINVAL;
    }

    if (!tb45_shell_value_is_numeric(argv[2], TB45_SHELL_SIM_PIN_MIN_LEN,
                                     TB45_SHELL_SIM_PIN_MAX_LEN)) {
        tb45_shell_modem_unlock_pin_print_usage(sh, argv, 2, "unlock_pin");
        return -EINVAL;
    }

    ret = tb45_cellular_submit_sim_puk_unlock(argv[1], argv[2]);
    if (ret < 0) {
        shell_error(sh, "Failed to unlock SIM PIN (%d)", ret);
        return ret;
    }

    shell_fprintf(sh, SHELL_NORMAL,
                  "For the Sim Unlock to take effect, please run: ");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "tb45 modem restart\n");
	return 0;
}

int tb45_shell_cmd_ppp_up(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 ppp up");
		return -EINVAL;
	}

	struct net_if *iface = NULL;
	ret = tb45_cellular_shell_ppp_get_iface(sh, &iface);
	if (ret < 0) {
		return ret;
	}

	if (tb45_cellular_shell_modem_off_in_progress()) {
		shell_warn(sh, "Modem is currently powering off...please wait...");
		ret = tb45_cellular_shell_modem_off_wait(sh);
		if (ret < 0) {
			return ret;
		}
		if ((ret != 0) && (ret != -EALREADY)) {
			shell_error(sh, "Modem off in progress failed (%d)", ret);
			return ret;
		}
	}

	if (net_if_is_up(iface)) {
		(void)tb45_cellular_shell_ppp_consume_down_triggered();
		shell_print(sh, "PPP interface is already up (idx=%d)", net_if_get_by_iface(iface));
		return 0;
	}

	if (tb45_cellular_shell_modem_down_triggered()) {
		shell_warn(sh, "PPP up detected modem-down state: using PPP recovery path");
		return tb45_cellular_shell_cmd_ppp_restart(sh, argc, argv);
	}

	if (tb45_cellular_shell_ppp_consume_down_triggered()) {
		shell_warn(sh, "PPP up detected prior ppp-down trigger: using PPP recovery path");
		return tb45_cellular_shell_cmd_ppp_restart(sh, argc, argv);
	}

	ret = tb45_cellular_shell_ppp_set_state(iface, true);
	if ((ret != 0) && (ret != -EALREADY)) {
		shell_error(sh, "Failed to bring PPP up (%d)", ret);
		return ret;
	}

	ret = tb45_cellular_shell_ppp_wait_link_ready(sh, iface);
	if (ret < 0) {
		shell_error(sh, "PPP up timeout waiting for PPP_LINK_STATE active/ready");
		return ret;
	}
	shell_print(sh, "PPP link is active/ready (idx=%d)", net_if_get_by_iface(iface));

	net_if_set_default(iface);
	if (net_if_get_default() != iface) {
		shell_error(sh, "Failed to set default traffic route to PPP");
		return -EIO;
	}

	shell_print(sh, "Default traffic route set to PPP (idx=%d)", net_if_get_by_iface(iface));

	ret = tb45_cellular_shell_ppp_wait_ipv4_ready(sh, iface);
	if (ret < 0) {
		if (ret == -ETIMEDOUT) {
			shell_error(sh, "PPP up timeout waiting for IPCP IPv4 readiness");
		}
		return ret;
	}

	ret = tb45_cellular_ppp_ready_post_actions();
	if (ret < 0) {
		shell_error(sh, "Failed PPP-ready post actions (%d)", ret);
		return ret;
	}

	return 0;
}

int tb45_shell_cmd_ppp_restart(const struct shell *sh, size_t argc, char **argv)
{
	return tb45_cellular_shell_cmd_ppp_restart(sh, argc, argv);
}

int tb45_shell_cmd_ppp_down(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 ppp down");
		return -EINVAL;
	}

	ret = tb45_cellular_probe_set_enabled(false);
	if (ret < 0) {
		shell_error(sh, "Failed to disable probe (%d)", ret);
		return ret;
	}

	struct net_if *iface = NULL;
	ret = tb45_cellular_shell_ppp_get_iface(sh, &iface);
	if (ret < 0) {
		return ret;
	}

	ret = tb45_cellular_shell_ppp_set_state(iface, false);
	if ((ret == 0) || (ret == -EALREADY)) {
		shell_print(sh, "PPP interface is down (idx=%d)", net_if_get_by_iface(iface));
		return tb45_shell_cmd_ppp_info(sh, 1, NULL);
	}

	shell_error(sh, "Failed to bring PPP down (%d)", ret);
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "Please try to run ");
	shell_fprintf(sh, SHELL_VT100_COLOR_YELLOW, "tb45 ppp down");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " again\n");
	return ret;
}

static struct net_if *tb45_shell_get_ethernet_iface(void)
{
	struct net_if *tmp = NULL;

	for (int i = 1; (tmp = net_if_get_by_index(i)) != NULL; i++) {
		if ((net_if_l2(tmp) == &NET_L2_GET_NAME(ETHERNET)) &&
		    !net_if_is_wifi(tmp)) {
			return tmp;
		}
	}

	return NULL;
}

static void tb45_shell_ppp_default_traffic_route_print_usage(const struct shell *sh,
							      size_t argc, char **argv)
{
	struct net_if *iface = net_if_get_default();
	const char *current = "unknown";

	if (iface != NULL) {
		if (net_if_l2(iface) == &NET_L2_GET_NAME(PPP)) {
			current = "on (PPP)";
		} else {
			current = "off (Ethernet)";
		}
	}

	const char *value = (argc >= 2U && argv[1] != NULL) ? argv[1] : "";
	if ((strcmp(value, "") != 0) && (strcmp(value, "on") != 0) &&
	    (strcmp(value, "off") != 0)) {
		shell_fprintf(sh, SHELL_NORMAL, "Invalid input: ");
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " %s\n", value);
	}

	shell_fprintf(sh, SHELL_NORMAL, "Current value: ");
	shell_fprintf(sh, SHELL_VT100_COLOR_YELLOW, " %s\n", current);

	shell_fprintf(sh, SHELL_NORMAL, "Usage: tb45 ppp default_traffic_route <");
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "on");
	shell_fprintf(sh, SHELL_NORMAL, "|");
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "off");
	shell_fprintf(sh, SHELL_NORMAL, ">\n");
}

int tb45_shell_cmd_ppp_default_traffic_route(const struct shell *sh, size_t argc, char **argv)
{
	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 2U) {
		tb45_shell_ppp_default_traffic_route_print_usage(sh, argc, argv);
		return (argc == 1U) ? 0 : -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		struct net_if *iface = NULL;
		ret = tb45_cellular_shell_ppp_get_iface(sh, &iface);
		if (ret < 0) {
			return ret;
		}

		ret = tb45_cellular_probe_set_enabled(true);
		if (ret < 0) {
			shell_error(sh, "Failed to enable probe (%d)", ret);
			return ret;
		}

		net_if_set_default(iface);
		shell_print(sh, "Default traffic route set to PPP (idx=%d)",
			    net_if_get_by_iface(iface));
		return tb45_shell_cmd_ppp_info(sh, 1, NULL);
	}

	if (strcmp(argv[1], "off") == 0) {
		struct net_if *iface = tb45_shell_get_ethernet_iface();
		if (iface == NULL) {
			shell_error(sh, "No Ethernet interface found");
			return -ENODEV;
		}

		ret = tb45_cellular_probe_set_enabled(false);
		if (ret < 0) {
			shell_error(sh, "Failed to disable probe (%d)", ret);
			return ret;
		}

		net_if_set_default(iface);
		shell_print(sh, "Default traffic route set to Ethernet (idx=%d)",
			    net_if_get_by_iface(iface));
		return tb45_shell_cmd_ppp_info(sh, 1, NULL);
	}

	tb45_shell_ppp_default_traffic_route_print_usage(sh, argc, argv);
	return -EINVAL;
}

#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
int tb45_shell_cmd_probe_on(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 probe on");
		return -EINVAL;
	}

	int ret = tb45_cellular_probe_set_enabled(true);
	if (ret < 0) {
		shell_error(sh, "Failed to enable probe (%d)", ret);
		return ret;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "Probe control: ");
	shell_fprintf(sh, SHELL_VT100_COLOR_GREEN, "enabled\n");
	return 0;
}

int tb45_shell_cmd_probe_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 probe off");
		return -EINVAL;
	}

	int ret = tb45_cellular_probe_set_enabled(false);
	if (ret < 0) {
		shell_error(sh, "Failed to disable probe (%d)", ret);
		return ret;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "Probe control: ");
	shell_fprintf(sh, SHELL_VT100_COLOR_RED, "disabled\n");
	return 0;
}
#endif

int tb45_shell_cmd_system_reboot(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n********************\n");
	shell_fprintf(sh, SHELL_NORMAL, "*");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, " REBOOTING DEVICE ");
	shell_fprintf(sh, SHELL_NORMAL, "*\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "********************\n");

	/* Give the shell backend time to flush before hard reset */
	k_msleep(1000);

	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

int tb45_shell_cmd_ppp_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show ppp_info");
		return -EINVAL;
	}

	if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
		shell_error(sh, "PPP is DISABLED (CONFIG_NET_L2_PPP=n)");
		return -ENOTSUP;
	}

	struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
	if (iface == NULL) {
		shell_error(sh, "No PPP interface found");
		return -ENODEV;
	}

	bool iface_up = net_if_is_up(iface);
	bool default_route_is_ppp = (net_if_get_default() == iface);
	bool use_blue = true;
#define TB45_PPP_LABEL_COLOR() (use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN)
#define TB45_PPP_TOGGLE_LABEL_COLOR() do { use_blue = !use_blue; } while (0)

	shell_fprintf(sh, SHELL_VT100_COLOR_MAGENTA, "\nPPP_INFO:\n");
	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PPP_IFACE_INDEX:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", net_if_get_by_iface(iface));
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PPP_IFACE_UP:");
	shell_fprintf(sh, iface_up ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED, " %s\n",
		      iface_up ? "yes" : "no");
	TB45_PPP_TOGGLE_LABEL_COLOR();

	bool iface_dormant = net_if_is_dormant(iface);
	bool carrier_ok = net_if_is_carrier_ok(iface);
	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PPP_CARRIER_OK:");
	shell_fprintf(sh, carrier_ok ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED, " %s\n",
		      carrier_ok ? "yes" : "no");
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PPP_LINK_STATE:");
	if (!iface_up) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " down\n");
	} else if (iface_dormant) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " paused/sleeping\n");
	} else if (carrier_ok) {
		shell_fprintf(sh, SHELL_VT100_COLOR_GREEN, " active/ready\n");
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " up/no-carrier\n");
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PPP_DEFAULT_TRAFFIC_ROUTE:");
	shell_fprintf(sh, default_route_is_ppp ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED,
		      " %s\n", default_route_is_ppp ? "yes" : "no");
	TB45_PPP_TOGGLE_LABEL_COLOR();

	if (!iface_up) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, "**IPCP_STATE**: stale (iface down)\n");
	}

	bool has_ipv4_addr = false;
	if (iface->config.ip.ipv4 == NULL) {
		shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <none>\n");
		TB45_PPP_TOGGLE_LABEL_COLOR();
	} else {
		char buf[NET_IPV4_ADDR_LEN];

		for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
			const struct in_addr *addr =
				&iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr;
			if (addr->s_addr == 0U) {
				continue;
			}

			shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR[%d]:", i);
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s%s\n",
				      net_addr_ntop(AF_INET, addr, buf, sizeof(buf)),
				      iface_up ? "" : " (stale)");
			TB45_PPP_TOGGLE_LABEL_COLOR();
			has_ipv4_addr = true;
		}

		if (!has_ipv4_addr) {
			shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR:");
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <none>\n");
			TB45_PPP_TOGGLE_LABEL_COLOR();
		}

		shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "IPCP_GW:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s%s\n",
			      net_addr_ntop(AF_INET, &iface->config.ip.ipv4->gw, buf, sizeof(buf)),
			      iface_up ? "" : " (stale)");
		TB45_PPP_TOGGLE_LABEL_COLOR();
	}

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "INTERNET:");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " run ");
	shell_fprintf(sh, SHELL_VT100_COLOR_YELLOW, "tb45 show probe_info");
#else
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " feature not built");
#endif
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "BAUDRATE:");
	uint32_t baudrate = 0U;
	ret = tb45_cellular_get_runtime_baudrate(&baudrate);
	if (ret == 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", (unsigned int)baudrate);
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " <err %d>\n", ret);
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "NETWORK_MODE:");
	int network_mode_code = tb45_shell_current_network_mode_code;
	ret = tb45_cellular_get_runtime_network_mode(&network_mode_code);
	if (ret == 0) {
		tb45_shell_current_network_mode_code = network_mode_code;
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d (%s)\n",
			      network_mode_code,
			      tb45_cellular_network_mode_to_str(network_mode_code));
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d (%s) [cached, err %d]\n",
			      tb45_shell_current_network_mode_code,
			      tb45_cellular_network_mode_to_str(
				      tb45_shell_current_network_mode_code),
			      ret);
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	char at_capture[TB45_SHELL_PPP_AT_CAPTURE_BUF_SIZE];
	char provider[64];
	int parse_ret;
	int rsrp_code = 255;
	int rsrp_dbm = 0;
	int rsrq_code = 255;
	int rsrq_db10 = 0;
	const char *rsrp_grade = "unknown";
	const char *rsrq_grade = "unknown";

	memset(at_capture, 0, sizeof(at_capture));
	ret = tb45_sms_at_exec_capture("AT+COPS?", at_capture, sizeof(at_capture),
				       TB45_SHELL_PPP_AT_QUERY_TIMEOUT_MS);
	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "PROVIDER:");
	if (ret < 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " <err %d>\n", ret);
	} else {
		parse_ret = tb45_shell_parse_cops_provider(at_capture, provider, sizeof(provider));
		if (parse_ret == 0) {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n", provider);
		} else {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown>\n");
		}
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	memset(at_capture, 0, sizeof(at_capture));
	ret = tb45_sms_at_exec_capture("AT+CESQ", at_capture, sizeof(at_capture),
				       TB45_SHELL_PPP_AT_QUERY_TIMEOUT_MS);
	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "SIGNAL_STRENGTH:");
	if (ret < 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " <err %d>\n", ret);
	} else {
		parse_ret = tb45_shell_parse_cesq(at_capture, &rsrq_code, &rsrp_code);
		if (parse_ret < 0) {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown>\n");
		} else if (tb45_shell_cesq_rsrp_code_to_dbm(rsrp_code, &rsrp_dbm) == 0) {
			if (rsrp_dbm >= -90) {
				rsrp_grade = "excellent";
			} else if (rsrp_dbm >= -100) {
				rsrp_grade = "good";
			} else if (rsrp_dbm >= -110) {
				rsrp_grade = "fair";
			} else {
				rsrp_grade = "poor";
			}

			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " RSRP=%d dBm (%s)\n",
				      rsrp_dbm, rsrp_grade);
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
				      " (range: excellent >=-90 | good -100..-91 | fair -110..-101 | poor <-110)\n");
		} else {
			shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown> (CESQ=%d)\n", rsrp_code);
		}
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "SIGNAL_QUALITY:");
	if (ret < 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_RED, " <err %d>\n", ret);
	} else if (parse_ret < 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown>\n");
	} else if (tb45_shell_cesq_rsrq_code_to_db10(rsrq_code, &rsrq_db10) == 0) {
		int rsrq_abs_db10 = (rsrq_db10 < 0) ? -rsrq_db10 : rsrq_db10;

		if (rsrq_db10 >= -90) {
			rsrq_grade = "excellent";
		} else if (rsrq_db10 >= -120) {
			rsrq_grade = "good";
		} else if (rsrq_db10 >= -150) {
			rsrq_grade = "fair";
		} else {
			rsrq_grade = "poor";
		}

		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " RSRQ=%d.%d dB (%s)\n",
			      rsrq_db10 / 10, rsrq_abs_db10 % 10, rsrq_grade);
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT,
			      " (range: excellent -3..-9 | good -10..-12 | fair -12..-15 | poor < -15)\n");
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " <unknown> (CESQ=%d)\n", rsrq_code);
	}
	TB45_PPP_TOGGLE_LABEL_COLOR();

	shell_fprintf(sh, SHELL_VT100_COLOR_YELLOW,
		      "NOTE: SIGNAL_QUALITY is determined factor (based on test results)\n");
	shell_fprintf(sh, TB45_PPP_LABEL_COLOR(), "\n");
#undef TB45_PPP_LABEL_COLOR
#undef TB45_PPP_TOGGLE_LABEL_COLOR
	return 0;
}

int tb45_shell_cmd_show_summary(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show summary");
		return -EINVAL;
	}

	int modem_ret = tb45_shell_cmd_modem_info(sh, 1, NULL);
	int ppp_ret = tb45_shell_cmd_ppp_info(sh, 1, NULL);

	if (modem_ret != 0) {
		return modem_ret;
	}
	return ppp_ret;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), summary, NULL,
		  "Print both modem and PPP status info",
		  tb45_shell_cmd_show_summary, 0, 0);

int tb45_shell_cmd_show_network_modes(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show network_modes");
		return -EINVAL;
	}

	bool use_blue = true;
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "CNMP Code  Meaning\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "---------  ----------------------------------------------\n");

	for (size_t i = 0U; i < (sizeof(tb45_shell_network_modes) / sizeof(tb45_shell_network_modes[0])); i++) {
		enum shell_vt100_color code_color = use_blue ? SHELL_VT100_COLOR_BLUE :
							   SHELL_VT100_COLOR_CYAN;
		shell_fprintf(sh, code_color, "%-9d", tb45_shell_network_modes[i].code);
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "  %s\n",
			      tb45_shell_network_modes[i].meaning);
		use_blue = !use_blue;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	return 0;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), network_modes, NULL,
		  "Print CNMP codes and meanings",
		  tb45_shell_cmd_show_network_modes, 0, 0);

SHELL_SUBCMD_ADD((tb45_show_cmds), isp_list, NULL,
		  "Scan and show ISP list (AT+COPS=?)",
		  tb45_shell_cmd_show_isp_list, 0, 0);

SHELL_SUBCMD_ADD((tb45_show_cmds), isp_current, NULL,
		  "Show current ISP (AT+COPS?)",
		  tb45_shell_cmd_show_isp_current, 0, 0);

SHELL_SUBCMD_ADD((tb45_show_cmds), ppp_info, NULL,
                  "Print PPP interface status and IPv4 info",
                  tb45_shell_cmd_ppp_info, 0, 0);

#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
int tb45_shell_cmd_probe_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show probe_info");
		return -EINVAL;
	}

	struct tb45_cellular_probe_info info = {0};
	struct tb45_cellular_restart_info restart_info = {0};
	ret = tb45_cellular_get_probe_info(&info);
	if (ret < 0) {
		shell_error(sh, "Failed to read probe counters (%d)", ret);
		return ret;
	}

	ret = tb45_cellular_get_restart_info(&restart_info);
	if (ret < 0) {
		shell_error(sh, "Failed to read restart counters (%d)", ret);
		return ret;
	}

	int total_executed = info.pass_count + info.fail_count;
	int total_restart_events = restart_info.ppp_restart_count +
		restart_info.modem_restart_count +
		restart_info.full_bringup_restart_count;
	int64_t active_time_ms = 0;
	int64_t active_time_s = 0;
	char at_capture[TB45_SHELL_PPP_AT_CAPTURE_BUF_SIZE];
	char provider[64];
	int parse_ret = -1;
	int rsrp_code = 255;
	int rsrp_dbm = 0;
	int rsrq_code = 255;
	int rsrq_db10 = 0;
	const char *rsrp_grade = "unknown";
	const char *rsrq_grade = "unknown";

	if ((info.periodic_interval_ms > 0) && (info.active_since_ms > 0)) {
		active_time_ms = k_uptime_get() - info.active_since_ms;
		if (active_time_ms < 0) {
			active_time_ms = 0;
		}
		active_time_s = active_time_ms / 1000;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_MAGENTA, "\nINET_ISALIVE_PROBE_INFO:\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "PERIODIC_INTERVAL_MS:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.periodic_interval_ms);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "ACTIVE_TIME:");
	if (info.periodic_interval_ms <= 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " disabled\n");
	} else if (info.active_since_ms <= 0) {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " not active\n");
	} else {
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %llds (%lld ms)\n",
			      (long long)active_time_s, (long long)active_time_ms);
	}
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "PASS_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.pass_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "FAIL_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.fail_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "TOTAL_EXECUTED:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", total_executed);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "TOTAL_RESTART_EVENTS:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", total_restart_events);

	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");
	return 0;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), probe_info, NULL,
		  "Print periodic PING probe counters",
		  tb45_shell_cmd_probe_info, 0, 0);

#endif

int tb45_shell_cmd_restart_info(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc != 1U) {
		shell_error(sh, "Usage: tb45 show restart_info");
		return -EINVAL;
	}

	struct tb45_cellular_restart_info info = {0};
	ret = tb45_cellular_get_restart_info(&info);
	if (ret < 0) {
		shell_error(sh, "Failed to read restart counters (%d)", ret);
		return ret;
	}

	int total = info.ppp_restart_count + info.modem_restart_count +
		info.full_bringup_restart_count;

	shell_fprintf(sh, SHELL_VT100_COLOR_MAGENTA, "\nRESTART_INFO:\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "PPP_RESTART_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.ppp_restart_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "MODEM_RESTART_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.modem_restart_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "FULL_BRINGUP_RESTART_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n", info.full_bringup_restart_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "TOTAL_RESTART_EVENTS:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %d\n\n", total);

	return 0;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), restart_info, NULL,
		  "Print restart counters since boot",
		  tb45_shell_cmd_restart_info, 0, 0);

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
int tb45_shell_cmd_sms_stat(const struct shell *sh, size_t argc, char **argv)
{
	bool show_all = false;

	int ret = tb45_cellular_shell_check_menu_loaded(sh);
	if (ret < 0) {
		return ret;
	}

	if (argc > 2U) {
		shell_error(sh, "Usage: tb45 show sms_stat [all]");
		return -EINVAL;
	}
	if (argc == 2U) {
		if (strcmp(argv[1], "all") != 0) {
			shell_error(sh, "Usage: tb45 show sms_stat [all]");
			return -EINVAL;
		}
		show_all = true;
	}

	struct tb45_sms_stats stats = {0};
	ret = tb45_sms_get_stats(&stats);
	if (ret < 0) {
		shell_error(sh, "Failed to read SMS stats (%d)", ret);
		return ret;
	}

	shell_fprintf(sh, SHELL_VT100_COLOR_MAGENTA, "\nSMS_STAT:\n");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_SETUP_DONE:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n",
		      stats.rx_setup_done != 0U ? "yes" : "no");
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_STARTUP_CLEANUP_DONE:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n",
		      stats.rx_cleanup_done != 0U ? "yes" : "no");
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_INIT_COMPLETED_LOGGED:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %s\n",
		      stats.rx_init_completed_logged != 0U ? "yes" : "no");
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_INIT_RETRY_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", stats.rx_init_retry_count);

	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_PROCESSED_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", stats.rx_processed_count);
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_SCAN_FAIL_COUNT:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", stats.rx_scan_fail_count);

	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_RESULT_QUEUE_USED:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", stats.rx_result_queue_used);
	shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "ASYNC_RESULT_QUEUE_USED:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n", stats.async_result_queue_used);
	shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "QUEUE_DROP_TOTAL:");
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
		      stats.rx_trigger_queue_drop_count + stats.rx_result_queue_drop_count +
			      stats.async_result_queue_drop_count);

	if (show_all) {
		shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_TRIGGER_QUEUE_USED:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_trigger_queue_used);
		shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_CMGR_FAIL_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_cmgr_fail_count);
		shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_PARSE_FAIL_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_parse_fail_count);
		shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_DELETE_FAIL_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_delete_fail_count);
		shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "RX_TRIGGER_QUEUE_DROP_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_trigger_queue_drop_count);
		shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "RX_RESULT_QUEUE_DROP_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.rx_result_queue_drop_count);
		shell_fprintf(sh, SHELL_VT100_COLOR_BLUE, "ASYNC_RESULT_QUEUE_DROP_COUNT:");
		shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, " %u\n",
			      stats.async_result_queue_drop_count);
	}
	shell_fprintf(sh, SHELL_VT100_COLOR_DEFAULT, "\n");

	return 0;
}

SHELL_SUBCMD_ADD((tb45_show_cmds), sms_stat, NULL,
		  "Print SMS health summary (use 'tb45 show sms_stat all' for full counters)",
		  tb45_shell_cmd_sms_stat, 0, 0);

#endif

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
static const struct shell_static_entry tb45_sms_subcmds[] = {
	SHELL_CMD_ARG(send, NULL, "Send SMS: tb45 sms send <phone> <text>",
		      tb45_shell_cmd_sms_send, 1, 255),
	SHELL_CMD_ARG(init, NULL,
		      "SMS init: delegate to tb45 modem restart",
		      tb45_shell_cmd_sms_init, 1, 0),
	SHELL_SUBCMD_SET_END,
};

const union shell_cmd_entry tb45_sms_cmds = {
	.entry = tb45_sms_subcmds,
};

#endif

#if TB45_SHELL_COMMANDS_ENABLED
SHELL_STATIC_SUBCMD_SET_CREATE(tb45_modem_cmds,
	SHELL_CMD(on, NULL, "Power on modem_cellular state machine",
		  tb45_shell_cmd_modem_on),
	SHELL_CMD(off, NULL, "Power off modem_cellular state machine",
		  tb45_shell_cmd_modem_off),
	SHELL_CMD(restart, NULL, "Force off then on recovery",
		  tb45_shell_cmd_modem_restart),
	SHELL_CMD(shutdown, NULL, "Graceful shutdown via PWRKEY/STATUS (leave modem off)",
		  tb45_shell_cmd_modem_shutdown),
	SHELL_CMD(unlock_pin, NULL,
		  "SIM PUK unlock: tb45 modem unlock_pin <pukcode> <pincode>",
		  tb45_shell_cmd_modem_unlock_pin),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_ppp_cmds,
	SHELL_CMD(up, NULL, "Bring PPP interface up", tb45_shell_cmd_ppp_up),
	SHELL_CMD(restart, NULL, "Force PPP recovery: modem_restart/up/route_on/show",
		  tb45_shell_cmd_ppp_restart),
	SHELL_CMD(down, NULL, "Bring PPP interface down", tb45_shell_cmd_ppp_down),
	SHELL_CMD_ARG(default_traffic_route, NULL,
		      "Set default traffic route: tb45 ppp default_traffic_route <on|off>",
		      tb45_shell_cmd_ppp_default_traffic_route, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_SUBCMD_SET_CREATE(tb45_show_cmds, (tb45_cmds));

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_system_cmds,
	SHELL_CMD(reboot, NULL, "Reboot the device", tb45_shell_cmd_system_reboot),
	SHELL_CMD(shell_lock_override, NULL,
		  "Force-unlock shell commands during blocked startup",
		  tb45_cellular_shell_cmd_system_shell_lock_override),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_net_cmds,
	SHELL_CMD_ARG(ping, NULL,
		      "Ping IPv4 host: tb45 net ping <ipv4-address> [count] [timeout_ms] [payload_bytes]",
		      tb45_shell_cmd_net_ping, 1, 4),
	SHELL_SUBCMD_SET_END
);

#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
SHELL_STATIC_SUBCMD_SET_CREATE(tb45_probe_cmds,
	SHELL_CMD(on, NULL, "Enable periodic health probe scheduling",
		  tb45_shell_cmd_probe_on),
	SHELL_CMD(off, NULL, "Disable periodic health probe scheduling",
		  tb45_shell_cmd_probe_off),
	SHELL_SUBCMD_SET_END
);
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_cmds,
	SHELL_CMD(help, NULL, "Show TB45 available commands banner", tb45_shell_cmd_help),
	SHELL_CMD(modem, &tb45_modem_cmds,
		  "Modem controls: tb45 modem <on|off|restart|shutdown|unlock_pin>", NULL),
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
	SHELL_CMD(show, &tb45_show_cmds,
		  TB45_SHELL_SHOW_COMMANDS_TEXT,
		  NULL),
#else
	SHELL_CMD(show, &tb45_show_cmds,
		  TB45_SHELL_SHOW_COMMANDS_TEXT,
		  NULL),
#endif
	SHELL_CMD(net, &tb45_net_cmds, "Network controls: tb45 net <ping>", NULL),
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
	SHELL_CMD(probe, &tb45_probe_cmds, "Probe controls: tb45 probe <on|off>", NULL),
#endif
	SHELL_CMD(ppp, &tb45_ppp_cmds, "PPP controls: tb45 ppp <up|down|default_traffic_route>",
		  NULL),
	#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
SHELL_CMD(sms, &tb45_sms_cmds, "SMS controls: tb45 sms send <phone_number> <message>",
		  NULL),
#endif
	SHELL_CMD(system, &tb45_system_cmds, "System commands", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tb45, &tb45_cmds, "TB45 modem cellular controls", NULL);
#endif /* TB45_SHELL_COMMANDS_ENABLED */
