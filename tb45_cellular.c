#include "tb45_cellular.h"
#include "tb45_ppp_probe.h"

#include <errno.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_l2.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>
#include <zephyr/version.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#ifdef CONFIG_SHELL
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#endif
#if defined(CONFIG_SHELL) && defined(CONFIG_SHELL_BACKEND_SERIAL) && \
	defined(CONFIG_SHELL_LOG_BACKEND) && defined(CONFIG_UART_CONSOLE)
#include "tb45_shell.h"
#endif
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
#include "tb45_sms.h"
#endif
#include "tb45_ping.h"

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
#define TB45_CELLULAR_SHOW_COMMANDS_TEXT "  tb45 show <ppp_info|modem_info|summary|network_modes|isp_list|isp_current|restart_info|probe_info|sms_stat>: Read-only status/info commands\n"
#else
#define TB45_CELLULAR_SHOW_COMMANDS_TEXT "  tb45 show <ppp_info|modem_info|summary|network_modes|isp_list|isp_current|restart_info|probe_info>: Read-only status/info commands\n"
#endif
#ifndef CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS
#if defined(CONFIG_APP_TB45_MODEM_PERIODIC_SCRIPT_MS)
#define CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS CONFIG_APP_TB45_MODEM_PERIODIC_SCRIPT_MS
#else
#define CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS 10000
#endif
#endif

#ifndef CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#if defined(CONFIG_APP_TB45_MODEM_BRINGUP_MAX_RETRIES)
#define CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES CONFIG_APP_TB45_MODEM_BRINGUP_MAX_RETRIES
#else
#define CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES 30
#endif
#endif

LOG_MODULE_DECLARE(modem_cellular_custom, CONFIG_MODEM_LOG_LEVEL);

#ifndef CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT
#define CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT BIT(1)
struct cellular_evt_modem_comms_check_result {
    bool success;
};
#endif

#ifndef CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED
#define CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED BIT(2)
struct cellular_evt_registration_status {
    enum cellular_registration_status status;
};
#endif

#if DT_NODE_EXISTS(DT_ALIAS(modem))
static const struct device *const tb45_cellular_dev = DEVICE_DT_GET(DT_ALIAS(modem));
#else
static const struct device *const tb45_cellular_dev = NULL;
#endif

#if DT_NODE_EXISTS(DT_ALIAS(modem)) && DT_NODE_EXISTS(DT_BUS(DT_ALIAS(modem)))
static const struct device *const tb45_modem_uart_dev = DEVICE_DT_GET(DT_BUS(DT_ALIAS(modem)));
#else
static const struct device *const tb45_modem_uart_dev = NULL;
#endif

#ifdef CONFIG_SHELL
static int cmd_tb45_ppp_restart(const struct shell *shell, size_t argc, char **argv);
static int cmd_tb45_modem_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_on(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_off(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_shutdown(const struct shell *shell, size_t argc, char **argv);
#endif
int tb45_cellular_probe_set_enabled(bool enabled);
int tb45_cellular_probe_get_enabled(bool *enabled_out);
int tb45_cellular_ppp_ready_post_actions(void);
#ifdef CONFIG_SHELL
static int tb45_ppp_recovery_sequence(const struct shell *shell);
#endif
#ifdef CONFIG_SHELL
static int tb45_wait_for_pm_state(const struct shell *shell, enum pm_device_state expected_state,
                                  int timeout_ms, const char *step_name);
static int tb45_modem_suspend_with_retry(const struct shell *shell, const char *label);
static int tb45_shell_modem_off_suspend_start(void);
static int tb45_shell_modem_off_suspend_wait(const struct shell *shell);
static int tb45_modem_restart_suspend_with_retry(const struct shell *shell);
#endif
static int tb45_ppp_ipcp_set_state(struct net_if *iface, bool target_up);
static bool tb45_ppp_iface_link_ready(struct net_if *iface);
static bool tb45_ppp_iface_has_ipv4_addr(struct net_if *iface);
static bool tb45_ppp_iface_runtime_ready(struct net_if *iface);
static bool tb45_ppp_iface_runtime_defaults_ready(struct net_if *iface);
#ifdef CONFIG_SHELL
static int tb45_wait_for_ppp_link_ready(const struct shell *shell, struct net_if *iface,
                                        int timeout_ms);
static int tb45_wait_for_ppp_ipv4_ready(const struct shell *shell, struct net_if *iface,
					int timeout_ms);
#endif
static void tb45_startup_ppp_autoup_work_handler(struct k_work *work);
static int tb45_cellular_restore_ppp_runtime_defaults(void);
static void tb45_startup_ppp_check_route_ready_restart(const char *reason);
static void tb45_reset_bringup_runtime_state(void);
static int tb45_schedule_ppp_ready_post_actions(bool continue_startup_finalize);
int modem_cellular_custom_trigger_ppp_check_route_ready_restart(const struct device *dev);
int modem_cellular_custom_get_cpin_state(const struct device *dev, int *state_out);
int modem_cellular_custom_submit_sim_puk_unlock(const struct device *dev, const char *puk,
                                                const char *new_pin);
int modem_cellular_custom_get_current_network_mode_code(const struct device *dev, int *mode_code);
extern struct k_work_q low_priority_wq;

/*
 * Runtime cellular configuration owned by this library. Populated by
 * tb45_cellular_init() (typically called from main()). The weak-extern getters
 * below satisfy the modem_cellular_custom runtime config contract, including
 * the newer auth_type support.
 */
#define TB45_CELL_FIELD_MAX 64
#define TB45_CELL_APN_MAX_LEN 31
#define TB45_CELL_SIM_PIN_MAX_LEN 8
#define TB45_CELL_CARRIER_ID_MAX_LEN 6
#define TB45_CARRIER_ID_AUTO "AUTO"
static char tb45_cell_apn[TB45_CELL_APN_MAX_LEN + 1] = {0};
static char tb45_cell_username[TB45_CELL_FIELD_MAX] = {0};
static char tb45_cell_password[TB45_CELL_FIELD_MAX] = {0};
static char tb45_cell_sim_pin[TB45_CELL_SIM_PIN_MAX_LEN + 1] = {0};
static char tb45_cell_carrier_id[TB45_CELL_CARRIER_ID_MAX_LEN + 1] = TB45_CARRIER_ID_AUTO;
static bool tb45_cell_apn_set = false;
static bool tb45_cell_apn_invalid = false;
static bool tb45_cell_username_set = false;
static bool tb45_cell_password_set = false;
static bool tb45_cell_sim_pin_set = false;
static bool tb45_cell_sim_pin_invalid = false;
static bool tb45_cell_carrier_id_set = false;
static uint8_t tb45_cell_auth_type = TB45_CELL_AUTH_NONE;

static void tb45_cell_store_field(char *dst, size_t dst_size, bool *set_flag,
                                  const char *src)
{
    if (src == NULL) {
        dst[0] = '\0';
        *set_flag = false;
        return;
    }

    size_t copy_len = strlen(src);
    if (copy_len >= dst_size) {
        copy_len = dst_size - 1U;
    }
    memcpy(dst, src, copy_len);
    dst[copy_len] = '\0';
    *set_flag = true;
}

static void tb45_cell_store_apn(const char *src)
{
    size_t len;

    tb45_cell_apn[0] = '\0';
    tb45_cell_apn_set = false;
    tb45_cell_apn_invalid = false;

    if (src == NULL) {
        return;
    }

    len = strlen(src);
    if (len > TB45_CELL_APN_MAX_LEN) {
        tb45_cell_apn_invalid = true;
        return;
    }

    memcpy(tb45_cell_apn, src, len + 1U);
    tb45_cell_apn_set = true;
}

static void tb45_cell_store_sim_pin(const char *src)
{
    size_t len;

    tb45_cell_sim_pin[0] = '\0';
    tb45_cell_sim_pin_set = false;
    tb45_cell_sim_pin_invalid = false;

    if (src == NULL) {
        return;
    }

    if ((src[0] == '\0') || ((src[0] == '?') && (src[1] == '\0'))) {
        (void)snprintf(tb45_cell_sim_pin, sizeof(tb45_cell_sim_pin), "%s", src);
        tb45_cell_sim_pin_set = true;
        return;
    }

    len = strlen(src);
    if ((len == 0U) || (len > TB45_CELL_SIM_PIN_MAX_LEN)) {
        tb45_cell_sim_pin_invalid = true;
        return;
    }

    for (size_t i = 0U; i < len; i++) {
        if (!isdigit((unsigned char)src[i])) {
            tb45_cell_sim_pin_invalid = true;
            return;
        }
    }

    memcpy(tb45_cell_sim_pin, src, len + 1U);
    tb45_cell_sim_pin_set = true;
}

static bool tb45_cell_is_auto_carrier_id(const char *carrier_id)
{
    return (carrier_id != NULL) &&
           (toupper((unsigned char)carrier_id[0]) == 'A') &&
           (toupper((unsigned char)carrier_id[1]) == 'U') &&
           (toupper((unsigned char)carrier_id[2]) == 'T') &&
           (toupper((unsigned char)carrier_id[3]) == 'O') &&
           (carrier_id[4] == '\0');
}

static bool tb45_cell_is_valid_plmn_carrier_id(const char *carrier_id)
{
    if (carrier_id == NULL) {
        return false;
    }

    size_t len = strlen(carrier_id);
    if ((len != 5U) && (len != 6U)) {
        return false;
    }

    for (size_t i = 0U; i < len; i++) {
        if (!isdigit((unsigned char)carrier_id[i])) {
            return false;
        }
    }

    return true;
}

static void tb45_cell_store_carrier_id_or_auto(const char *src)
{
    if ((src == NULL) || (src[0] == '\0') || tb45_cell_is_auto_carrier_id(src)) {
        strncpy(tb45_cell_carrier_id, TB45_CARRIER_ID_AUTO, sizeof(tb45_cell_carrier_id) - 1U);
        tb45_cell_carrier_id[sizeof(tb45_cell_carrier_id) - 1U] = '\0';
        return;
    }

    if (tb45_cell_is_valid_plmn_carrier_id(src)) {
        strncpy(tb45_cell_carrier_id, src, sizeof(tb45_cell_carrier_id) - 1U);
        tb45_cell_carrier_id[sizeof(tb45_cell_carrier_id) - 1U] = '\0';
        return;
    }

    LOG_WRN("Invalid carrier_id \"%s\"; using AUTO (expected AUTO or 5 or 6 digits MCC+MNC)", src);
    strncpy(tb45_cell_carrier_id, TB45_CARRIER_ID_AUTO, sizeof(tb45_cell_carrier_id) - 1U);
    tb45_cell_carrier_id[sizeof(tb45_cell_carrier_id) - 1U] = '\0';
}

const char *tb45_main_get_cellapn(void)
{
    return tb45_cell_apn_set ? tb45_cell_apn : NULL;
}

bool tb45_main_get_cellapn_invalid(void)
{
    return tb45_cell_apn_invalid;
}

const char *tb45_main_get_cellun(void)
{
    return tb45_cell_username_set ? tb45_cell_username : NULL;
}

const char *tb45_main_get_cellpw(void)
{
    return tb45_cell_password_set ? tb45_cell_password : NULL;
}

const char *tb45_main_get_cellpin(void)
{
    return tb45_cell_sim_pin_set ? tb45_cell_sim_pin : NULL;
}

bool tb45_main_get_cellpin_invalid(void)
{
    return tb45_cell_sim_pin_invalid;
}

const char *tb45_main_get_cellcarrier(void)
{
    return tb45_cell_carrier_id_set ? tb45_cell_carrier_id : NULL;
}

int tb45_main_get_cellauth(void)
{
    return (int)tb45_cell_auth_type;
}

static void tb45_startup_finalize_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_finalize_work, tb45_startup_finalize_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_ppp_autoup_work, tb45_startup_ppp_autoup_work_handler);
static atomic_t tb45_shell_commands_printed = ATOMIC_INIT(0);
static atomic_t tb45_shell_menu_loaded = ATOMIC_INIT(0);
static atomic_t tb45_modem_registration_ready = ATOMIC_INIT(0);
static atomic_t tb45_startup_finalize_scheduled = ATOMIC_INIT(0);
static atomic_t tb45_ppp_down_triggered = ATOMIC_INIT(0);
static atomic_t tb45_modem_off_triggered = ATOMIC_INIT(0);
static atomic_t tb45_raw_shutdown_triggered = ATOMIC_INIT(0);
static atomic_t tb45_ppp_recovery_in_progress = ATOMIC_INIT(0);
static atomic_t tb45_ppp_recovery_internal_call = ATOMIC_INIT(0);
static atomic_t tb45_ppp_restart_manual_active = ATOMIC_INIT(0);
static atomic_t tb45_ppp_restart_cancel_requested = ATOMIC_INIT(0);
static atomic_t tb45_startup_ppp_autoup_armed = ATOMIC_INIT(0);
static atomic_t tb45_startup_ppp_autoup_done = ATOMIC_INIT(0);
static atomic_t tb45_restart_count_ppp = ATOMIC_INIT(0);
static atomic_t tb45_restart_count_modem = ATOMIC_INIT(0);
static atomic_t tb45_restart_count_full_bringup = ATOMIC_INIT(0);
static struct k_work_q *tb45_periodic_wq = NULL;
static const struct device *tb45_runtime_modem_uart_dev = NULL;
static int tb45_startup_ppp_ready_stable_elapsed_ms = 0;
static int tb45_startup_ppp_check_internet_reachability_attempt = 0;
static int tb45_startup_finalize_stage = 0;
static int64_t tb45_startup_ppp_autoup_deadline_ms = 0;
#ifdef CONFIG_SHELL
static void tb45_shell_modem_off_suspend_work_handler(struct k_work *work);
static K_WORK_DEFINE(tb45_shell_modem_off_suspend_work, tb45_shell_modem_off_suspend_work_handler);
static K_SEM_DEFINE(tb45_shell_modem_off_suspend_done, 0, 1);
static atomic_t tb45_shell_modem_off_suspend_active = ATOMIC_INIT(0);
static atomic_t tb45_shell_modem_off_suspend_result_ready = ATOMIC_INIT(0);
static int tb45_shell_modem_off_suspend_result = 0;
#endif

/*
 * PPP health checks use ICMP ping reachability.
 */
#define TB45_PPP_RESTART_MAX_ATTEMPTS CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#define TB45_PPP_RESTART_RETRY_DELAY_MS 1000
#define TB45_PPP_RESTART_STEP_PAUSE_MS 3000
#define TB45_SDWN_HARD_OFF_HOLD_MS 500
#define TB45_SDWN_REENABLE_DELAY_MS 100
#define TB45_PWRKEY_SHUTDOWN_PULSE_MS 2600
#define TB45_STATUS_OFF_TIMEOUT_MS 30000
#define TB45_MODEM_SUSPEND_RETRY_COUNT 3
#define TB45_MODEM_SUSPEND_RETRY_DELAY_1_MS 200
#define TB45_MODEM_SUSPEND_RETRY_DELAY_2_MS 400
#define TB45_MODEM_SUSPEND_RETRY_DELAY_3_MS 800
#define TB45_STEP_WAIT_INTERVAL_MS 100
#define TB45_PPP_READY_STABLE_MS 2500
#define TB45_WAIT_PM_ACTIVE_TIMEOUT_MS 10000
#define TB45_WAIT_PPP_READY_TIMEOUT_MS 30000
#define TB45_PPP_CHECK_ROUTE_READY_TIMEOUT_MS \
    ((CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS > 60000) ? \
     CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS : 60000)
#define TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS 500
#define TB45_SHELL_ASCII_CTRL_C 0x03U

#if IS_ENABLED(CONFIG_MODEM_LOG_LEVEL_DBG)
#define TB45_PERIODIC_PROBE_LOG_DBG(...) LOG_DBG(__VA_ARGS__)
#else
#define TB45_PERIODIC_PROBE_LOG_DBG(...) do { } while (0)
#endif

static atomic_t tb45_runtime_wq_not_ready_warned = ATOMIC_INIT(0);

static struct k_work_q *tb45_runtime_work_q(void)
{
    if (tb45_periodic_wq != NULL) {
        return tb45_periodic_wq;
    }

    return &low_priority_wq;
}

static bool tb45_runtime_wq_ready(void)
{
    struct k_work_q *wq = tb45_runtime_work_q();
    bool ready = (wq != NULL) && (k_work_queue_thread_get(wq) != NULL);

    if (ready) {
        atomic_set(&tb45_runtime_wq_not_ready_warned, 0);
    }

    return ready;
}

static int tb45_runtime_wq_not_ready(const char *op)
{
    if (atomic_cas(&tb45_runtime_wq_not_ready_warned, 0, 1)) {
        LOG_WRN("TB45 runtime queue not ready: skipping %s until work_queues_init() completes",
                op);
    }

    return -EAGAIN;
}

static int tb45_schedule_work(struct k_work_delayable *dwork, k_timeout_t delay)
{
    if (!tb45_runtime_wq_ready()) {
        return tb45_runtime_wq_not_ready("schedule");
    }

    return k_work_schedule_for_queue(tb45_runtime_work_q(), dwork, delay);
}

static int tb45_reschedule_work(struct k_work_delayable *dwork, k_timeout_t delay)
{
    if (!tb45_runtime_wq_ready()) {
        return tb45_runtime_wq_not_ready("reschedule");
    }

    return k_work_reschedule_for_queue(tb45_runtime_work_q(), dwork, delay);
}

static int tb45_submit_work(struct k_work *work)
{
    if (!tb45_runtime_wq_ready()) {
        return tb45_runtime_wq_not_ready("submit");
    }

    return k_work_submit_to_queue(tb45_runtime_work_q(), work);
}

static int tb45_get_runtime_baudrate(uint32_t *baudrate)
{
    if ((tb45_runtime_modem_uart_dev == NULL) || (baudrate == NULL)) {
        return -ENODEV;
    }

    struct uart_config cfg = {0};
    int ret = uart_config_get(tb45_runtime_modem_uart_dev, &cfg);
    if (ret < 0) {
        return ret;
    }

	*baudrate = cfg.baudrate;
	return 0;
}

static int tb45_get_runtime_network_mode(int *mode_code)
{
	if ((tb45_cellular_dev == NULL) || (mode_code == NULL) || !device_is_ready(tb45_cellular_dev)) {
		return -ENODEV;
	}

	return modem_cellular_custom_get_current_network_mode_code(tb45_cellular_dev, mode_code);
}

static const char *tb45_cnmp_mode_to_str(int code)
{
    switch (code) {
    case 2:  return "AUTO";
    case 9:  return "GSM+LTE";
    case 10: return "GSM+WCDMA+LTE";
    case 13: return "GSM only";
    case 14: return "WCDMA only";
    case 19: return "GSM+WCDMA";
    case 22: return "LTE+WCDMA";
    case 38: return "LTE only";
    case 39: return "GSM+WCDMA+LTE";
    case 48: return "LTE profile";
    case 51: return "NR5G/LTE auto";
    case 54: return "LTE+WCDMA profile";
    case 59: return "LTE profile";
    case 60: return "LTE profile";
    case 63: return "LTE profile";
    case 67: return "LTE profile";
    default: return "UNKNOWN";
    }
}

int tb45_cellular_get_runtime_baudrate(uint32_t *baudrate)
{
	return tb45_get_runtime_baudrate(baudrate);
}

int tb45_cellular_get_runtime_network_mode(int *mode_code)
{
	return tb45_get_runtime_network_mode(mode_code);
}

const char *tb45_cellular_network_mode_to_str(int code)
{
	return tb45_cnmp_mode_to_str(code);
}

int tb45_cellular_get_probe_info(struct tb45_cellular_probe_info *info)
{
	return tb45_ppp_probe_get_info(info);
}

int tb45_cellular_get_restart_info(struct tb45_cellular_restart_info *info)
{
	if (info == NULL) {
		return -EINVAL;
	}

	info->ppp_restart_count = atomic_get(&tb45_restart_count_ppp);
	info->modem_restart_count = atomic_get(&tb45_restart_count_modem);
	info->full_bringup_restart_count = atomic_get(&tb45_restart_count_full_bringup);

	return 0;
}

#ifdef CONFIG_SHELL
static int tb45_check_shell_menu_loaded(const struct shell *shell)
{
    if (atomic_get(&tb45_shell_menu_loaded) != 0) {
        if ((atomic_get(&tb45_ppp_recovery_in_progress) != 0) &&
            (atomic_get(&tb45_ppp_recovery_internal_call) == 0)) {
            shell_error(shell, "Shell commands temporarily offline");
            shell_error(shell, "PPP recovery in progress - please wait");
            return -EBUSY;
        }

        return 0;
    }

    shell_error(shell, "Shell commands temporarily offline");
    shell_error(shell, "System is still initializing - please wait");
	shell_error(shell, "Commands will be available when fully loaded");
	return -EBUSY;
}

int tb45_cellular_shell_menu_require_loaded(const struct shell *shell)
{
    return tb45_check_shell_menu_loaded(shell);
}

int tb45_cellular_shell_check_menu_loaded(const struct shell *shell)
{
	return tb45_check_shell_menu_loaded(shell);
}

int tb45_cellular_shell_cmd_system_shell_lock_override(const struct shell *shell, size_t argc,
					       char **argv)
{
	ARG_UNUSED(argv);

	if (argc != 1U) {
		shell_error(shell, "Usage: tb45 system shell_lock_override");
		return -EINVAL;
	}

	atomic_set(&tb45_shell_menu_loaded, 1);
	shell_warn(shell, "Shell lock override enabled");
	shell_warn(shell, "Startup gating bypassed; commands are now accessible");

	return 0;
}

#endif /* CONFIG_SHELL */

static bool tb45_ppp_iface_link_ready(struct net_if *iface)
{
    if (iface == NULL) {
        return false;
    }

    bool iface_up = net_if_is_up(iface);
    bool iface_dormant = net_if_is_dormant(iface);
    bool carrier_ok = net_if_is_carrier_ok(iface);

    return iface_up && !iface_dormant && carrier_ok;
}

int tb45_cellular_probe_set_enabled(bool enabled)
{
    return tb45_ppp_probe_set_enabled(enabled);
}

int tb45_cellular_probe_get_enabled(bool *enabled_out)
{
    return tb45_ppp_probe_get_enabled(enabled_out);
}

static int tb45_cellular_restore_ppp_runtime_defaults(void)
{
    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));

    if (iface == NULL) {
        return -ENODEV;
    }

    net_if_set_default(iface);
    if (net_if_get_default() != iface) {
        return -EIO;
    }

    return 0;
}

int tb45_cellular_ppp_ready_post_actions(void)
{
	int ret;

	ret = tb45_ppp_probe_on_ppp_ready_post_actions();
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
	ret = tb45_sms_receive_recover_after_modem_reconnect();
	if (ret < 0) {
		return ret;
	}

#endif
	return 0;
}

const struct device *tb45_cellular_get_device(void)
{
	return tb45_cellular_dev;
}

int tb45_cellular_submit_sim_puk_unlock(const char *puk, const char *new_pin)
{
	if ((puk == NULL) || (new_pin == NULL)) {
		return -EINVAL;
	}

	if ((tb45_cellular_dev == NULL) || !device_is_ready(tb45_cellular_dev)) {
		return -ENODEV;
	}

	return modem_cellular_custom_submit_sim_puk_unlock(tb45_cellular_dev, puk, new_pin);
}

int tb45_cellular_get_cpin_state(int *state_out)
{
	if (state_out == NULL) {
		return -EINVAL;
	}

	if ((tb45_cellular_dev == NULL) || !device_is_ready(tb45_cellular_dev)) {
		return -ENODEV;
	}

	return modem_cellular_custom_get_cpin_state(tb45_cellular_dev, state_out);
}

#ifdef CONFIG_SHELL
static bool tb45_ppp_restart_interrupt_enabled(const struct shell *shell)
{
    return (shell != NULL) && (atomic_get(&tb45_ppp_restart_manual_active) != 0);
}

static bool tb45_ppp_restart_cancel_pending(const struct shell *shell)
{
    if (!tb45_ppp_restart_interrupt_enabled(shell)) {
        return false;
    }

    if (atomic_get(&tb45_ppp_restart_cancel_requested) != 0) {
        return true;
    }

    if ((shell->iface == NULL) || (shell->iface->api == NULL) || (shell->iface->api->read == NULL)) {
        return false;
    }

    uint8_t rx_buf[16];

    while (true) {
        size_t cnt = 0;
        int ret = shell->iface->api->read(shell->iface, rx_buf, sizeof(rx_buf), &cnt);
        if ((ret < 0) || (cnt == 0)) {
            break;
        }

        for (size_t i = 0; i < cnt; i++) {
            if (rx_buf[i] == TB45_SHELL_ASCII_CTRL_C) {
                atomic_set(&tb45_ppp_restart_cancel_requested, 1);
                return true;
            }
        }
    }

    return false;
}

static int tb45_ppp_restart_sleep_interruptible(const struct shell *shell, int delay_ms)
{
    int elapsed_ms = 0;

    while (elapsed_ms < delay_ms) {
        if (tb45_ppp_restart_cancel_pending(shell)) {
            return -ECANCELED;
        }

        int slice_ms = delay_ms - elapsed_ms;
        if (slice_ms > TB45_STEP_WAIT_INTERVAL_MS) {
            slice_ms = TB45_STEP_WAIT_INTERVAL_MS;
        }

        k_msleep(slice_ms);
        elapsed_ms += slice_ms;
    }

    if (tb45_ppp_restart_cancel_pending(shell)) {
        return -ECANCELED;
    }

    return 0;
}

static int tb45_wait_for_ppp_link_ready(const struct shell *shell, struct net_if *iface,
                                        int timeout_ms)
{
    if (iface == NULL) {
        return -EINVAL;
    }

    int elapsed_ms = 0;
    int ready_stable_elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms) {
        if (tb45_ppp_iface_link_ready(iface)) {
            ready_stable_elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
            if (ready_stable_elapsed_ms >= TB45_PPP_READY_STABLE_MS) {
                return 0;
            }
        } else {
            ready_stable_elapsed_ms = 0;
        }

        int wait_ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_STEP_WAIT_INTERVAL_MS);
        if (wait_ret < 0) {
            return wait_ret;
        }

        elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
    }

    return -ETIMEDOUT;
}

static int tb45_wait_for_ppp_ipv4_ready(const struct shell *shell, struct net_if *iface,
					int timeout_ms)
{
    if (iface == NULL) {
        return -EINVAL;
    }

    int elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms) {
        if (tb45_ppp_iface_link_ready(iface) && tb45_ppp_iface_has_ipv4_addr(iface)) {
            return 0;
        }

        int wait_ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_STEP_WAIT_INTERVAL_MS);
        if (wait_ret < 0) {
            return wait_ret;
        }

        elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
    }

    return -ETIMEDOUT;
}

#endif /* CONFIG_SHELL */

static int tb45_ppp_ipcp_set_state(struct net_if *iface, bool target_up)
{
    if (iface == NULL) {
        return -EINVAL;
    }

    int ret = target_up ? net_if_up(iface) : net_if_down(iface);
    if ((ret == 0) || (ret == -EALREADY)) {
        atomic_set(&tb45_ppp_down_triggered, target_up ? 0 : 1);
    }

    return ret;
}

static void tb45_reset_bringup_runtime_state(void)
{
    atomic_set(&tb45_startup_ppp_autoup_armed, 0);
    atomic_set(&tb45_startup_ppp_autoup_done, 0);
    atomic_set(&tb45_modem_registration_ready, 0);
    tb45_ppp_probe_reset_runtime_state();
    tb45_startup_ppp_ready_stable_elapsed_ms = 0;
    tb45_startup_ppp_check_internet_reachability_attempt = 0;
    tb45_startup_finalize_stage = 0;
    tb45_startup_ppp_autoup_deadline_ms = 0;
    atomic_set(&tb45_startup_finalize_scheduled, 0);
    atomic_set(&tb45_shell_commands_printed, 0);
    (void)k_work_cancel_delayable(&tb45_startup_finalize_work);
    (void)k_work_cancel_delayable(&tb45_startup_ppp_autoup_work);
}

static void tb45_startup_ppp_check_route_ready_restart(const char *reason)
{
    atomic_inc(&tb45_restart_count_full_bringup);

    ARG_UNUSED(reason);


    tb45_reset_bringup_runtime_state();
    atomic_set(&tb45_startup_ppp_autoup_done, 1);

    if ((tb45_cellular_dev == NULL) || !device_is_ready(tb45_cellular_dev)) {
        LOG_ERR("TB45 startup: restart skipped");
        if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
            (void)tb45_schedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
        }
        return;
    }

    int ret = modem_cellular_custom_trigger_ppp_check_route_ready_restart(tb45_cellular_dev);
    if (ret < 0) {
        LOG_ERR("TB45 startup: ppp_check_route_ready restart trigger failed (%d)", ret);
        if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
            (void)tb45_schedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
        }
    }
}

static void tb45_startup_ppp_autoup_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if ((atomic_get(&tb45_startup_ppp_autoup_armed) == 0) ||
        (atomic_get(&tb45_startup_ppp_autoup_done) != 0)) {
        return;
    }

    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        LOG_WRN("TB45 startup: PPP auto bring-up skipped (CONFIG_NET_L2_PPP=n)");
        atomic_set(&tb45_startup_ppp_autoup_done, 1);
        if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
            (void)tb45_schedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
        }
        return;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    int64_t now_ms = k_uptime_get();
    bool ppp_ready_stable = false;
    bool route_ready = false;

    if (iface != NULL) {
        int ret = tb45_ppp_ipcp_set_state(iface, true);
        if ((ret != 0) && (ret != -EALREADY)) {
            LOG_WRN("TB45 startup: PPP IPCP up request pending (%d)", ret);
        }

        if (tb45_ppp_iface_link_ready(iface) &&
            tb45_ppp_iface_has_ipv4_addr(iface)) {
            tb45_startup_ppp_ready_stable_elapsed_ms += TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS;
        } else {
            tb45_startup_ppp_ready_stable_elapsed_ms = 0;
        }

        ppp_ready_stable = (tb45_startup_ppp_ready_stable_elapsed_ms >= TB45_PPP_READY_STABLE_MS);
        if (ppp_ready_stable) {
            net_if_set_default(iface);
            route_ready = (net_if_get_default() == iface);
            if (route_ready) {
                if (atomic_get(&tb45_ppp_recovery_in_progress) != 0) {
                    atomic_set(&tb45_startup_ppp_autoup_done, 1);
                    return;
                }

                atomic_set(&tb45_startup_ppp_autoup_done, 1);
                ret = tb45_schedule_ppp_ready_post_actions(true);
                if (ret < 0) {
                    LOG_WRN("TB45 startup: deferred PPP-ready post actions could not be scheduled (%d)", ret);
                }
                return;
            }
        }
    } else {
        tb45_startup_ppp_ready_stable_elapsed_ms = 0;
    }

    /* If TB45_PPP_RESTART_MAX_ATTEMPTS = 0, then retry INFINITELY */
    if (TB45_PPP_RESTART_MAX_ATTEMPTS != 0) {
        if (now_ms >= tb45_startup_ppp_autoup_deadline_ms) {
            const char *reason = "PPP IPCP link not active/ready";

            if ((iface != NULL) && ppp_ready_stable) {
                reason = "PPP active/ready, but default route did not switch to PPP";
            }

            tb45_startup_ppp_check_route_ready_restart(reason);
            return;
        }
    }

    (void)tb45_reschedule_work(&tb45_startup_ppp_autoup_work,
                            K_MSEC(TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS));
}

#ifdef CONFIG_SHELL
static void tb45_print_available_commands_menu(const struct shell *shell)
{
    if (shell == NULL) {
        return;
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "#########################################################\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_MAGENTA, "Available commands:\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "-------------------------------------------------\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  help: list all the available commands\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 help: Show this available commands banner\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE,
                  "  modem at [command] [expected_response] [timeout_ms]: Send AT command\n");
    shell_fprintf(shell, SHELL_NORMAL, "    EXAMPLES:\n");
    shell_fprintf(shell, SHELL_NORMAL, "      modem at AT+CSQ OK\n");
    shell_fprintf(shell, SHELL_NORMAL, "      modem at AT+COPS=? OK 120000\n");
    shell_fprintf(shell, SHELL_NORMAL, "      modem at AT+CGDATA=\"PPP\",1 CONNECT\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 modem <on|off|restart|shutdown|unlock_pin>: Modem controls\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 modem on: Power on modem_cellular state machine (if off)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 modem off: Power off modem_cellular state machine (if on)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 modem restart: Force off then on recovery path\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 modem shutdown: Graceful shutdown via PWRKEY/STATUS (leave modem off)\n");
	shell_fprintf(shell, SHELL_NORMAL,
	              "    tb45 modem unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 modem restart)\n");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN,
                  TB45_CELLULAR_SHOW_COMMANDS_TEXT);
#else
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN,
                  TB45_CELLULAR_SHOW_COMMANDS_TEXT);
#endif
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show ppp_info: Print PPP interface status/IP\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show modem_info: Print modem info via cellular API\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show summary: Print both modem_info and ppp_info\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show network_modes: Print CNMP codes and meanings\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show isp_list: Scan and show ISP list (AT+COPS=?)\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show isp_current: Show current ISP (AT+COPS?)\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show restart_info: Print restart counters since boot\n");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show probe_info: Print periodic internet probe counters\n");
#endif
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)\n");
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE,
                  "  tb45 ppp <up|down|default_traffic_route>: PPP controls\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp up: Bring PPP interface up\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp down: Bring PPP interface down\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet\n");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 probe <on|off>: Probe controls\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 probe on: Enable periodic health probe scheduling\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 probe off: Disable periodic health probe scheduling\n");
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 system <reboot|shell_lock_override>: System commands\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 system shell_lock_override: Force-unlock shell commands during blocked startup\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host\n");
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 sms send <phone> <text>: Send SMS message\n");
#endif
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "#########################################################\n\n");
}

void tb45_cellular_shell_print_available_commands_menu(const struct shell *shell)
{
	tb45_print_available_commands_menu(shell);
}
#endif


#define TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS 3
#define TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS_ONLY 4

static int tb45_schedule_ppp_ready_post_actions(bool continue_startup_finalize)
{
    tb45_startup_finalize_stage = continue_startup_finalize ?
        TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS :
        TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS_ONLY;

    if (!atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
        return 0;
    }

    int ret = tb45_schedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
    if (ret < 0) {
        atomic_set(&tb45_startup_finalize_scheduled, 0);
    }

    return ret;
}

static void tb45_startup_finalize_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if ((tb45_startup_finalize_stage == TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS) ||
        (tb45_startup_finalize_stage == TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS_ONLY)) {
        bool post_actions_only =
            (tb45_startup_finalize_stage == TB45_STARTUP_FINALIZE_STAGE_POST_ACTIONS_ONLY);
        int ret = tb45_cellular_ppp_ready_post_actions();
        if (ret < 0) {
            LOG_WRN("TB45: deferred PPP-ready post actions failed (%d)", ret);
        }

        if (post_actions_only) {
            tb45_startup_finalize_stage = 2;
            atomic_set(&tb45_startup_finalize_scheduled, 0);
            return;
        }

        tb45_startup_finalize_stage = 0;
    }

    if (tb45_startup_finalize_stage >= 2) {
        atomic_set(&tb45_startup_finalize_scheduled, 0);
        return;
    }

    if (tb45_startup_finalize_stage == 0) {
        tb45_startup_finalize_stage = 1;
        tb45_startup_ppp_check_internet_reachability_attempt = 0;

        (void)tb45_reschedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
        return;
    }

    if (tb45_startup_finalize_stage == 1) {
        bool ppp_check_internet_reachability_done = true;

        /* Product decision: do not gate boot finalization on internet probe.
         * Runtime periodic probe/recovery owns health checking.
         */
        tb45_startup_ppp_check_internet_reachability_attempt = 0;

        if (IS_ENABLED(CONFIG_APP_TB45_SHOW_AVAILABLE_COMMANDS_MENU)) {
#ifdef CONFIG_SHELL
            const struct shell *shell = shell_backend_uart_get_ptr();

            if (shell != NULL) {
                tb45_print_available_commands_menu(shell);
            } else {
                LOG_INF("");
                LOG_INF("#########################################################");
                LOG_INF("Available commands:");
                LOG_INF("---------------------------------------------------------");
                LOG_INF("  help: list all the available commands");
                LOG_INF("  tb45 help: Show this available commands banner");
                LOG_INF("  modem at [command] [expected_response] [timeout_ms]: Send AT command");
                LOG_INF("    EXAMPLES:");
                LOG_INF("      modem at AT+CSQ OK");
                LOG_INF("      modem at AT+COPS=? OK 120000");
                LOG_INF("      modem at AT+CGDATA=\"PPP\",1 CONNECT");
                LOG_INF("  tb45 modem <on|off|restart|shutdown|unlock_pin>: Modem controls");
                LOG_INF("    tb45 modem on: Power on modem_cellular state machine (if off)");
                LOG_INF("    tb45 modem off: Power off modem_cellular state machine (if on)");
                LOG_INF("    tb45 modem restart: Force off then on recovery path");
                LOG_INF("    tb45 modem shutdown: Graceful shutdown via PWRKEY/STATUS (leave modem off)");
                LOG_INF("    tb45 modem unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 modem restart)");
                #if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
                LOG_INF("%s", TB45_CELLULAR_SHOW_COMMANDS_TEXT);
                #else
                LOG_INF("%s", TB45_CELLULAR_SHOW_COMMANDS_TEXT);
                #endif
                LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
                LOG_INF("    tb45 show modem_info: Print modem info via cellular API");
                LOG_INF("    tb45 show summary: Print both modem_info and ppp_info");
                LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
                LOG_INF("    tb45 show isp_list: Scan and show ISP list (AT+COPS=?)");
                LOG_INF("    tb45 show isp_current: Show current ISP (AT+COPS?)");
                LOG_INF("    tb45 show restart_info: Print restart counters since boot");
                #if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
                LOG_INF("    tb45 show probe_info: Print periodic internet probe counters");
                #endif
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
                LOG_INF("    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)");
#endif
                LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
                LOG_INF("    tb45 ppp up: Bring PPP interface up");
                LOG_INF("    tb45 ppp down: Bring PPP interface down");
                LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
                LOG_INF("  tb45 probe <on|off>: Probe controls");
                LOG_INF("    tb45 probe on: Enable periodic health probe scheduling");
                LOG_INF("    tb45 probe off: Disable periodic health probe scheduling");
#endif
                LOG_INF("  tb45 system <reboot|shell_lock_override>: System commands");
                LOG_INF("    tb45 system shell_lock_override: Force-unlock shell commands during blocked startup");
                LOG_INF("  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host");
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
                LOG_INF("  tb45 sms send <phone> <text>: Send SMS message");
#endif

                LOG_INF("#########################################################");
                LOG_INF("");
            }
#else
            LOG_INF("");
            LOG_INF("#########################################################");
            LOG_INF("Available commands:");
            LOG_INF("---------------------------------------------------------");
            LOG_INF("  help: list all the available commands");
            LOG_INF("  tb45 help: Show this available commands banner");
            LOG_INF("  modem at [command] [expected_response] [timeout_ms]: Send AT command");
            LOG_INF("    EXAMPLES:");
            LOG_INF("      modem at AT+CSQ OK");
            LOG_INF("      modem at AT+COPS=? OK 120000");
            LOG_INF("      modem at AT+CGDATA=\"PPP\",1 CONNECT");
            LOG_INF("  tb45 modem <on|off|restart|shutdown|unlock_pin>: Modem controls");
            LOG_INF("    tb45 modem on: Power on modem_cellular state machine (if off)");
            LOG_INF("    tb45 modem off: Power off modem_cellular state machine (if on)");
            LOG_INF("    tb45 modem restart: Force off then on recovery path");
            LOG_INF("    tb45 modem shutdown: Graceful shutdown via PWRKEY/STATUS (leave modem off)");
            LOG_INF("    tb45 modem unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 modem restart)");
            #if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
            LOG_INF("%s", TB45_CELLULAR_SHOW_COMMANDS_TEXT);
            #else
            LOG_INF("%s", TB45_CELLULAR_SHOW_COMMANDS_TEXT);
            #endif
            LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
            LOG_INF("    tb45 show modem_info: Print modem info via cellular API");
            LOG_INF("    tb45 show summary: Print both modem_info and ppp_info");
            LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
            LOG_INF("    tb45 show isp_list: Scan and show ISP list (AT+COPS=?)");
            LOG_INF("    tb45 show isp_current: Show current ISP (AT+COPS?)");
            LOG_INF("    tb45 show restart_info: Print restart counters since boot");
            #if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
            LOG_INF("    tb45 show probe_info: Print periodic internet probe counters");
            #endif
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
            LOG_INF("    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)");
#endif
            LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
            LOG_INF("    tb45 ppp up: Bring PPP interface up");
            LOG_INF("    tb45 ppp down: Bring PPP interface down");
            LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
            LOG_INF("  tb45 probe <on|off>: Probe controls");
            LOG_INF("    tb45 probe on: Enable periodic health probe scheduling");
            LOG_INF("    tb45 probe off: Disable periodic health probe scheduling");
#endif
            LOG_INF("  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host");
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
            LOG_INF("  tb45 sms send <phone> <text>: Send SMS message");
#endif
            LOG_INF("#########################################################");
            LOG_INF("");
#endif
        }
        atomic_set(&tb45_shell_menu_loaded, 1);
        tb45_startup_finalize_stage = 2;
        tb45_ppp_probe_on_startup_finalize(ppp_check_internet_reachability_done);
        atomic_set(&tb45_startup_finalize_scheduled, 0);
        return;
    }
}

static void tb45_cellular_event_cb(const struct device *dev, enum cellular_event event,
                                   const void *payload, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    bool ready_to_print = false;

    if (event == CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT) {
        const struct cellular_evt_modem_comms_check_result *result =
            (const struct cellular_evt_modem_comms_check_result *)payload;
        ready_to_print = (result != NULL) && result->success;
    } else if (event == CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED) {
        const struct cellular_evt_registration_status *reg =
            (const struct cellular_evt_registration_status *)payload;
        bool registered_ready = (reg != NULL) &&
                                ((reg->status == CELLULAR_REGISTRATION_REGISTERED_HOME) ||
                                 (reg->status == CELLULAR_REGISTRATION_REGISTERED_ROAMING));
        atomic_set(&tb45_modem_registration_ready, registered_ready ? 1 : 0);
        ready_to_print = registered_ready;
    }

    if (!ready_to_print) {
        return;
    }

    if (atomic_cas(&tb45_shell_commands_printed, 0, 1)) {
        /* Event-driven startup path for PPP:
         * request IPCP up now, then wait for active/ready before default-route + UP notification. */
        tb45_ppp_probe_reset_runtime_state();
        tb45_startup_ppp_ready_stable_elapsed_ms = 0;
        tb45_startup_ppp_check_internet_reachability_attempt = 0;
        tb45_startup_finalize_stage = 0;
        tb45_startup_ppp_autoup_deadline_ms = k_uptime_get() + TB45_PPP_CHECK_ROUTE_READY_TIMEOUT_MS;
        atomic_set(&tb45_startup_ppp_autoup_armed, 1);
        atomic_set(&tb45_startup_ppp_autoup_done, 0);
        atomic_set(&tb45_startup_finalize_scheduled, 0);
        LOG_DBG("ppp_check_route_ready: running after registration readiness");
        (void)k_work_cancel_delayable(&tb45_startup_finalize_work);
        (void)tb45_schedule_work(&tb45_startup_ppp_autoup_work, K_NO_WAIT);
    }
}

#if !DT_NODE_EXISTS(DT_NODELABEL(tb45_sdwn))
#error "Overlay for tb45_sdwn node not properly defined."
#endif

#if !DT_NODE_EXISTS(DT_NODELABEL(tb45_pwrkey))
#error "Overlay for tb45_pwrkey node not properly defined."
#endif

#if !DT_NODE_EXISTS(DT_NODELABEL(tb45_status))
#error "Overlay for tb45_status node not properly defined."
#endif

static const struct gpio_dt_spec tb45_sdwn_gpio =
	GPIO_DT_SPEC_GET(DT_ALIAS(modem), mdm_wake_gpios);
static const struct gpio_dt_spec tb45_pwrkey_gpio =
	GPIO_DT_SPEC_GET(DT_ALIAS(modem), mdm_power_gpios);
/* tb45_status is currently mapped to PD9 in the project overlay. */
static const struct gpio_dt_spec tb45_status_gpio = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpiod)),
	.pin = 9,
	.dt_flags = GPIO_ACTIVE_HIGH,
};

static void tb45_cellular_startup_init_runtime(const struct device *uart_dev)
{
    const char *uart_name = (uart_dev != NULL) ? uart_dev->name : "<unknown>";
    tb45_runtime_modem_uart_dev = uart_dev;
    const char *shell_menu_status = "deferred shell-command banner not armed";

#if DT_NODE_EXISTS(DT_ALIAS(modem))
    if (!device_is_ready(tb45_cellular_dev)) {
        LOG_WRN("TB45 startup: modem device is not ready (%s)", tb45_cellular_dev->name);
    }
#else
    LOG_WRN("TB45 startup: modem DT alias missing/not-ready, cellular path may not start");
#endif

#if DT_NODE_EXISTS(DT_ALIAS(modem))
    int cb_ret = cellular_set_callback(tb45_cellular_dev,
                                       CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT |
                                       CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED,
                                       tb45_cellular_event_cb, NULL);
    if (cb_ret != 0) {
        LOG_WRN("TB45 startup: failed to register modem comms callback (%d)", cb_ret);
    } else {
        shell_menu_status = "deferred shell-command banner armed";
    }
#endif

    LOG_DBG("TB45 Modem Startup\r\n"
            "  Using Zephyr MODEM_CELLULAR driver on %s\r\n"
            "  Legacy GPIO/AT bootstrap is DISABLED in this mode\r\n"
            "  Modem init/AT sequence is handled by modem_cellular state machine\r\n"
            "  Modem starts automatically via modem_cellular PM/device init\r\n"
            "  Manual AT passthrough: use \x27modem at <command>\x27 only when modem user-pipe is available\r\n"
            "  %s\r\n"
            "  Waiting for modem-ready event before showing shell command menu",
            uart_name, shell_menu_status);
}

int tb45_cellular_init(const struct tb45_cellular_config *cfg)
{
    tb45_cell_store_apn(cfg != NULL ? cfg->apn : NULL);
    tb45_cell_store_field(tb45_cell_username, sizeof(tb45_cell_username),
                          &tb45_cell_username_set, cfg != NULL ? cfg->username : NULL);
    tb45_cell_store_field(tb45_cell_password, sizeof(tb45_cell_password),
                          &tb45_cell_password_set, cfg != NULL ? cfg->password : NULL);
    tb45_cell_store_sim_pin(cfg != NULL ? cfg->sim_pin : NULL);
    tb45_cell_store_carrier_id_or_auto(cfg != NULL ? cfg->carrier_id : NULL);
    tb45_cell_carrier_id_set = true;
    tb45_cell_auth_type = cfg != NULL ? cfg->auth_type : TB45_CELL_AUTH_NONE;
    tb45_periodic_wq = (cfg != NULL && cfg->wq != NULL) ? cfg->wq : &low_priority_wq;

    const struct device *uart_dev = NULL;

    if (tb45_modem_uart_dev != NULL) {
        uart_dev = tb45_modem_uart_dev;
        if (!device_is_ready(tb45_modem_uart_dev)) {
            LOG_WRN("TB45 startup: modem UART device is not ready (%s)", tb45_modem_uart_dev->name);
        }
    }

    tb45_cellular_startup_init_runtime(uart_dev);

    static const struct tb45_ppp_probe_ops probe_ops = {
        .reschedule_work = tb45_reschedule_work,
        .submit_work = tb45_submit_work,
        .restore_ppp_runtime_defaults = tb45_cellular_restore_ppp_runtime_defaults,
        .ppp_iface_runtime_ready = tb45_ppp_iface_runtime_ready,
        .ppp_iface_runtime_defaults_ready = tb45_ppp_iface_runtime_defaults_ready,
        .trigger_restart = tb45_startup_ppp_check_route_ready_restart,
    };

    int ret = tb45_ppp_probe_init(&probe_ops);
    if (ret < 0) {
        LOG_WRN("TB45 periodic: probe init failed (%d)", ret);
    }

    ret = tb45_cellular_probe_set_enabled(true);
    if (ret < 0) {
        LOG_WRN("TB45 periodic: failed to enable probe at init (%d)", ret);
    }

    return 0;
}

#ifdef CONFIG_SHELL
static int tb45_cellular_dev_check(const struct shell *shell)
{
#if DT_NODE_EXISTS(DT_ALIAS(modem))
	if (!device_is_ready(tb45_cellular_dev)) {
        shell_error(shell, "Cellular device not ready: %s", tb45_cellular_dev->name);
        return -ENODEV;
    }
#else
    if (tb45_cellular_dev == NULL) {
        shell_error(shell, "No DT alias 'modem' found");
        return -ENODEV;
    }
#endif

	return 0;
}

static int tb45_prepare_for_raw_shutdown_power_on(const struct shell *shell, const char *context)
{
    int ret;

    if (!gpio_is_ready_dt(&tb45_sdwn_gpio)) {
        shell_error(shell, "%s: TB45 SDWN GPIO not ready", context);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&tb45_sdwn_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        shell_error(shell, "%s: failed to configure TB45 SDWN GPIO (%d)", context, ret);
        return ret;
    }

    ret = gpio_pin_set_dt(&tb45_sdwn_gpio, 1);
    if (ret < 0) {
        shell_error(shell, "%s: failed to drive TB45 SDWN high (%d)", context, ret);
        return ret;
    }

    shell_print(shell, "%s: re-enabling TB45 SDWN before power-on", context);
    return tb45_ppp_restart_sleep_interruptible(shell, TB45_SDWN_REENABLE_DELAY_MS);
}

int tb45_cellular_shell_cmd_modem_on(const struct shell *shell, size_t argc, char **argv)
{
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 modem on");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    bool bringup_required = (sret != 0) ||
        (state != PM_DEVICE_STATE_ACTIVE) ||
        (atomic_get(&tb45_ppp_down_triggered) != 0) ||
        (atomic_get(&tb45_modem_off_triggered) != 0) ||
        (atomic_get(&tb45_raw_shutdown_triggered) != 0) ||
        (atomic_get(&tb45_shell_modem_off_suspend_active) != 0);

    if (!bringup_required) {
        shell_print(shell, "Modem already on");
        return 0;
    }

    /* Reuse modem restart so modem-on follows the same post-power-on
     * bring-up path and relies on automatic PPP bring-up. */
    return cmd_tb45_modem_restart(shell, argc, argv);
}

int tb45_cellular_shell_cmd_modem_off(const struct shell *shell, size_t argc, char **argv)
{
    bool manual_triggered = (argv != NULL);
    int cmd_ret = 0;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 modem off");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    if (manual_triggered) {
        (void)tb45_cellular_shell_interrupt_begin();
    }

    atomic_set(&tb45_modem_off_triggered, 0);
    atomic_set(&tb45_raw_shutdown_triggered, 0);
    atomic_set(&tb45_modem_registration_ready, 0);

    ret = tb45_cellular_probe_set_enabled(false);
    if (ret < 0) {
        shell_error(shell, "Failed to disable probe (%d)", ret);
        cmd_ret = ret;
        goto done;
    }

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
        atomic_set(&tb45_modem_off_triggered, 1);
        shell_print(shell, "Modem already off");
        cmd_ret = 0;
        goto done;
    }

    ret = tb45_shell_modem_off_suspend_start();
    if (ret == -EBUSY) {
        shell_error(shell, "Modem off is already in progress");
        cmd_ret = ret;
        goto done;
    }
    if ((ret != 0) && (ret != -EALREADY)) {
        shell_error(shell, "Modem failed to power off");
        cmd_ret = ret;
        goto done;
    }

    shell_print(shell, "Modem powering off...please wait...");
    ret = tb45_shell_modem_off_suspend_wait(shell);
    if (ret < 0) {
        cmd_ret = ret;
        goto done;
    }
    if ((ret != 0) && (ret != -EALREADY)) {
        shell_error(shell, "Modem failed to power off");
        cmd_ret = ret;
        goto done;
    }

    ret = tb45_wait_for_pm_state(shell, PM_DEVICE_STATE_SUSPENDED,
                                 TB45_WAIT_PM_ACTIVE_TIMEOUT_MS,
                                 "modem power-off completion");
    if (ret < 0) {
        cmd_ret = ret;
        goto done;
    }

    atomic_set(&tb45_modem_off_triggered, 1);
    shell_print(shell, "Modem powered off");
    cmd_ret = 0;

done:
    if (cmd_ret == -ECANCELED) {
        shell_warn(shell, "Modem off interrupted by Ctrl+C; power-off continues in background");
    }

    if (manual_triggered) {
        tb45_cellular_shell_interrupt_end();
    }

    return cmd_ret;
}

int tb45_cellular_shell_cmd_modem_shutdown(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    bool graceful_confirmed = false;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 modem shutdown");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    if (!gpio_is_ready_dt(&tb45_sdwn_gpio)) {
        shell_error(shell, "TB45 SDWN GPIO not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&tb45_pwrkey_gpio)) {
        shell_error(shell, "TB45 PWRKEY GPIO not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&tb45_status_gpio)) {
        shell_error(shell, "TB45 STATUS GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&tb45_sdwn_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        shell_error(shell, "Failed to configure TB45 SDWN GPIO (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&tb45_pwrkey_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        shell_error(shell, "Failed to configure TB45 PWRKEY GPIO (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&tb45_status_gpio, GPIO_INPUT);
    if (ret < 0) {
        shell_error(shell, "Failed to configure TB45 STATUS GPIO (%d)", ret);
        return ret;
    }

    ret = tb45_cellular_probe_set_enabled(false);
    if (ret < 0) {
        shell_error(shell, "Failed to disable probe (%d)", ret);
        return ret;
    }

    atomic_set(&tb45_modem_registration_ready, 0);
    atomic_set(&tb45_modem_off_triggered, 1);
    atomic_set(&tb45_raw_shutdown_triggered, 1);

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    if (iface != NULL) {
        (void)tb45_ppp_ipcp_set_state(iface, false);
    }
    atomic_set(&tb45_ppp_down_triggered, 0);

    ret = gpio_pin_set_dt(&tb45_sdwn_gpio, 1);
    if (ret < 0) {
        shell_error(shell, "Failed to drive TB45 SDWN high (%d)", ret);
        return ret;
    }

    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_SDWN_REENABLE_DELAY_MS);
    if (ret < 0) {
        return ret;
    }

    int status = gpio_pin_get_dt(&tb45_status_gpio);
    if (status < 0) {
        shell_error(shell, "Failed to read TB45 STATUS GPIO (%d)", status);
        return status;
    }

    if (status != 0) {
        int elapsed_ms = 0;

        shell_print(shell, "TB45 graceful shutdown: pulsing PWRKEY and waiting for STATUS LOW...");

        ret = gpio_pin_set_dt(&tb45_pwrkey_gpio, 1);
        if (ret < 0) {
            shell_error(shell, "Failed to drive TB45 PWRKEY high (%d)", ret);
            return ret;
        }

        k_msleep(TB45_PWRKEY_SHUTDOWN_PULSE_MS);

        ret = gpio_pin_set_dt(&tb45_pwrkey_gpio, 0);
        if (ret < 0) {
            shell_error(shell, "Failed to drive TB45 PWRKEY low (%d)", ret);
            return ret;
        }

        while (elapsed_ms <= TB45_STATUS_OFF_TIMEOUT_MS) {
            status = gpio_pin_get_dt(&tb45_status_gpio);
            if (status < 0) {
                shell_error(shell, "Failed to read TB45 STATUS GPIO during shutdown wait (%d)", status);
                return status;
            }

            if (status == 0) {
                graceful_confirmed = true;
                break;
            }

            ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_STEP_WAIT_INTERVAL_MS);
            if (ret < 0) {
                return ret;
            }

            elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
        }

        if (!graceful_confirmed) {
            shell_warn(shell,
                       "TB45 STATUS stayed HIGH for %d ms after shutdown pulse; forcing SDWN low",
                       TB45_STATUS_OFF_TIMEOUT_MS);
        }
    } else {
        graceful_confirmed = true;
        shell_print(shell, "TB45 STATUS already LOW before shutdown pulse");
    }

    ret = gpio_pin_set_dt(&tb45_sdwn_gpio, 0);
    if (ret < 0) {
        shell_error(shell, "Failed to drive TB45 SDWN low (%d)", ret);
        return ret;
    }

    shell_print(shell, "TB45 SDWN LOW: regulators disabled; modem should remain off");
    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_SDWN_HARD_OFF_HOLD_MS);
    if (ret < 0) {
        return ret;
    }

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state != PM_DEVICE_STATE_SUSPENDED)) {
        ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
        if ((ret != 0) && (ret != -EALREADY)) {
            if (!graceful_confirmed) {
                shell_warn(shell, "Graceful shutdown: could not sync PM suspend state (%d)", ret);
            }
        } else {
            ret = tb45_wait_for_pm_state(shell, PM_DEVICE_STATE_SUSPENDED,
                                         TB45_WAIT_PM_ACTIVE_TIMEOUT_MS,
                                         "graceful shutdown PM suspend sync");
            if (ret < 0) {
                if (!graceful_confirmed) {
                    return ret;
                }
            }
        }
    }

    if (graceful_confirmed && (tb45_cellular_dev->pm_base != NULL) &&
        (tb45_cellular_dev->pm_base->state != PM_DEVICE_STATE_SUSPENDED)) {
        tb45_cellular_dev->pm_base->state = PM_DEVICE_STATE_SUSPENDED;
    }

    if (graceful_confirmed) {
        shell_print(shell, "TB45 graceful shutdown confirmed by STATUS LOW");
        shell_warn(shell, "Use tb45 modem on, tb45 ppp restart, or tb45 modem restart for recovery testing");
        return 0;
    }

    shell_warn(shell, "TB45 shutdown forced off without STATUS confirmation");
    shell_warn(shell, "Use tb45 modem on, tb45 ppp restart, or tb45 modem restart for recovery testing");
    return -ETIMEDOUT;
}

static int cmd_tb45_modem_restart(const struct shell *shell, size_t argc, char **argv)
{
    bool manual_triggered = (argv != NULL);
    int cmd_ret = 0;

    /* Intentionally do NOT gate on tb45_check_shell_menu_loaded():
     * this is a recovery command for non-ready startup states.
     */
    if (argc != 1) {
        shell_error(shell, "Usage: tb45 modem restart");
        return -EINVAL;
    }

    if ((tb45_cellular_dev == NULL) || !device_is_ready(tb45_cellular_dev)) {
        shell_error(shell, "Cellular device not ready for modem restart");
        return -ENODEV;
    }

    int ret = tb45_cellular_probe_set_enabled(false);
    if (ret < 0) {
        shell_error(shell, "Failed to disable probe (%d)", ret);
        return ret;
    }

    if (manual_triggered) {
        atomic_set(&tb45_ppp_restart_manual_active, 1);
        atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    }

    if (atomic_get(&tb45_shell_modem_off_suspend_active) != 0) {
        shell_warn(shell, "Modem is currently powering off...please wait...");
        ret = tb45_shell_modem_off_suspend_wait(shell);
        if (ret < 0) {
            cmd_ret = ret;
            goto done;
        }
        if ((ret != 0) && (ret != -EALREADY)) {
            shell_error(shell, "Modem off in progress failed (%d)", ret);
            cmd_ret = ret;
            goto done;
        }
    }

    bool raw_shutdown_marker = (atomic_get(&tb45_raw_shutdown_triggered) != 0);

    atomic_inc(&tb45_restart_count_modem);
    atomic_set(&tb45_modem_off_triggered, 0);
    atomic_set(&tb45_raw_shutdown_triggered, 0);
    atomic_set(&tb45_modem_registration_ready, 0);

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
        atomic_set(&tb45_modem_off_triggered, 1);
        shell_print(shell, "Modem already off");
    } else {
        ret = tb45_modem_restart_suspend_with_retry(shell);
        if (ret == -EALREADY) {
            atomic_set(&tb45_modem_off_triggered, 1);
            shell_print(shell, "Modem already off");
        } else if (ret == 0) {
            atomic_set(&tb45_modem_off_triggered, 1);
            shell_print(shell, "Modem powering off...please wait...");
            ret = tb45_wait_for_pm_state(shell, PM_DEVICE_STATE_SUSPENDED,
                                         TB45_WAIT_PM_ACTIVE_TIMEOUT_MS,
                                         "modem power-off completion");
            if (ret < 0) {
                cmd_ret = ret;
                goto done;
            }
        } else {
            shell_error(shell, "Modem restart failed at power-off (%d)", ret);
            cmd_ret = ret;
            goto done;
        }
    }

    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_PPP_RESTART_STEP_PAUSE_MS);
    if (ret < 0) {
        cmd_ret = ret;
        goto done;
    }

    atomic_set(&tb45_modem_off_triggered, 0);

    if (raw_shutdown_marker) {
        ret = tb45_prepare_for_raw_shutdown_power_on(shell, "Modem restart");
        if (ret < 0) {
            cmd_ret = ret;
            goto done;
        }
    }

    tb45_reset_bringup_runtime_state();

    state = PM_DEVICE_STATE_ACTIVE;
    sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_ACTIVE)) {
        shell_print(shell, "Modem already on");
    } else {
        ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_RESUME);
        if (ret == -EALREADY) {
            shell_print(shell, "Modem already on");
        } else if (ret == 0) {
            shell_print(shell, "Modem powering on...please wait...");
            ret = tb45_wait_for_pm_state(shell, PM_DEVICE_STATE_ACTIVE,
                                         TB45_WAIT_PM_ACTIVE_TIMEOUT_MS,
                                         "modem power-on completion");
            if (ret < 0) {
                cmd_ret = ret;
                goto done;
            }
        } else {
            shell_error(shell, "Modem restart failed at power-on (%d)", ret);
            cmd_ret = ret;
            goto done;
        }
    }

    cmd_ret = 0;

done:
    if (cmd_ret == -ECANCELED) {
        shell_warn(shell, "Modem restart canceled by Ctrl+C");
    }

    if (manual_triggered) {
        atomic_set(&tb45_ppp_restart_manual_active, 0);
        atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    }

    return cmd_ret;
}

int tb45_cellular_shell_cmd_modem_restart(const struct shell *shell, size_t argc, char **argv)
{
    if (tb45_cellular_shell_ppp_consume_down_triggered()) {
        shell_warn(shell, "Modem restart detected prior ppp-down trigger: using PPP recovery path");
        return cmd_tb45_ppp_restart(shell, argc, argv);
    }

    return cmd_tb45_modem_restart(shell, argc, argv);
}

static int tb45_get_ppp_iface(const struct shell *shell, struct net_if **iface_out)
{
    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        shell_error(shell, "PPP is DISABLED (CONFIG_NET_L2_PPP=n)");
        return -ENOTSUP;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    if (iface == NULL) {
        shell_error(shell, "No PPP interface found");
        return -ENODEV;
    }

    *iface_out = iface;
    return 0;
}

int tb45_cellular_shell_ppp_get_iface(const struct shell *shell, struct net_if **iface_out)
{
	return tb45_get_ppp_iface(shell, iface_out);
}

int tb45_cellular_shell_ppp_set_state(struct net_if *iface, bool target_up)
{
	return tb45_ppp_ipcp_set_state(iface, target_up);
}

int tb45_cellular_shell_ppp_wait_link_ready(const struct shell *shell, struct net_if *iface)
{
	return tb45_wait_for_ppp_link_ready(shell, iface, TB45_WAIT_PPP_READY_TIMEOUT_MS);
}

int tb45_cellular_shell_ppp_wait_ipv4_ready(const struct shell *shell, struct net_if *iface)
{
	return tb45_wait_for_ppp_ipv4_ready(shell, iface, TB45_WAIT_PPP_READY_TIMEOUT_MS);
}

int tb45_cellular_shell_interrupt_begin(void)
{
    atomic_set(&tb45_ppp_restart_manual_active, 1);
    atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    return 0;
}

void tb45_cellular_shell_interrupt_end(void)
{
    atomic_set(&tb45_ppp_restart_manual_active, 0);
    atomic_set(&tb45_ppp_restart_cancel_requested, 0);
}

bool tb45_cellular_shell_interrupt_cancel_pending(const struct shell *shell)
{
    return tb45_ppp_restart_cancel_pending(shell);
}

bool tb45_cellular_shell_modem_down_triggered(void)
{
	return atomic_get(&tb45_modem_off_triggered) != 0;
}

bool tb45_cellular_shell_modem_off_in_progress(void)
{
	return atomic_get(&tb45_shell_modem_off_suspend_active) != 0;
}

int tb45_cellular_shell_modem_off_wait(const struct shell *shell)
{
	return tb45_shell_modem_off_suspend_wait(shell);
}

bool tb45_cellular_shell_ppp_consume_down_triggered(void)
{
	return atomic_cas(&tb45_ppp_down_triggered, 1, 0);
}

#endif /* CONFIG_SHELL */

static bool tb45_ppp_iface_has_ipv4_addr(struct net_if *iface)
{
    if ((iface == NULL) || (iface->config.ip.ipv4 == NULL)) {
        return false;
    }

    for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
        const struct in_addr *addr = &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr;
        if (addr->s_addr != 0U) {
            return true;
        }
    }

    return false;
}

static bool tb45_ppp_iface_runtime_ready(struct net_if *iface)
{
    return (iface != NULL) &&
           tb45_ppp_iface_link_ready(iface) &&
           tb45_ppp_iface_has_ipv4_addr(iface);
}

static bool tb45_ppp_iface_runtime_defaults_ready(struct net_if *iface)
{
    return tb45_ppp_iface_runtime_ready(iface) &&
           (net_if_get_default() == iface);
}

#ifdef CONFIG_SHELL
static void tb45_shell_modem_off_suspend_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    tb45_shell_modem_off_suspend_result =
        pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
    if ((tb45_shell_modem_off_suspend_result == 0) ||
        (tb45_shell_modem_off_suspend_result == -EALREADY)) {
        atomic_set(&tb45_modem_off_triggered, 1);
    }

    atomic_set(&tb45_shell_modem_off_suspend_result_ready, 1);
    atomic_set(&tb45_shell_modem_off_suspend_active, 0);
    k_sem_give(&tb45_shell_modem_off_suspend_done);
}

static int tb45_shell_modem_off_suspend_start(void)
{
    while (k_sem_take(&tb45_shell_modem_off_suspend_done, K_NO_WAIT) == 0) {
    }

    if (!atomic_cas(&tb45_shell_modem_off_suspend_active, 0, 1)) {
        return -EBUSY;
    }

    atomic_set(&tb45_shell_modem_off_suspend_result_ready, 0);
    tb45_shell_modem_off_suspend_result = -EINPROGRESS;

    int ret = k_work_submit(&tb45_shell_modem_off_suspend_work);
    if (ret < 0) {
        atomic_set(&tb45_shell_modem_off_suspend_active, 0);
        return ret;
    }

    return 0;
}

static int tb45_shell_modem_off_suspend_wait(const struct shell *shell)
{
    while (atomic_get(&tb45_shell_modem_off_suspend_result_ready) == 0) {
        int ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_STEP_WAIT_INTERVAL_MS);
        if (ret < 0) {
            return ret;
        }
    }

    return tb45_shell_modem_off_suspend_result;
}

static int tb45_modem_suspend_retry_delay_ms(int retry_idx)
{
    switch (retry_idx) {
    case 0:
        return TB45_MODEM_SUSPEND_RETRY_DELAY_1_MS;
    case 1:
        return TB45_MODEM_SUSPEND_RETRY_DELAY_2_MS;
    default:
        return TB45_MODEM_SUSPEND_RETRY_DELAY_3_MS;
    }
}

static int tb45_modem_suspend_with_retry(const struct shell *shell, const char *label)
{
    int ret = 0;

    for (int attempt = 0; attempt <= TB45_MODEM_SUSPEND_RETRY_COUNT; attempt++) {
        if (atomic_get(&tb45_shell_modem_off_suspend_active) != 0) {
            shell_warn(shell, "Modem is currently powering off...please wait...");
            ret = tb45_shell_modem_off_suspend_wait(shell);
            if (ret < 0) {
                return ret;
            }
            if ((ret != 0) && (ret != -EALREADY)) {
                return ret;
            }
            return 0;
        }

        ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
        if ((ret == 0) || (ret == -EALREADY)) {
            return ret;
        }

        if ((ret != -EAGAIN) && (ret != -EBUSY)) {
            return ret;
        }

        enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
        int sret = pm_device_state_get(tb45_cellular_dev, &state);
        if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
            shell_warn(shell,
                       "%s: power-off busy (%d), but PM state is already suspended",
                       label, ret);
            return 0;
        }

        if (attempt >= TB45_MODEM_SUSPEND_RETRY_COUNT) {
            shell_warn(shell,
                       "%s: power-off busy (%d) after %d retries; giving up",
                       label, ret, TB45_MODEM_SUSPEND_RETRY_COUNT);
            return ret;
        }

        int delay_ms = tb45_modem_suspend_retry_delay_ms(attempt);
        shell_warn(shell,
                   "%s: power-off busy (%d); wait %d ms before retry %d/%d",
                   label, ret, delay_ms, attempt + 1, TB45_MODEM_SUSPEND_RETRY_COUNT);
        ret = tb45_ppp_restart_sleep_interruptible(shell, delay_ms);
        if (ret < 0) {
            return ret;
        }
    }

    return ret;
}

static int tb45_modem_restart_suspend_with_retry(const struct shell *shell)
{
    return tb45_modem_suspend_with_retry(shell, "Modem restart");
}

static int tb45_wait_for_pm_state(const struct shell *shell, enum pm_device_state expected_state,
                                  int timeout_ms, const char *step_name)
{
    int elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms) {
        enum pm_device_state current_state = PM_DEVICE_STATE_ACTIVE;
        int ret = pm_device_state_get(tb45_cellular_dev, &current_state);
        if ((ret == 0) && (current_state == expected_state)) {
            return 0;
        }

        ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_STEP_WAIT_INTERVAL_MS);
        if (ret < 0) {
            return ret;
        }

        elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
    }

    shell_error(shell, "PPP recovery timeout waiting for %s", step_name);
    return -ETIMEDOUT;
}

static int tb45_ppp_recovery_sequence(const struct shell *shell)
{
    int ret = 0;

    if (!atomic_cas(&tb45_ppp_recovery_in_progress, 0, 1)) {
        shell_error(shell, "PPP recovery is already in progress");
        return -EBUSY;
    }

    atomic_set(&tb45_ppp_recovery_internal_call, 1);
    shell_print(shell, "PPP recovery sequence:");
    shell_print(shell, "  1) tb45 modem restart");
    ret = cmd_tb45_modem_restart(shell, 1, NULL);
    if (ret < 0) {
        shell_error(shell, "PPP recovery failed at modem restart: %d", ret);
    }

    atomic_set(&tb45_ppp_recovery_internal_call, 0);
    atomic_set(&tb45_ppp_recovery_in_progress, 0);
    return ret;
}

static int cmd_tb45_ppp_restart(const struct shell *shell, size_t argc, char **argv)
{
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 ppp restart");
        return -EINVAL;
    }

    atomic_inc(&tb45_restart_count_ppp);

    bool manual_triggered = (argv != NULL);
    if (manual_triggered) {
        atomic_set(&tb45_ppp_restart_manual_active, 1);
        atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    }

    atomic_set(&tb45_ppp_down_triggered, 0);
    ret = tb45_ppp_recovery_sequence(shell);
    if (ret == -ECANCELED) {
        shell_warn(shell, "PPP restart canceled by Ctrl+C");
    }

    if (manual_triggered) {
        atomic_set(&tb45_ppp_restart_manual_active, 0);
        atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    }

    return ret;
}

int tb45_cellular_shell_cmd_ppp_restart(const struct shell *shell, size_t argc, char **argv)
{
	return cmd_tb45_ppp_restart(shell, argc, argv);
}

#endif /* CONFIG_SHELL */
