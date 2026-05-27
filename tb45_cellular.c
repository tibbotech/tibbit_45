#include "tb45_cellular.h"

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
#include "tb45_sms.h"
#include "tb45_ping.h"

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
static int cmd_tb45_cell_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_cell_resume(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_cell_suspend(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_cell_restart(const struct shell *shell, size_t argc, char **argv);
#endif
int tb45_cellular_probe_set_enabled(bool enabled);
int tb45_cellular_probe_get_enabled(bool *enabled_out);
int tb45_cellular_ppp_ready_post_actions(void);
#ifdef CONFIG_SHELL
static int tb45_ppp_recovery_sequence(const struct shell *shell);
#endif
static bool tb45_ppp_is_healthy_reachable(void);
static void tb45_probe_ping_quiet_progress_cb(const struct tb45_ping_progress *progress,
                                              void *user_data);
#ifdef CONFIG_SHELL
static int tb45_wait_for_shell_menu_loaded(int timeout_ms);
#endif
static int tb45_ppp_ipcp_set_state(struct net_if *iface, bool target_up);
static bool tb45_ppp_iface_link_ready(struct net_if *iface);
static bool tb45_ppp_iface_has_ipv4_addr(struct net_if *iface);
#ifdef CONFIG_SHELL
static int tb45_wait_for_ppp_link_ready(const struct shell *shell, struct net_if *iface,
                                        int timeout_ms);
static int tb45_wait_for_ppp_ipv4_ready(const struct shell *shell, struct net_if *iface,
					int timeout_ms);
#endif
static bool tb45_periodic_ping_probe_with_counters(bool *probe_executed);
static void tb45_startup_ppp_autoup_work_handler(struct k_work *work);
static void tb45_ppp_periodic_arm_work_handler(struct k_work *work);
static void tb45_ppp_periodic_health_work_handler(struct k_work *work);
static void tb45_ppp_periodic_recovery_work_handler(struct k_work *work);
static int tb45_submit_periodic_recovery_work(void);
static void tb45_ppp_periodic_arm_schedule(int delay_ms);
static void tb45_ppp_periodic_health_schedule(int delay_ms);
static void tb45_ppp_periodic_health_stop(void);
static void tb45_startup_ppp_check_route_ready_restart(const char *reason);
int modem_cellular_custom_trigger_ppp_check_route_ready_restart(const struct device *dev);
int modem_cellular_custom_get_cpin_state(const struct device *dev, int *state_out);
int modem_cellular_custom_submit_sim_puk_unlock(const struct device *dev, const char *puk,
                                                const char *new_pin);
int modem_cellular_custom_get_current_network_mode_code(const struct device *dev, int *mode_code);
extern struct k_work_q low_priority_wq;

/*
 * Runtime cellular configuration owned by this library. Populated by
 * tb45_cellular_init() (typically called from main()). The five
 * tb45_main_get_cell* symbols below satisfy the weak-extern contract that the
 * modem_cellular_custom driver still uses to fetch APN/PIN/carrier settings
 * at runtime.
 */
#define TB45_CELL_FIELD_MAX 64
#define TB45_CARRIER_ID_AUTO "AUTO"
static char tb45_cell_apn[TB45_CELL_FIELD_MAX];
static char tb45_cell_username[TB45_CELL_FIELD_MAX];
static char tb45_cell_password[TB45_CELL_FIELD_MAX];
static char tb45_cell_sim_pin[TB45_CELL_FIELD_MAX];
static char tb45_cell_carrier_id[TB45_CELL_FIELD_MAX] = TB45_CARRIER_ID_AUTO;
static bool tb45_cell_apn_set;
static bool tb45_cell_username_set;
static bool tb45_cell_password_set;
static bool tb45_cell_sim_pin_set;

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

const char *tb45_main_get_cellcarrier(void)
{
    return tb45_cell_carrier_id;
}

static void tb45_startup_finalize_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_finalize_work, tb45_startup_finalize_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_ppp_autoup_work, tb45_startup_ppp_autoup_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_ppp_periodic_arm_work, tb45_ppp_periodic_arm_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_ppp_periodic_health_work, tb45_ppp_periodic_health_work_handler);
static atomic_t tb45_shell_commands_printed;
static atomic_t tb45_shell_menu_loaded;
static atomic_t tb45_startup_finalize_scheduled;
static atomic_t tb45_ppp_down_triggered;
static atomic_t tb45_cell_suspend_triggered;
static atomic_t tb45_ppp_recovery_in_progress;
static atomic_t tb45_ppp_recovery_internal_call;
static atomic_t tb45_ppp_restart_manual_active;
static atomic_t tb45_ppp_restart_cancel_requested;
static atomic_t tb45_startup_ppp_autoup_armed;
static atomic_t tb45_startup_ppp_autoup_done;
static atomic_t tb45_ppp_periodic_recovery_running;
static atomic_t tb45_restart_count_ppp;
static atomic_t tb45_restart_count_cell;
static atomic_t tb45_restart_count_full_bringup;
static atomic_t tb45_periodic_probe_pass_count;
static atomic_t tb45_periodic_probe_fail_count;
static atomic_t tb45_periodic_probe_precheck_skip_count;
static atomic_t tb45_periodic_probe_gate_skip_count;
static atomic_t tb45_periodic_probe_enabled = ATOMIC_INIT(0);
static struct k_work_q *tb45_periodic_wq;
static const struct device *tb45_runtime_modem_uart_dev;
static int tb45_startup_ppp_ready_stable_elapsed_ms;
static int tb45_startup_ppp_check_internet_reachability_attempt;
static int tb45_startup_finalize_stage;
static int64_t tb45_startup_ppp_autoup_deadline_ms;
static int tb45_ppp_periodic_consecutive_failures;
static int64_t tb45_ppp_periodic_last_recovery_ms;
static int64_t tb45_ppp_periodic_active_since_ms;
static atomic_t tb45_ppp_periodic_cycle_timeout_ms = ATOMIC_INIT(0);
static atomic_t tb45_ppp_periodic_full_restart_attempts = ATOMIC_INIT(0);

#if defined(CONFIG_APP_TB45_GOOGLE_IPV4)
#define TB45_GOOGLE_IPV4 CONFIG_APP_TB45_GOOGLE_IPV4
#else
#define TB45_GOOGLE_IPV4 "8.8.8.8"
#endif

/*
 * PPP health checks use ICMP ping reachability.
 */
#define TB45_PPP_RESTART_MAX_ATTEMPTS CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#define TB45_PPP_RESTART_RETRY_DELAY_MS 1000
#define TB45_PPP_RESTART_STEP_PAUSE_MS 3000
#define TB45_STEP_WAIT_INTERVAL_MS 100
#define TB45_PPP_READY_STABLE_MS 2500
#define TB45_WAIT_PM_ACTIVE_TIMEOUT_MS 10000
#define TB45_WAIT_PPP_READY_TIMEOUT_MS 30000
#define TB45_PPP_CHECK_ROUTE_READY_TIMEOUT_MS \
    ((CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS > 60000) ? \
     CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS : 60000)
#define TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS 500
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_RETRY_MS 5000
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS 4
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_PROBE_INTERVAL_MS)
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS CONFIG_APP_TB45_PPP_PERIODIC_PROBE_INTERVAL_MS
#else
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS 30000
#endif
#define TB45_PPP_PERIODIC_HEALTH_UNREACHABLE_INTERVAL_MS 5000
#define TB45_PPP_PERIODIC_ARM_RETRY_MS 1000
#define TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS
#define TB45_PPP_PERIODIC_POST_RESTART_GRACE_MS 5000
#define TB45_PPP_PERIODIC_FULL_RESTART_MAX_RETRIES 3
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS
#else
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS 60000
#endif
#define TB45_SHELL_ASCII_CTRL_C 0x03U

#if IS_ENABLED(CONFIG_APP_TB45_PPP_CHECK_WARN_LOGS)
#define TB45_PPP_CHECK_LOG_WRN(...) LOG_WRN(__VA_ARGS__)
#else
#define TB45_PPP_CHECK_LOG_WRN(...) do { } while (0)
#endif

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

static int tb45_reschedule_periodic_work(struct k_work_delayable *dwork, int delay_ms)
{
    return tb45_reschedule_work(dwork, K_MSEC(delay_ms));
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
	if (info == NULL) {
		return -EINVAL;
	}

	info->pass_count = atomic_get(&tb45_periodic_probe_pass_count);
	info->fail_count = atomic_get(&tb45_periodic_probe_fail_count);
	info->precheck_skip_count = atomic_get(&tb45_periodic_probe_precheck_skip_count);
	info->gate_skip_count = atomic_get(&tb45_periodic_probe_gate_skip_count);
	info->periodic_interval_ms = TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS;
	info->active_since_ms = tb45_ppp_periodic_active_since_ms;

	return 0;
}

int tb45_cellular_get_restart_info(struct tb45_cellular_restart_info *info)
{
	if (info == NULL) {
		return -EINVAL;
	}

	info->ppp_restart_count = atomic_get(&tb45_restart_count_ppp);
	info->cell_restart_count = atomic_get(&tb45_restart_count_cell);
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

int tb45_cellular_shell_check_menu_loaded(const struct shell *shell)
{
	return tb45_check_shell_menu_loaded(shell);
}

static int tb45_wait_for_shell_menu_loaded(int timeout_ms)
{
    int elapsed_ms = 0;

    while (elapsed_ms <= timeout_ms) {
        if (atomic_get(&tb45_shell_menu_loaded) != 0) {
            return 0;
        }

        k_msleep(TB45_STEP_WAIT_INTERVAL_MS);
        elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
    }

    return -ETIMEDOUT;
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

static bool tb45_periodic_ping_probe_with_counters(bool *probe_executed)
{
    if (probe_executed != NULL) {
        *probe_executed = true;
    }

    TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe cycle start");
    bool probe_ok = tb45_ppp_is_healthy_reachable();
    if (probe_ok) {
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe cycle PASS");
        atomic_inc(&tb45_periodic_probe_pass_count);
    } else {
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe cycle FAIL");
        atomic_inc(&tb45_periodic_probe_fail_count);
    }

    return probe_ok;
}

static void tb45_ppp_periodic_health_schedule(int delay_ms)
{
    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    if (delay_ms < 0) {
        delay_ms = atomic_get(&tb45_ppp_periodic_cycle_timeout_ms);
        if (delay_ms <= 0) {
            delay_ms = TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS;
        }
    }

    if (delay_ms < 0) {
        delay_ms = 0;
    }

    (void)tb45_reschedule_periodic_work(&tb45_ppp_periodic_health_work, delay_ms);
}

static void tb45_ppp_periodic_arm_schedule(int delay_ms)
{
    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    if (delay_ms < 0) {
        delay_ms = TB45_PPP_PERIODIC_ARM_RETRY_MS;
    }

    (void)tb45_reschedule_periodic_work(&tb45_ppp_periodic_arm_work, delay_ms);
}

static void tb45_ppp_periodic_health_stop(void)
{
    tb45_ppp_periodic_consecutive_failures = 0;
    tb45_ppp_periodic_active_since_ms = 0;
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    (void)k_work_cancel_delayable(&tb45_ppp_periodic_arm_work);
    (void)k_work_cancel_delayable(&tb45_ppp_periodic_health_work);
}

static void tb45_ppp_periodic_arm_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    bool ready = (iface != NULL) &&
                 tb45_ppp_iface_link_ready(iface) &&
                 (net_if_get_default() == iface) &&
                 tb45_ppp_iface_has_ipv4_addr(iface);
    if (!ready) {
        atomic_inc(&tb45_periodic_probe_gate_skip_count);
        tb45_ppp_periodic_arm_schedule(TB45_PPP_PERIODIC_ARM_RETRY_MS);
        return;
    }

    if (tb45_ppp_periodic_active_since_ms <= 0) {
        tb45_ppp_periodic_active_since_ms = k_uptime_get();
    }
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    tb45_ppp_periodic_health_schedule(-1);
}

static void tb45_ppp_periodic_recovery_work_handler(struct k_work *work)
{
    int restart_attempt;

    ARG_UNUSED(work);

    atomic_inc(&tb45_ppp_periodic_full_restart_attempts);
    restart_attempt = atomic_get(&tb45_ppp_periodic_full_restart_attempts);

    if (restart_attempt > TB45_PPP_PERIODIC_FULL_RESTART_MAX_RETRIES) {
        LOG_ERR("TB45 periodic: full modem bring-up failed after %d retries; rebooting system",
                TB45_PPP_PERIODIC_FULL_RESTART_MAX_RETRIES);
        k_msleep(100);
        sys_reboot(SYS_REBOOT_COLD);
        return;
    }

    TB45_PPP_CHECK_LOG_WRN("TB45 periodic: internet unreachable; triggering full modem bring-up (%d/%d)",
                           restart_attempt, TB45_PPP_PERIODIC_FULL_RESTART_MAX_RETRIES);
    atomic_set(&tb45_ppp_periodic_recovery_running, 0);
    tb45_startup_ppp_check_route_ready_restart("periodic health failed");
}

static K_WORK_DEFINE(tb45_ppp_periodic_recovery_work, tb45_ppp_periodic_recovery_work_handler);

static int tb45_submit_periodic_recovery_work(void)
{
    return tb45_submit_work(&tb45_ppp_periodic_recovery_work);
}

static void tb45_ppp_periodic_health_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    bool periodic_probe_ok = tb45_periodic_ping_probe_with_counters(NULL);

    if (periodic_probe_ok) {
        tb45_ppp_periodic_consecutive_failures = 0;
        atomic_set(&tb45_ppp_periodic_full_restart_attempts, 0);
        atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_UNREACHABLE_INTERVAL_MS);
    tb45_ppp_periodic_consecutive_failures++;
    TB45_PPP_CHECK_LOG_WRN("TB45 periodic: internet check failed (%d/%d)",
                           tb45_ppp_periodic_consecutive_failures,
                           TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD);

    if (tb45_ppp_periodic_consecutive_failures < TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD) {
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    int64_t now_ms = k_uptime_get();
    if ((tb45_ppp_periodic_last_recovery_ms != 0) &&
        ((now_ms - tb45_ppp_periodic_last_recovery_ms) < TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)) {
        TB45_PPP_CHECK_LOG_WRN("TB45 periodic: recovery cooldown active; retrying check later");
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    if (!atomic_cas(&tb45_ppp_periodic_recovery_running, 0, 1)) {
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    tb45_ppp_periodic_consecutive_failures = 0;
    tb45_ppp_periodic_last_recovery_ms = now_ms;

    int submit_ret = tb45_submit_periodic_recovery_work();
    if (submit_ret < 0) {
        atomic_set(&tb45_ppp_periodic_recovery_running, 0);
        TB45_PPP_CHECK_LOG_WRN("TB45 periodic: failed to queue recovery work (%d)", submit_ret);
        tb45_ppp_periodic_health_schedule(-1);
    }
}

int tb45_cellular_probe_set_enabled(bool enabled)
{
    if (!enabled) {
        atomic_set(&tb45_periodic_probe_enabled, 0);
        atomic_set(&tb45_ppp_periodic_full_restart_attempts, 0);
        tb45_ppp_periodic_health_stop();
        return 0;
    }

    atomic_set(&tb45_periodic_probe_enabled, 1);

    if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) {
        return 0;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    bool ready = (iface != NULL) &&
                 tb45_ppp_iface_link_ready(iface) &&
                 (net_if_get_default() == iface) &&
                 tb45_ppp_iface_has_ipv4_addr(iface);
    if (!ready) {
        tb45_ppp_periodic_arm_schedule(TB45_PPP_PERIODIC_ARM_RETRY_MS);
        return 0;
    }

    (void)k_work_cancel_delayable(&tb45_ppp_periodic_arm_work);
    if (tb45_ppp_periodic_active_since_ms <= 0) {
        tb45_ppp_periodic_active_since_ms = k_uptime_get();
    }
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    tb45_ppp_periodic_health_schedule(-1);
    return 0;
}

int tb45_cellular_probe_get_enabled(bool *enabled_out)
{
    if (enabled_out == NULL) {
        return -EINVAL;
    }

	*enabled_out = (atomic_get(&tb45_periodic_probe_enabled) != 0);
	return 0;
}

int tb45_cellular_ppp_ready_post_actions(void)
{
	int ret = tb45_sms_receive_trigger_scan();
	if (ret < 0) {
		return ret;
	}

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

static void tb45_startup_ppp_check_route_ready_restart(const char *reason)
{
    atomic_inc(&tb45_restart_count_full_bringup);

    if (reason == NULL) {
        reason = "PPP IPCP link/default route not ready";
    }

    TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_route_ready failed (%s in %d ms)",
                            reason, TB45_PPP_CHECK_ROUTE_READY_TIMEOUT_MS);

    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    atomic_set(&tb45_startup_ppp_autoup_armed, 0);
    atomic_set(&tb45_startup_ppp_autoup_done, 1);
    atomic_set(&tb45_ppp_periodic_recovery_running, 0);
    tb45_ppp_periodic_health_stop();
    tb45_startup_ppp_ready_stable_elapsed_ms = 0;
    tb45_startup_ppp_check_internet_reachability_attempt = 0;
    tb45_startup_finalize_stage = 0;
    atomic_set(&tb45_startup_finalize_scheduled, 0);
    atomic_set(&tb45_shell_commands_printed, 0);
    (void)k_work_cancel_delayable(&tb45_startup_finalize_work);

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

        if (tb45_ppp_iface_link_ready(iface)) {
            tb45_startup_ppp_ready_stable_elapsed_ms += TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS;
        } else {
            tb45_startup_ppp_ready_stable_elapsed_ms = 0;
        }

        ppp_ready_stable = (tb45_startup_ppp_ready_stable_elapsed_ms >= TB45_PPP_READY_STABLE_MS);
        if (ppp_ready_stable) {
            net_if_set_default(iface);
            route_ready = (net_if_get_default() == iface);
            if (!route_ready) {
                LOG_WRN("TB45 startup: PPP active/ready, but default traffic route is not PPP yet");
            } else {
                atomic_set(&tb45_startup_ppp_autoup_done, 1);
                LOG_DBG("ppp_check_route_ready: script success");
                LOG_DBG("TB45 startup: PPP IPCP link active/ready, route=PPP, idx=%d",
                        net_if_get_by_iface(iface));
                ret = tb45_cellular_ppp_ready_post_actions();
                if (ret < 0) {
                    LOG_WRN("TB45 startup: PPP-ready post actions failed (%d)", ret);
                } else {
                    LOG_DBG("TB45 startup: PPP-ready post actions completed");
                }
                if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
                    (void)tb45_schedule_work(&tb45_startup_finalize_work, K_NO_WAIT);
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
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 cell <resume|suspend|restart|unlock_pin>: Cellular controls\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell resume: Resume modem_cellular state machine (if suspended)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell suspend: Suspend modem_cellular state machine (if running)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell restart: Force suspend then resume recovery path\n");
	shell_fprintf(shell, SHELL_NORMAL,
	              "    tb45 cell unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 cell restart)\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN,
                  "  tb45 show <ppp_info|cell_info|summary|network_modes|service_provider|restart_info|probe_info|sms_stat>: Read-only status/info commands\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show ppp_info: Print PPP interface status/IP\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show cell_info: Print modem info via cellular API\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show summary: Print both cell_info and ppp_info\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show network_modes: Print CNMP codes and meanings\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show service_provider: Scan and show service provider list (AT+COPS=?)\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show restart_info: Print restart counters since boot\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show probe_info: Print periodic internet probe counters\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE,
                  "  tb45 ppp <up|down|default_traffic_route>: PPP controls\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp up: Bring PPP interface up\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp down: Bring PPP interface down\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 probe <on|off>: Probe controls\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 probe on: Enable periodic health probe scheduling\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 probe off: Disable periodic health probe scheduling\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 sms send <phone> <text>: Send SMS message\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "#########################################################\n\n");
}

void tb45_cellular_shell_print_available_commands_menu(const struct shell *shell)
{
	tb45_print_available_commands_menu(shell);
}
#endif


static void tb45_startup_finalize_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (tb45_startup_finalize_stage >= 2) {
        return;
    }

    if (tb45_startup_finalize_stage == 0) {
        tb45_startup_finalize_stage = 1;
        tb45_startup_ppp_check_internet_reachability_attempt = 0;
        LOG_DBG("ppp_check_internet_reachability: checking...please wait");
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
                LOG_INF("  tb45 cell <resume|suspend|restart|unlock_pin>: Cellular controls");
                LOG_INF("    tb45 cell resume: Resume modem_cellular state machine (if suspended)");
                LOG_INF("    tb45 cell suspend: Suspend modem_cellular state machine (if running)");
                LOG_INF("    tb45 cell restart: Force suspend then resume recovery path");
                LOG_INF("    tb45 cell unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 cell restart)");
                LOG_INF("  tb45 show <ppp_info|cell_info|summary|network_modes|service_provider|restart_info|probe_info|sms_stat>: Read-only status/info commands");
                LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
                LOG_INF("    tb45 show cell_info: Print modem info via cellular API");
                LOG_INF("    tb45 show summary: Print both cell_info and ppp_info");
                LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
                LOG_INF("    tb45 show service_provider: Scan and show service provider list (AT+COPS=?)");
                LOG_INF("    tb45 show restart_info: Print restart counters since boot");
                LOG_INF("    tb45 show probe_info: Print periodic internet probe counters");
                LOG_INF("    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)");
                LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
                LOG_INF("    tb45 ppp up: Bring PPP interface up");
                LOG_INF("    tb45 ppp down: Bring PPP interface down");
                LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
                LOG_INF("  tb45 probe <on|off>: Probe controls");
                LOG_INF("    tb45 probe on: Enable periodic health probe scheduling");
                LOG_INF("    tb45 probe off: Disable periodic health probe scheduling");
                LOG_INF("  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host");
                LOG_INF("  tb45 sms send <phone> <text>: Send SMS message");

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
            LOG_INF("  tb45 cell <resume|suspend|restart|unlock_pin>: Cellular controls");
            LOG_INF("    tb45 cell resume: Resume modem_cellular state machine (if suspended)");
            LOG_INF("    tb45 cell suspend: Suspend modem_cellular state machine (if running)");
            LOG_INF("    tb45 cell restart: Force suspend then resume recovery path");
            LOG_INF("    tb45 cell unlock_pin <pukcode> <pincode>: SIM PUK unlock (then run tb45 cell restart)");
            LOG_INF("  tb45 show <ppp_info|cell_info|summary|network_modes|service_provider|restart_info|probe_info|sms_stat>: Read-only status/info commands");
            LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
            LOG_INF("    tb45 show cell_info: Print modem info via cellular API");
            LOG_INF("    tb45 show summary: Print both cell_info and ppp_info");
            LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
            LOG_INF("    tb45 show service_provider: Scan and show service provider list (AT+COPS=?)");
            LOG_INF("    tb45 show restart_info: Print restart counters since boot");
            LOG_INF("    tb45 show probe_info: Print periodic internet probe counters");
            LOG_INF("    tb45 show sms_stat [all]: Print SMS health summary (add all for full counters)");
            LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
            LOG_INF("    tb45 ppp up: Bring PPP interface up");
            LOG_INF("    tb45 ppp down: Bring PPP interface down");
            LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
            LOG_INF("  tb45 probe <on|off>: Probe controls");
            LOG_INF("    tb45 probe on: Enable periodic health probe scheduling");
            LOG_INF("    tb45 probe off: Disable periodic health probe scheduling");
            LOG_INF("  tb45 net ping <host-or-ipv4> [count] [timeout_ms] [payload_bytes]: Ping host");
            LOG_INF("  tb45 sms send <phone> <text>: Send SMS message");
            LOG_INF("#########################################################");
            LOG_INF("");
#endif
        }

        atomic_set(&tb45_shell_menu_loaded, 1);
        tb45_startup_finalize_stage = 2;
        tb45_ppp_periodic_consecutive_failures = 0;
        if (ppp_check_internet_reachability_done &&
            (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) &&
            (atomic_get(&tb45_periodic_probe_enabled) != 0)) {
            tb45_ppp_periodic_active_since_ms = k_uptime_get();
            atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
            LOG_DBG("TB45 periodic: health check armed (interval=%d ms)",
                    TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
            tb45_ppp_periodic_health_schedule(-1);
        } else if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) &&
                   (atomic_get(&tb45_periodic_probe_enabled) != 0)) {
            LOG_WRN("TB45 periodic: health check not armed because startup internet check did not succeed");
            
        }
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
        ready_to_print = (reg != NULL) &&
                         ((reg->status == CELLULAR_REGISTRATION_REGISTERED_HOME) ||
                          (reg->status == CELLULAR_REGISTRATION_REGISTERED_ROAMING));
    }

    if (!ready_to_print) {
        return;
    }

    if (atomic_cas(&tb45_shell_commands_printed, 0, 1)) {
        /* Event-driven startup path for PPP:
         * request IPCP up now, then wait for active/ready before default-route + UP notification. */
        tb45_ppp_periodic_health_stop();
        atomic_set(&tb45_ppp_periodic_recovery_running, 0);
        tb45_ppp_periodic_consecutive_failures = 0;
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


static void tb45_cellular_startup_init_runtime(const struct device *uart_dev)
{
    const char *uart_name = (uart_dev != NULL) ? uart_dev->name : "<unknown>";
    tb45_runtime_modem_uart_dev = uart_dev;

    LOG_DBG("TB45 startup: using Zephyr MODEM_CELLULAR driver on %s", uart_name);
    LOG_DBG("TB45 startup: legacy GPIO/AT bootstrap is DISABLED in this mode");

#if DT_NODE_EXISTS(DT_ALIAS(modem))
    if (!device_is_ready(tb45_cellular_dev)) {
        LOG_WRN("TB45 startup: modem device is not ready (%s)", tb45_cellular_dev->name);
    }
#else
    LOG_WRN("TB45 startup: modem DT alias missing/not-ready, cellular path may not start");
#endif

    LOG_DBG("TB45 modem init/AT sequence is handled by modem_cellular state machine");
    LOG_DBG("TB45 modem starts automatically via modem_cellular PM/device init");

    LOG_DBG("TB45 manual AT passthrough: use 'modem at <command>' only when modem user-pipe is available");

#if DT_NODE_EXISTS(DT_ALIAS(modem))
    int cb_ret = cellular_set_callback(tb45_cellular_dev,
                                       CELLULAR_EVENT_MODEM_COMMS_CHECK_RESULT |
                                       CELLULAR_EVENT_REGISTRATION_STATUS_CHANGED,
                                       tb45_cellular_event_cb, NULL);
    if (cb_ret != 0) {
        LOG_WRN("TB45 startup: failed to register modem comms callback (%d)", cb_ret);
    } else {
        LOG_DBG("TB45 startup: deferred shell-command banner armed");
    }
#endif

    LOG_DBG("TB45 startup: waiting for modem-ready event before showing shell command menu");
}

int tb45_cellular_init(const struct tb45_cellular_config *cfg)
{
    if (cfg != NULL) {
        tb45_cell_store_field(tb45_cell_apn, sizeof(tb45_cell_apn),
                              &tb45_cell_apn_set, cfg->apn);
        tb45_cell_store_field(tb45_cell_username, sizeof(tb45_cell_username),
                              &tb45_cell_username_set, cfg->username);
        tb45_cell_store_field(tb45_cell_password, sizeof(tb45_cell_password),
                              &tb45_cell_password_set, cfg->password);
        tb45_cell_store_field(tb45_cell_sim_pin, sizeof(tb45_cell_sim_pin),
                              &tb45_cell_sim_pin_set, cfg->sim_pin);
        tb45_cell_store_carrier_id_or_auto(cfg->carrier_id);
        tb45_periodic_wq = (cfg->wq != NULL) ? cfg->wq : &low_priority_wq;
    } else {
        tb45_cell_store_carrier_id_or_auto(NULL);
        tb45_periodic_wq = &low_priority_wq;
    }

    const struct device *uart_dev = NULL;

    if (tb45_modem_uart_dev != NULL) {
        uart_dev = tb45_modem_uart_dev;
        if (!device_is_ready(tb45_modem_uart_dev)) {
            LOG_WRN("TB45 startup: modem UART device is not ready (%s)", tb45_modem_uart_dev->name);
        }
    }

    tb45_cellular_startup_init_runtime(uart_dev);

    int ret = tb45_cellular_probe_set_enabled(true);
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

int tb45_cellular_shell_cmd_cell_resume(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 cell resume");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    if (atomic_cas(&tb45_cell_suspend_triggered, 1, 0)) {
        shell_warn(shell, "Cell resume detected prior cell suspend trigger: rebooting system");
        k_msleep(100);
        sys_reboot(SYS_REBOOT_COLD);
        return 0;
    }

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_ACTIVE)) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell Already resumed");
        ret = tb45_cellular_ppp_ready_post_actions();
        if (ret < 0) {
            shell_error(shell, "Failed PPP-ready post actions (%d)", ret);
            return ret;
        }
        return 0;
    }

    ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_RESUME);
    if (ret == -EALREADY) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell Already resumed");
        ret = tb45_cellular_ppp_ready_post_actions();
        if (ret < 0) {
            shell_error(shell, "Failed PPP-ready post actions (%d)", ret);
            return ret;
        }
        return 0;
    }

    if (ret == 0) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell RESUMED...please wait...");
        ret = tb45_cellular_ppp_ready_post_actions();
        if (ret < 0) {
            shell_error(shell, "Failed PPP-ready post actions (%d)", ret);
            return ret;
        }
        return 0;
    }

    shell_error(shell, "Cell FAILED to resume");
    return ret;
}

int tb45_cellular_shell_cmd_cell_suspend(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 cell suspend");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    atomic_set(&tb45_cell_suspend_triggered, 0);

    ret = tb45_cellular_probe_set_enabled(false);
    if (ret < 0) {
        shell_error(shell, "Failed to disable probe (%d)", ret);
        return ret;
    }

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
        atomic_set(&tb45_cell_suspend_triggered, 1);
        shell_print(shell, "Cell Already suspended");
        return 0;
    }

    ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
    if (ret == -EALREADY) {
        atomic_set(&tb45_cell_suspend_triggered, 1);
        shell_print(shell, "Cell Already suspended");
        return 0;
    }

    if (ret == 0) {
        atomic_set(&tb45_cell_suspend_triggered, 1);
        shell_print(shell, "Cell SUSPENDED...please wait...");
        return 0;
    }

    shell_error(shell, "Cell FAILED to suspend");
    return ret;
}

static int cmd_tb45_cell_restart(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;

    /* Intentionally do NOT gate on tb45_check_shell_menu_loaded():
     * this is a recovery command for non-ready startup states.
     */
    if (argc != 1) {
        shell_error(shell, "Usage: tb45 cell restart");
        return -EINVAL;
    }

    atomic_set(&tb45_cell_suspend_triggered, 0);
    shell_warn(shell, "Cell restart policy: rebooting system unconditionally");
    k_msleep(100);
    sys_reboot(SYS_REBOOT_COLD);
    return 0;
}

int tb45_cellular_shell_cmd_cell_restart(const struct shell *shell, size_t argc, char **argv)
{
    return cmd_tb45_cell_restart(shell, argc, argv);
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

static void tb45_probe_ping_quiet_progress_cb(const struct tb45_ping_progress *progress,
                                              void *user_data)
{
    ARG_UNUSED(progress);
    ARG_UNUSED(user_data);
}

static bool tb45_ppp_is_healthy_reachable(void)
{
    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        return false;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    if (iface == NULL) {
        return false;
    }

    bool iface_up = net_if_is_up(iface);
    bool iface_dormant = net_if_is_dormant(iface);
    bool carrier_ok = net_if_is_carrier_ok(iface);
    bool default_route_is_ppp = (net_if_get_default() == iface);
    bool has_ipv4_addr = tb45_ppp_iface_has_ipv4_addr(iface);
    bool precheck_ok = iface_up && !iface_dormant && carrier_ok && default_route_is_ppp && has_ipv4_addr;
    if (!precheck_ok) {
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: precheck failed up=%d dormant=%d carrier=%d route=%d ipv4=%d",
                                   iface_up, iface_dormant, carrier_ok, default_route_is_ppp, has_ipv4_addr);
        return false;
    }

    TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: precheck passed; running ping probe to %s", TB45_GOOGLE_IPV4);
    /* Match app_queue_ppp_ping_test request semantics exactly:
     * count=0 and timeout=0 resolve to async ping defaults in tb45_ping_prepare_request().
     */
    int ret = tb45_ping_run_ex(TB45_GOOGLE_IPV4, 0U, 0U,
                              TB45_PING_DEFAULT_PAYLOAD_SIZE,
                              tb45_probe_ping_quiet_progress_cb, NULL);
    if (ret < 0) {
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe failed (%d)", ret);
        return false;
    }

    TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe PASS");
    return true;
}

#ifdef CONFIG_SHELL

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
    int sequence_ret = 0;
    struct net_if *iface = NULL;

    if (!atomic_cas(&tb45_ppp_recovery_in_progress, 0, 1)) {
        shell_error(shell, "PPP recovery is already in progress");
        return -EBUSY;
    }

    atomic_set(&tb45_ppp_recovery_internal_call, 1);

    shell_print(shell, "PPP recovery sequence:");

    /* Restart Cell (suspend + resume) */
    shell_print(shell, "  1) tb45 cell restart");
    ret = cmd_tb45_cell_restart(shell, 1, NULL);
    if (ret < 0) {
        shell_error(shell, "PPP recovery failed at step 1 (cell restart): %d", ret);
        sequence_ret = ret;
        goto show_info;
    }
    ret = tb45_wait_for_pm_state(shell, PM_DEVICE_STATE_ACTIVE,
                                 TB45_WAIT_PM_ACTIVE_TIMEOUT_MS, "cell restart completion");
    if (ret < 0) {
        sequence_ret = ret;
        goto show_info;
    }
    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_PPP_RESTART_STEP_PAUSE_MS);
    if (ret < 0) {
        sequence_ret = ret;
        goto cleanup;
    }

    /* Bring PPP interface UP */
    shell_print(shell, "  2) tb45 ppp up");
    ret = tb45_get_ppp_iface(shell, &iface);
    if (ret < 0) {
        shell_error(shell, "PPP recovery failed at step 2 (get PPP iface): %d", ret);
        sequence_ret = ret;
        goto show_info;
    }

    ret = tb45_ppp_ipcp_set_state(iface, true);
    if ((ret != 0) && (ret != -EALREADY)) {
        shell_error(shell, "PPP recovery failed at step 2 (net_if_up): %d", ret);
        sequence_ret = ret;
        goto show_info;
    }

    ret = tb45_wait_for_ppp_link_ready(shell, iface, TB45_WAIT_PPP_READY_TIMEOUT_MS);
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            shell_error(shell, "PPP recovery timeout waiting for PPP_LINK_STATE active/ready");
        }
        sequence_ret = ret;
        goto show_info;
    }
    shell_print(shell, "PPP link is active/ready (idx=%d)", net_if_get_by_iface(iface));

    /* Set traffic to go through PPP interface as part of step 3 behavior. */
    net_if_set_default(iface);
    if (net_if_get_default() != iface) {
        shell_error(shell, "PPP recovery failed at step 3 (set default route)");
        sequence_ret = -EIO;
        goto show_info;
    }
    shell_print(shell, "Default traffic route set to PPP (idx=%d)", net_if_get_by_iface(iface));

    ret = tb45_wait_for_ppp_ipv4_ready(shell, iface, TB45_WAIT_PPP_READY_TIMEOUT_MS);
    if (ret < 0) {
        if (ret == -ETIMEDOUT) {
            shell_error(shell, "PPP recovery timeout waiting for IPCP IPv4 readiness");
        }
        sequence_ret = ret;
        goto show_info;
    }

    ret = tb45_cellular_ppp_ready_post_actions();
    if (ret < 0) {
        shell_error(shell, "PPP recovery failed at step 2 (PPP-ready post actions): %d", ret);
        sequence_ret = ret;
        goto show_info;
    }

    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_PPP_RESTART_STEP_PAUSE_MS);
    if (ret < 0) {
        sequence_ret = ret;
        goto cleanup;
    }

show_info:
    if (sequence_ret != -ECANCELED) {
        shell_print(shell, "  3) tb45 show ppp_info");
        atomic_set(&tb45_ppp_down_triggered, 0);
#if defined(CONFIG_SHELL) && defined(CONFIG_SHELL_BACKEND_SERIAL) && \
	defined(CONFIG_SHELL_LOG_BACKEND) && defined(CONFIG_UART_CONSOLE)
        ret = tb45_shell_cmd_ppp_info(shell, 1, NULL);
#else
        ret = 0;
#endif
    }

cleanup:
    atomic_set(&tb45_ppp_recovery_internal_call, 0);
    atomic_set(&tb45_ppp_recovery_in_progress, 0);
    if (sequence_ret < 0) {
        return sequence_ret;
    }
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

    ret = tb45_cellular_probe_set_enabled(false);
    if (ret < 0) {
        shell_error(shell, "Failed to disable probe (%d)", ret);
        return ret;
    }

    atomic_inc(&tb45_restart_count_ppp);

    bool manual_triggered = (argv != NULL);
    if (manual_triggered) {
        atomic_set(&tb45_ppp_restart_manual_active, 1);
        atomic_set(&tb45_ppp_restart_cancel_requested, 0);
    }

    int last_ret = -EIO;

    /* If MAX_ATTEMPTS is 0, we treat it as infinite */
    bool infinite = (TB45_PPP_RESTART_MAX_ATTEMPTS == 0);
    
    for (int attempt = 1; infinite || (attempt <= TB45_PPP_RESTART_MAX_ATTEMPTS); attempt++) {
        if (tb45_ppp_restart_cancel_pending(shell)) {
            last_ret = -ECANCELED;
            break;
        }

        if (infinite) {
            shell_print(shell, "PPP restart attempt %d (infinite)", attempt);
        } else {
            shell_print(shell, "PPP restart attempt %d/%d", attempt, TB45_PPP_RESTART_MAX_ATTEMPTS);
        }
        
        atomic_set(&tb45_ppp_down_triggered, 0);

        int loop_ret = tb45_ppp_recovery_sequence(shell);
        last_ret = loop_ret;
        if (loop_ret == -ECANCELED) {
            break;
        }

        if ((loop_ret == 0) && tb45_ppp_is_healthy_reachable()) {
            ret = 0;
            goto done;
        }

        if (infinite || (attempt < TB45_PPP_RESTART_MAX_ATTEMPTS)) {
            shell_warn(shell, "PPP still not healthy/reachable, retrying...");
            int wait_ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_PPP_RESTART_RETRY_DELAY_MS);
            if (wait_ret < 0) {
                last_ret = wait_ret;
                break;
            }
        }
    }

    if (last_ret == -ECANCELED) {
        shell_warn(shell, "PPP restart canceled by Ctrl+C");
        ret = -ECANCELED;
        goto done;
    }

    shell_error(shell, "PPP restart failed after %d attempts", TB45_PPP_RESTART_MAX_ATTEMPTS);
    ret = (last_ret == 0) ? -ETIMEDOUT : last_ret;

done:
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
