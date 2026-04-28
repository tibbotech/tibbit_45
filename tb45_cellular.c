#include "tb45_cellular.h"

#include <ctype.h>
#include <errno.h>
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
#include <zephyr/net/icmp.h>
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
#include "tb45_sms.h"

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

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 3, 0)
#define TB45_NET_ICMP_INIT_CTX(ctx, handler) \
    net_icmp_init_ctx((ctx), NET_AF_INET, NET_ICMPV4_ECHO_REPLY, 0, (handler))
#else
#define TB45_NET_ICMP_INIT_CTX(ctx, handler) \
    net_icmp_init_ctx((ctx), NET_ICMPV4_ECHO_REPLY, 0, (handler))
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

static int cmd_tb45_ppp_restart(const struct shell *shell, size_t argc, char **argv);
static int cmd_tb45_ppp_info(const struct shell *shell, size_t argc, char **argv);
static int cmd_tb45_restart_info(const struct shell *shell, size_t argc, char **argv);
static int cmd_tb45_probe_info(const struct shell *shell, size_t argc, char **argv);
static int cmd_tb45_cell_restart(const struct shell *shell, size_t argc, char **argv);
static int tb45_ppp_recovery_sequence(const struct shell *shell);
static bool tb45_ppp_is_healthy_reachable(void);
static int tb45_wait_for_shell_menu_loaded(int timeout_ms);
static int tb45_ppp_ipcp_set_state(struct net_if *iface, bool target_up);
static bool tb45_ppp_iface_link_ready(struct net_if *iface);
static bool tb45_ppp_iface_has_ipv4_addr(struct net_if *iface);
static int tb45_wait_for_ppp_link_ready(const struct shell *shell, struct net_if *iface,
                                        int timeout_ms);
static bool tb45_startup_ppp_check_internet_reachability_probe_with_http_fallback(void);
static bool tb45_periodic_http_probe_with_fallback(void);
static bool tb45_periodic_ping_probe_with_counters(bool *probe_executed);
static void tb45_startup_ppp_autoup_work_handler(struct k_work *work);
static void tb45_ppp_periodic_health_work_handler(struct k_work *work);
static void tb45_ppp_periodic_recovery_thread_entry(void *arg1, void *arg2, void *arg3);
static void tb45_ppp_periodic_health_schedule(int delay_ms);
static void tb45_ppp_periodic_health_stop(void);
static void tb45_startup_ppp_check_route_ready_restart(const char *reason);
int modem_cellular_custom_trigger_ppp_check_route_ready_restart(const struct device *dev);
int modem_cellular_custom_submit_sim_puk_unlock(const struct device *dev, const char *puk,
                                                const char *new_pin);
int modem_cellular_custom_get_current_network_mode_code(const struct device *dev, int *mode_code);
extern const char *tb45_main_get_cellapn(void) __attribute__((weak));

static void tb45_startup_finalize_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_finalize_work, tb45_startup_finalize_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_startup_ppp_autoup_work, tb45_startup_ppp_autoup_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_ppp_periodic_health_work, tb45_ppp_periodic_health_work_handler);
static atomic_t tb45_shell_commands_printed;
static atomic_t tb45_shell_menu_loaded;
static atomic_t tb45_startup_finalize_scheduled;
static atomic_t tb45_ppp_down_triggered;
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
static const struct device *tb45_runtime_modem_uart_dev;
static int tb45_current_network_mode_code = 2; /* AUTO default */
static int tb45_startup_ppp_ready_stable_elapsed_ms;
static int tb45_startup_ppp_check_internet_reachability_attempt;
static int tb45_startup_finalize_stage;
static int64_t tb45_startup_ppp_autoup_deadline_ms;
static int tb45_ppp_periodic_consecutive_failures;
static int64_t tb45_ppp_periodic_last_recovery_ms;
static int64_t tb45_ppp_periodic_active_since_ms;
K_THREAD_STACK_DEFINE(tb45_ppp_periodic_recovery_stack, 3072);
static struct k_thread tb45_ppp_periodic_recovery_thread;

#if defined(CONFIG_APP_TB45_PING_COUNT_DEFAULT)
#define PING_COUNT_DEFAULT CONFIG_APP_TB45_PING_COUNT_DEFAULT
#else
#define PING_COUNT_DEFAULT 4
#endif

#if defined(CONFIG_APP_TB45_GOOGLE_DNS_IPV4)
#define TB45_GOOGLE_DNS_IPV4 CONFIG_APP_TB45_GOOGLE_DNS_IPV4
#else
#define TB45_GOOGLE_DNS_IPV4 "8.8.8.8"
#endif

/* Task 12 phase-1: keep periodic HTTP probe endpoints as file-local constants. */
#define TB45_PERIODIC_HTTP_PROBE_PORT "80"
#define TB45_PERIODIC_HTTP_CONNECT_TIMEOUT_MS 3000

#define TB45_PERIODIC_HTTP_HOST_PRIMARY "connectivitycheck.gstatic.com"
#define TB45_PERIODIC_HTTP_PATH_PRIMARY "/generate_204"
#define TB45_PERIODIC_HTTP_EXPECTED_STATUS_PRIMARY 204

#define TB45_PERIODIC_HTTP_HOST_SECONDARY "msftconnecttest.com"
#define TB45_PERIODIC_HTTP_PATH_SECONDARY "/connecttest.txt"
#define TB45_PERIODIC_HTTP_EXPECTED_STATUS_SECONDARY 200

#define TB45_PERIODIC_HTTP_HOST_TERTIARY "captive.apple.com"
#define TB45_PERIODIC_HTTP_PATH_TERTIARY "/hotspot-detect.html"
#define TB45_PERIODIC_HTTP_EXPECTED_STATUS_TERTIARY 200

struct tb45_periodic_http_target {
    const char *host;
    const char *path;
    int expected_status;
};

static const struct tb45_periodic_http_target tb45_periodic_http_targets[] = {
    {
        .host = TB45_PERIODIC_HTTP_HOST_PRIMARY,
        .path = TB45_PERIODIC_HTTP_PATH_PRIMARY,
        .expected_status = TB45_PERIODIC_HTTP_EXPECTED_STATUS_PRIMARY,
    },
    {
        .host = TB45_PERIODIC_HTTP_HOST_SECONDARY,
        .path = TB45_PERIODIC_HTTP_PATH_SECONDARY,
        .expected_status = TB45_PERIODIC_HTTP_EXPECTED_STATUS_SECONDARY,
    },
    {
        .host = TB45_PERIODIC_HTTP_HOST_TERTIARY,
        .path = TB45_PERIODIC_HTTP_PATH_TERTIARY,
        .expected_status = TB45_PERIODIC_HTTP_EXPECTED_STATUS_TERTIARY,
    },
};

static const char *tb45_get_runtime_cellapn(void)
{
    if (tb45_main_get_cellapn == NULL) {
        return NULL;
    }

    return tb45_main_get_cellapn();
}

static void tb45_log_startup_apn_hint(void)
{
    const char *apn = tb45_get_runtime_cellapn();

    if (apn == NULL) {
        LOG_WRN("TB45 startup hint: CELLAPN is not present in app/src/main.cpp");
        return;
    }

    if (apn[0] == '\0') {
        LOG_WRN("TB45 startup hint: CELLAPN is present but empty");
        return;
    }

    LOG_ERR("TB45 startup hint: verify CELLAPN is correct for your SIM/operator");
    LOG_ERR("TB45 startup hint: current CELLAPN=\"%s\"", apn);

}
/*
 * ICMP timing defaults used by `tb45 net ping`.
 * PPP health checks use HTTP request fallback reachability.
 */
#define TB45_INTERNET_PING_TIMEOUT_MS 2000
#define TB45_INTERNET_PING_RETRIES PING_COUNT_DEFAULT
#define TB45_PPP_RESTART_MAX_ATTEMPTS CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#define TB45_PPP_RESTART_RETRY_DELAY_MS 1000
#define TB45_CELL_RESTART_GAP_MS 3000
#define TB45_PPP_RESTART_STEP_PAUSE_MS 3000
#define TB45_STEP_WAIT_INTERVAL_MS 100
#define TB45_PPP_READY_STABLE_MS 2500
#define TB45_WAIT_PM_SUSPENDED_TIMEOUT_MS 5000
#define TB45_WAIT_PM_ACTIVE_TIMEOUT_MS 10000
#define TB45_WAIT_PPP_READY_TIMEOUT_MS 30000
#define TB45_PPP_CHECK_ROUTE_READY_TIMEOUT_MS CONFIG_MODEM_CELLULAR_PERIODIC_SCRIPT_MS
#define TB45_CELL_RESTART_SHOW_INFO_WAIT_MS 30000
#define TB45_CELL_RESTART_WAIT_PPP_READY_MS 30000
#define TB45_STARTUP_PPP_AUTOCHECK_INTERVAL_MS 500
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_RETRY_MS 5000
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS 4
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS)
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS CONFIG_APP_TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS
#else
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS 30000
#endif
#define TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS
#define TB45_PPP_PERIODIC_POST_RESTART_GRACE_MS 5000
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS
#else
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS 60000
#endif
#if defined(CONFIG_APP_TB45_PPP_CHECK_INTERNET_REACHABILITY_DNS_HOST)
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_DNS_HOST CONFIG_APP_TB45_PPP_CHECK_INTERNET_REACHABILITY_DNS_HOST
#else
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_DNS_HOST "google.com"
#endif

#if defined(CONFIG_APP_TB45_PPP_CHECK_INTERNET_REACHABILITY_TCP_PORT_STR)
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_TCP_PORT_STR CONFIG_APP_TB45_PPP_CHECK_INTERNET_REACHABILITY_TCP_PORT_STR
#else
#define TB45_PPP_CHECK_INTERNET_REACHABILITY_TCP_PORT_STR "443"
#endif
#define TB45_SHELL_ASCII_CTRL_C 0x03U
#define TB45_SIM_PUK_LEN        8U
#define TB45_SIM_PIN_MIN_LEN    4U
#define TB45_SIM_PIN_MAX_LEN    8U

#if IS_ENABLED(CONFIG_APP_TB45_PPP_CHECK_WARN_LOGS)
#define TB45_PPP_CHECK_LOG_WRN(...) LOG_WRN(__VA_ARGS__)
#else
#define TB45_PPP_CHECK_LOG_WRN(...) do { } while (0)
#endif

#if IS_ENABLED(CONFIG_MODEM_LOG_LEVEL_DBG)
#define TB45_PERIODIC_HTTP_LOG_DBG(...) LOG_DBG(__VA_ARGS__)
#else
#define TB45_PERIODIC_HTTP_LOG_DBG(...) do { } while (0)
#endif

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

    TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP probe cycle start");
    bool probe_ok = tb45_ppp_is_healthy_reachable();
    if (probe_ok) {
        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP probe cycle PASS");
        atomic_inc(&tb45_periodic_probe_pass_count);
    } else {
        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP probe cycle FAIL");
        atomic_inc(&tb45_periodic_probe_fail_count);
    }

    return probe_ok;
}

static void tb45_ppp_periodic_health_schedule(int delay_ms)
{
    if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) {
        return;
    }

    if (delay_ms < 0) {
        delay_ms = 0;
    }

    (void)k_work_reschedule(&tb45_ppp_periodic_health_work, K_MSEC(delay_ms));
}

static void tb45_ppp_periodic_health_stop(void)
{
    tb45_ppp_periodic_consecutive_failures = 0;
    tb45_ppp_periodic_active_since_ms = 0;
    (void)k_work_cancel_delayable(&tb45_ppp_periodic_health_work);
}

static void tb45_ppp_periodic_recovery_thread_entry(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    TB45_PPP_CHECK_LOG_WRN("TB45 periodic: internet unreachable; triggering full modem bring-up");
    atomic_set(&tb45_ppp_periodic_recovery_running, 0);
    tb45_startup_ppp_check_route_ready_restart("periodic health failed");
}

static void tb45_ppp_periodic_health_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) {
        return;
    }

    bool periodic_probe_ok = tb45_periodic_ping_probe_with_counters(NULL);

    if (periodic_probe_ok) {
        tb45_ppp_periodic_consecutive_failures = 0;
        tb45_ppp_periodic_health_schedule(TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        return;
    }

    tb45_ppp_periodic_consecutive_failures++;
    TB45_PPP_CHECK_LOG_WRN("TB45 periodic: internet check failed (%d/%d)",
                           tb45_ppp_periodic_consecutive_failures,
                           TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD);

    if (tb45_ppp_periodic_consecutive_failures < TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD) {
        tb45_ppp_periodic_health_schedule(TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        return;
    }

    int64_t now_ms = k_uptime_get();
    if ((tb45_ppp_periodic_last_recovery_ms != 0) &&
        ((now_ms - tb45_ppp_periodic_last_recovery_ms) < TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)) {
        TB45_PPP_CHECK_LOG_WRN("TB45 periodic: recovery cooldown active; retrying check later");
        tb45_ppp_periodic_health_schedule(TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        return;
    }

    if (!atomic_cas(&tb45_ppp_periodic_recovery_running, 0, 1)) {
        tb45_ppp_periodic_health_schedule(TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        return;
    }

    tb45_ppp_periodic_consecutive_failures = 0;
    tb45_ppp_periodic_last_recovery_ms = now_ms;

    (void)k_thread_create(&tb45_ppp_periodic_recovery_thread,
                          tb45_ppp_periodic_recovery_stack,
                          K_THREAD_STACK_SIZEOF(tb45_ppp_periodic_recovery_stack),
                          tb45_ppp_periodic_recovery_thread_entry, NULL, NULL, NULL,
                          K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&tb45_ppp_periodic_recovery_thread, "tb45_ppp_periodic_recovery");
}

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

static int tb45_wait_for_ppp_ready_after_cell_restart(void)
{
    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        return -ENOTSUP;
    }

    int elapsed_ms = 0;
    int ready_stable_elapsed_ms = 0;

    while (elapsed_ms <= TB45_CELL_RESTART_WAIT_PPP_READY_MS) {
        struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
        if (tb45_ppp_iface_link_ready(iface)) {
            ready_stable_elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
            if (ready_stable_elapsed_ms >= TB45_PPP_READY_STABLE_MS) {
                return 0;
            }
        } else {
            ready_stable_elapsed_ms = 0;
        }

        k_msleep(TB45_STEP_WAIT_INTERVAL_MS);
        elapsed_ms += TB45_STEP_WAIT_INTERVAL_MS;
    }

    return -ETIMEDOUT;
}

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
        LOG_ERR("TB45 startup: ppp_check_route_ready restart skipped (modem device not ready)");
        if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
            (void)k_work_schedule(&tb45_startup_finalize_work, K_NO_WAIT);
        }
        return;
    }

    int ret = modem_cellular_custom_trigger_ppp_check_route_ready_restart(tb45_cellular_dev);
    if (ret < 0) {
        LOG_ERR("TB45 startup: ppp_check_route_ready restart trigger failed (%d)", ret);
        if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
            (void)k_work_schedule(&tb45_startup_finalize_work, K_NO_WAIT);
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
            (void)k_work_schedule(&tb45_startup_finalize_work, K_NO_WAIT);
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
                if (atomic_cas(&tb45_startup_finalize_scheduled, 0, 1)) {
                    (void)k_work_schedule(&tb45_startup_finalize_work, K_NO_WAIT);
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

    (void)k_work_reschedule(&tb45_startup_ppp_autoup_work,
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
                  "  modem at [command] [expected_response]: Send AT command\n");
    shell_fprintf(shell, SHELL_NORMAL, "    EXAMPLES:\n");
    shell_fprintf(shell, SHELL_NORMAL, "      modem at AT+CSQ OK\n");
    shell_fprintf(shell, SHELL_NORMAL, "      modem at AT+CGDATA=\"PPP\",1 CONNECT\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  modem_stats buffer: Get buffer statistics\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "  tb45 cell <resume|suspend|restart|puk|unlock_pin>: Cellular controls\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell resume: Resume modem_cellular state machine (if suspended)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell suspend: Suspend modem_cellular state machine (if running)\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell restart: Force suspend then resume recovery path\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell puk <pukcode> <pincode>: SIM PUK unlock and restart\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 cell unlock_pin <pukcode> <pincode>: Alias for SIM PUK unlock and restart\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN,
                  "  tb45 show <ppp_info|cell_info|summary|network_modes|restart_info|probe_info>: Read-only status/info commands\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show ppp_info: Print PPP interface status/IP\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show cell_info: Print modem info via cellular API\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show summary: Print both cell_info and ppp_info\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show network_modes: Print CNMP codes and meanings\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show restart_info: Print restart counters since boot\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 show probe_info: Print periodic HTTP probe counters\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE,
                  "  tb45 ppp <up|down|default_traffic_route>: PPP controls\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp up: Bring PPP interface up\n");
    shell_fprintf(shell, SHELL_NORMAL, "    tb45 ppp down: Bring PPP interface down\n");
    shell_fprintf(shell, SHELL_NORMAL,
                  "    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "  tb45 net ping <ipv4-address> [count]: Ping IPv4 host\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "#########################################################\n\n");
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
        LOG_DBG("ppp_check_internet_reachability: running before available commands pane");
        (void)k_work_reschedule(&tb45_startup_finalize_work, K_NO_WAIT);
        return;
    }

    if (tb45_startup_finalize_stage == 1) {
        bool ppp_check_internet_reachability_done = false;

        tb45_startup_ppp_check_internet_reachability_attempt++;

        if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
            TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_internet_reachability skipped (CONFIG_NET_L2_PPP=n)");
            ppp_check_internet_reachability_done = true;
        } else {
            struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
            if (iface == NULL) {
                TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_internet_reachability pending (no PPP interface yet, attempt %d/%d)",
                                        tb45_startup_ppp_check_internet_reachability_attempt, TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS);
            } else {
                bool precheck_ok = tb45_ppp_iface_link_ready(iface) &&
                                   (net_if_get_default() == iface) &&
                                   tb45_ppp_iface_has_ipv4_addr(iface);

                if (!precheck_ok) {
                    TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_internet_reachability pending (PPP link/route/IPv4 not ready yet, attempt %d/%d)",
                                            tb45_startup_ppp_check_internet_reachability_attempt, TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS);
                } else if (!tb45_startup_ppp_check_internet_reachability_probe_with_http_fallback()) {
                    TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_internet_reachability pending (internet not reachable yet, attempt %d/%d)",
                                            tb45_startup_ppp_check_internet_reachability_attempt, TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS);
                } else {
                    LOG_DBG("ppp_check_internet_reachability: script success");
                    LOG_DBG("TB45 startup: Internet Reachable via PPP, idx=%d",
                            net_if_get_by_iface(iface));
                    ppp_check_internet_reachability_done = true;
                }
            }
        }

        if (!ppp_check_internet_reachability_done &&
            (tb45_startup_ppp_check_internet_reachability_attempt < TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS)) {
            (void)k_work_reschedule(&tb45_startup_finalize_work,
                                    K_MSEC(TB45_PPP_CHECK_INTERNET_REACHABILITY_RETRY_MS));
            return;
        }

        if (!ppp_check_internet_reachability_done) {
            TB45_PPP_CHECK_LOG_WRN("TB45 startup: ppp_check_internet_reachability failed after %d attempts",
                                    TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS);
            tb45_log_startup_apn_hint();
            TB45_PPP_CHECK_LOG_WRN("TB45 startup: triggering full modem bring-up restart after internet reachability failure");
            tb45_startup_ppp_check_route_ready_restart("startup internet check failed");
            return;
        }

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
                LOG_INF("  modem at [command] [expected_response]: Send AT command");
                LOG_INF("    EXAMPLES:");
                LOG_INF("      modem at AT+CSQ OK");
                LOG_INF("      modem at AT+CGDATA=\"PPP\",1 CONNECT");
                LOG_INF("  modem_stats buffer: Get buffer statistics");
                LOG_INF("  tb45 cell <resume|suspend|restart|puk|unlock_pin>: Cellular controls");
                LOG_INF("    tb45 cell resume: Resume modem_cellular state machine (if suspended)");
                LOG_INF("    tb45 cell suspend: Suspend modem_cellular state machine (if running)");
                LOG_INF("    tb45 cell restart: Force suspend then resume recovery path");
                LOG_INF("    tb45 cell puk <pukcode> <pincode>: SIM PUK unlock and restart");
                LOG_INF("    tb45 cell unlock_pin <pukcode> <pincode>: Alias for SIM PUK unlock and restart");
                LOG_INF("  tb45 show <ppp_info|cell_info|summary|network_modes|restart_info|probe_info>: Read-only status/info commands");
                LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
                LOG_INF("    tb45 show cell_info: Print modem info via cellular API");
                LOG_INF("    tb45 show summary: Print both cell_info and ppp_info");
                LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
                LOG_INF("    tb45 show restart_info: Print restart counters since boot");
                LOG_INF("    tb45 show probe_info: Print periodic HTTP probe counters");
                LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
                LOG_INF("    tb45 ppp up: Bring PPP interface up");
                LOG_INF("    tb45 ppp down: Bring PPP interface down");
                LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
                LOG_INF("  tb45 net ping <ipv4-address> [count]: Ping IPv4 host");
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
            LOG_INF("  modem at [command] [expected_response]: Send AT command");
            LOG_INF("    EXAMPLES:");
            LOG_INF("      modem at AT+CSQ OK");
            LOG_INF("      modem at AT+CGDATA=\"PPP\",1 CONNECT");
            LOG_INF("  modem_stats buffer: Get buffer statistics");
            LOG_INF("  tb45 cell <resume|suspend|restart|puk|unlock_pin>: Cellular controls");
            LOG_INF("    tb45 cell resume: Resume modem_cellular state machine (if suspended)");
            LOG_INF("    tb45 cell suspend: Suspend modem_cellular state machine (if running)");
            LOG_INF("    tb45 cell restart: Force suspend then resume recovery path");
            LOG_INF("    tb45 cell puk <pukcode> <pincode>: SIM PUK unlock and restart");
            LOG_INF("    tb45 cell unlock_pin <pukcode> <pincode>: Alias for SIM PUK unlock and restart");
            LOG_INF("  tb45 show <ppp_info|cell_info|summary|network_modes|restart_info|probe_info>: Read-only status/info commands");
            LOG_INF("    tb45 show ppp_info: Print PPP interface status/IP");
            LOG_INF("    tb45 show cell_info: Print modem info via cellular API");
            LOG_INF("    tb45 show summary: Print both cell_info and ppp_info");
            LOG_INF("    tb45 show network_modes: Print CNMP codes and meanings");
            LOG_INF("    tb45 show restart_info: Print restart counters since boot");
            LOG_INF("    tb45 show probe_info: Print periodic HTTP probe counters");
            LOG_INF("  tb45 ppp <up|down|default_traffic_route>: PPP controls");
            LOG_INF("    tb45 ppp up: Bring PPP interface up");
            LOG_INF("    tb45 ppp down: Bring PPP interface down");
            LOG_INF("    tb45 ppp default_traffic_route <on|off>: Route default traffic via PPP or Ethernet");
            LOG_INF("  tb45 net ping <ipv4-address> [count]: Ping IPv4 host");
            LOG_INF("#########################################################");
            LOG_INF("");
#endif
        }

        atomic_set(&tb45_shell_menu_loaded, 1);
        tb45_startup_finalize_stage = 2;
        tb45_ppp_periodic_consecutive_failures = 0;
        if (ppp_check_internet_reachability_done &&
            (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0)) {
            tb45_ppp_periodic_active_since_ms = k_uptime_get();
            LOG_DBG("TB45 periodic: health check armed (interval=%d ms)",
                    TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
            tb45_ppp_periodic_health_schedule(TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        } else if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) {
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
        (void)k_work_schedule(&tb45_startup_ppp_autoup_work, K_NO_WAIT);
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

static int tb45_cellular_startup_init(void)
{
    const struct device *uart_dev = NULL;

    if (tb45_modem_uart_dev != NULL) {
        uart_dev = tb45_modem_uart_dev;
        if (!device_is_ready(tb45_modem_uart_dev)) {
            LOG_WRN("TB45 startup: modem UART device is not ready (%s)", tb45_modem_uart_dev->name);
        }
    }

    tb45_cellular_startup_init_runtime(uart_dev);
    return 0;
}

SYS_INIT(tb45_cellular_startup_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

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

static bool tb45_value_is_numeric(const char *value, size_t min_len, size_t max_len)
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

static void tb45_cell_unlock_pin_print_usage(const struct shell *shell, char **argv, int argv_index,
					     const char *command)
{
	if ((argv_index >= 0) && (argv != NULL) && (argv[argv_index] != NULL)) {
		shell_fprintf(shell, SHELL_NORMAL, "Invalid input: ");
		shell_fprintf(shell, SHELL_VT100_COLOR_RED, " %s\n", argv[argv_index]);
	}

	if (command == NULL) {
		command = "unlock_pin";
	}

	shell_fprintf(shell, SHELL_NORMAL, "Usage: tb45 cell %s <", command);
	shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "pukcode");
	shell_fprintf(shell, SHELL_NORMAL, "> <");
	shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "pincode");
	shell_fprintf(shell, SHELL_NORMAL, ">\n");

	shell_fprintf(shell, SHELL_NORMAL, "Example: tb45 cell %s ", command);
	shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "12345678");
	shell_fprintf(shell, SHELL_NORMAL, " ");
	shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "0000\n");
}

static int tb45_cell_submit_puk_unlock(const struct shell *shell, const char *puk,
				       const char *new_pin)
{
	int ret;

	if (!tb45_value_is_numeric(puk, TB45_SIM_PUK_LEN, TB45_SIM_PUK_LEN)) {
		shell_error(shell, "PUK must be exactly 8 numeric digits");
		return -EINVAL;
	}

	if (!tb45_value_is_numeric(new_pin, TB45_SIM_PIN_MIN_LEN, TB45_SIM_PIN_MAX_LEN)) {
		shell_error(shell, "New PIN must be 4-8 numeric digits");
		return -EINVAL;
	}

	ret = modem_cellular_custom_submit_sim_puk_unlock(tb45_cellular_dev, puk, new_pin);
	if (ret < 0) {
		shell_error(shell, "Failed to queue PUK unlock command (%d)", ret);
		return ret;
	}

	shell_print(shell, "PUK unlock command queued; restarting modem init sequence");
	return cmd_tb45_cell_restart(shell, 1, NULL);
}

static int cmd_tb45_cell_puk(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	/* Intentionally do NOT gate on tb45_check_shell_menu_loaded():
	 * this is a recovery command for non-ready startup states.
	 */
	if (argc != 3) {
		tb45_cell_unlock_pin_print_usage(shell, argv, -1, "puk");
		return -EINVAL;
	}

	ret = tb45_cellular_dev_check(shell);
	if (ret < 0) {
		return ret;
	}

	if (!tb45_value_is_numeric(argv[1], TB45_SIM_PUK_LEN, TB45_SIM_PUK_LEN)) {
		tb45_cell_unlock_pin_print_usage(shell, argv, 1, "puk");
		return -EINVAL;
	}

	if (!tb45_value_is_numeric(argv[2], TB45_SIM_PIN_MIN_LEN, TB45_SIM_PIN_MAX_LEN)) {
		tb45_cell_unlock_pin_print_usage(shell, argv, 2, "puk");
		return -EINVAL;
	}

	return tb45_cell_submit_puk_unlock(shell, argv[1], argv[2]);
}

static int cmd_tb45_cell_unlock_pin(const struct shell *shell, size_t argc, char **argv)
{
	int ret;

	/* Intentionally do NOT gate on tb45_check_shell_menu_loaded():
	 * this is a recovery command for non-ready startup states.
	 */
	if (argc != 3) {
		tb45_cell_unlock_pin_print_usage(shell, argv, -1, "unlock_pin");
		return -EINVAL;
	}

	ret = tb45_cellular_dev_check(shell);
	if (ret < 0) {
		return ret;
	}

	if (!tb45_value_is_numeric(argv[1], TB45_SIM_PUK_LEN, TB45_SIM_PUK_LEN)) {
		tb45_cell_unlock_pin_print_usage(shell, argv, 1, "unlock_pin");
		return -EINVAL;
	}

	if (!tb45_value_is_numeric(argv[2], TB45_SIM_PIN_MIN_LEN, TB45_SIM_PIN_MAX_LEN)) {
		tb45_cell_unlock_pin_print_usage(shell, argv, 2, "unlock_pin");
		return -EINVAL;
	}

	return tb45_cell_submit_puk_unlock(shell, argv[1], argv[2]);
}

static int cmd_tb45_cell_resume(const struct shell *shell, size_t argc, char **argv)
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

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_ACTIVE)) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell Already resumed");
        return 0;
    }

    ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_RESUME);
    if (ret == -EALREADY) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell Already resumed");
        return 0;
    }

    if (ret == 0) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell SUCCESSFULLY resumed");
        return 0;
    }

    shell_error(shell, "Cell FAILED to resume");
    return ret;
}

static int cmd_tb45_cell_suspend(const struct shell *shell, size_t argc, char **argv)
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

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
        shell_print(shell, "Cell Already suspended");
        return 0;
    }

    ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
    if (ret == -EALREADY) {
        shell_print(shell, "Cell Already suspended");
        return 0;
    }

    if (ret == 0) {
        shell_print(shell, "Cell SUCCESSFULLY suspended");
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

    int ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    atomic_inc(&tb45_restart_count_cell);

    if (tb45_ppp_restart_cancel_pending(shell)) {
        return -ECANCELED;
    }

    int suspend_ret = 0;
    int resume_ret = 0;

    enum pm_device_state state = PM_DEVICE_STATE_ACTIVE;
    int sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_SUSPENDED)) {
        shell_print(shell, "Cell Already suspended");
    } else {
        ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_SUSPEND);
        if (ret == 0 || ret == -EALREADY) {
            shell_print(shell, "Cell SUCCESSFULLY suspended");
        } else {
            shell_error(shell, "Cell FAILED to suspend");
            suspend_ret = ret;
        }
    }

    /* Pause between suspend and resume actions. */
    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_CELL_RESTART_GAP_MS);
    if (ret < 0) {
        return ret;
    }

    if (tb45_ppp_restart_cancel_pending(shell)) {
        return -ECANCELED;
    }

    sret = pm_device_state_get(tb45_cellular_dev, &state);
    if ((sret == 0) && (state == PM_DEVICE_STATE_ACTIVE)) {
        atomic_set(&tb45_ppp_down_triggered, 0);
        shell_print(shell, "Cell Already resumed");
    } else {
        ret = pm_device_action_run(tb45_cellular_dev, PM_DEVICE_ACTION_RESUME);
        if (ret == 0 || ret == -EALREADY) {
            atomic_set(&tb45_ppp_down_triggered, 0);
            shell_print(shell, "Cell SUCCESSFULLY resumed");
        } else {
            shell_error(shell, "Cell FAILED to resume");
            resume_ret = ret;
        }
    }

    if (suspend_ret != 0) {
        return suspend_ret;
    }

    /* For direct 'tb45 cell restart', print PPP status once shell/menu is ready.
     * Skip this when restart is invoked from PPP recovery to avoid duplicate output.
     */
    if (atomic_get(&tb45_ppp_recovery_internal_call) == 0) {
        if (tb45_wait_for_shell_menu_loaded(TB45_CELL_RESTART_SHOW_INFO_WAIT_MS) == 0) {
            (void)tb45_wait_for_ppp_ready_after_cell_restart();
            shell_print(shell, "  tb45 show ppp_info");
            (void)cmd_tb45_ppp_info(shell, 1, NULL);
        } else {
            shell_warn(shell, "PPP info skipped: shell/menu not ready yet");
        }
    }

    return resume_ret;
}

static int tb45_cell_info_print_info(const struct shell *shell, const char *label,
                                     enum cellular_modem_info_type info, bool *use_blue)
{
    char buf[96] = {0};
    int ret = cellular_get_modem_info(tb45_cellular_dev, info, buf, sizeof(buf));
    enum shell_vt100_color label_color = *use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN;

    shell_fprintf(shell, label_color, "%s:", label);
    if (ret == 0) {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %s\n", buf);
        *use_blue = !(*use_blue);
        return 0;
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", ret);
    *use_blue = !(*use_blue);
    return ret;
}

static void tb45_cell_info_print_on_off(const struct shell *shell, const char *label, bool is_on,
                                        bool *use_blue)
{
    enum shell_vt100_color label_color = *use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN;
    enum shell_vt100_color value_color = is_on ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED;

    shell_fprintf(shell, label_color, "%s:", label);
    shell_fprintf(shell, value_color, " %s\n", is_on ? "On" : "Off");
    *use_blue = !(*use_blue);
}

static int cmd_tb45_cell_info(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show cell_info");
        return -EINVAL;
    }

    ret = tb45_cellular_dev_check(shell);
    if (ret < 0) {
        return ret;
    }

    int overall_ret = 0;
    bool use_blue = true;

    enum pm_device_state pm_state = PM_DEVICE_STATE_ACTIVE;
    int pm_ret = pm_device_state_get(tb45_cellular_dev, &pm_state);
    bool pm_on = (pm_ret == 0) && (pm_state == PM_DEVICE_STATE_ACTIVE);

    
    shell_fprintf(shell, SHELL_VT100_COLOR_MAGENTA, "\nCELL_INFO:");

    shell_fprintf(shell, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "\n");
    
    ret = tb45_cell_info_print_info(shell, "IMEI", CELLULAR_MODEM_INFO_IMEI, &use_blue);
    if (ret != 0) {
        overall_ret = ret;
    }

    ret = tb45_cell_info_print_info(shell, "MODEL", CELLULAR_MODEM_INFO_MODEL_ID, &use_blue);
    if (ret != 0) {
        overall_ret = ret;
    }

    ret = tb45_cell_info_print_info(shell, "MFR", CELLULAR_MODEM_INFO_MANUFACTURER, &use_blue);
    if (ret != 0) {
        overall_ret = ret;
    }

    ret = tb45_cell_info_print_info(shell, "FW", CELLULAR_MODEM_INFO_FW_VERSION, &use_blue);
    if (ret != 0) {
        overall_ret = ret;
    }

    enum cellular_registration_status reg = CELLULAR_REGISTRATION_NOT_REGISTERED;
    ret = cellular_get_registration_status(tb45_cellular_dev,
                                           CELLULAR_ACCESS_TECHNOLOGY_LTE,
                                           &reg);
    shell_fprintf(shell, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "REG_STATUS:");
    if (ret == 0) {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", (int)reg);
    } else {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", ret);
        overall_ret = ret;
    }
    use_blue = !use_blue;

    tb45_cell_info_print_on_off(shell, "PPP", pm_on, &use_blue);
    tb45_cell_info_print_on_off(shell, "Power-Management", pm_on, &use_blue);
    if (pm_ret != 0) {
        shell_fprintf(shell, use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN, "PM_STATE:");
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " <err %d>\n", pm_ret);
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "\n");

    return overall_ret;
}

static int cmd_tb45_system_reboot(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "\n********************\n");
    shell_fprintf(shell, SHELL_NORMAL, "*");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, " REBOOTING DEVICE ");
    shell_fprintf(shell, SHELL_NORMAL, "*\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "********************\n");

	/* Give the shell backend time to flush before hard reset */
	k_msleep(1000);

	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

static int cmd_tb45_help(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 help");
        return -EINVAL;
    }

    tb45_print_available_commands_menu(shell);
    return 0;
}

static int cmd_tb45_ppp_info(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show ppp_info");
        return -EINVAL;
    }

    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        shell_error(shell, "PPP is DISABLED (CONFIG_NET_L2_PPP=n)");
        return -ENOTSUP;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    if (iface == NULL) {
        shell_error(shell, "No PPP interface found");
        return -ENODEV;
    }
    bool iface_up = net_if_is_up(iface);
    bool default_route_is_ppp = (net_if_get_default() == iface);

    bool use_blue = true;
#define TB45_PPP_LABEL_COLOR() (use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN)
#define TB45_PPP_TOGGLE_LABEL_COLOR() do { use_blue = !use_blue; } while (0)

    shell_fprintf(shell, SHELL_VT100_COLOR_MAGENTA, "\nPPP_INFO:\n");
    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "PPP_IFACE_INDEX:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", net_if_get_by_iface(iface));
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "PPP_IFACE_UP:");
    shell_fprintf(shell, iface_up ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED, " %s\n",
                  iface_up ? "yes" : "no");
    TB45_PPP_TOGGLE_LABEL_COLOR();

    bool iface_dormant = net_if_is_dormant(iface);
    bool carrier_ok = net_if_is_carrier_ok(iface);
    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "PPP_CARRIER_OK:");
    shell_fprintf(shell, carrier_ok ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED, " %s\n",
                  carrier_ok ? "yes" : "no");
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "PPP_LINK_STATE:");
    if (!iface_up) {
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " down\n");
    } else if (iface_dormant) {
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " paused/sleeping\n");
    } else if (carrier_ok) {
        shell_fprintf(shell, SHELL_VT100_COLOR_GREEN, " active/ready\n");
    } else {
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " up/no-carrier\n");
    }
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "PPP_DEFAULT_TRAFFIC_ROUTE:");
    shell_fprintf(shell, default_route_is_ppp ? SHELL_VT100_COLOR_GREEN : SHELL_VT100_COLOR_RED, " %s\n",
                  default_route_is_ppp ? "yes" : "no");
    TB45_PPP_TOGGLE_LABEL_COLOR();

    if (!iface_up) {
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, "**IPCP_STATE**: stale (iface down)\n");
    }

    bool has_ipv4_addr = false;

    if (iface->config.ip.ipv4 == NULL) {
        shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR:");
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " <none>\n");
        TB45_PPP_TOGGLE_LABEL_COLOR();
    } else {
        char buf[NET_IPV4_ADDR_LEN];

        for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
            const struct in_addr *addr =
                &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr;

            if (addr->s_addr == 0U) {
                continue;
            }

            shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR[%d]:", i);
            shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %s%s\n",
                          net_addr_ntop(AF_INET, addr, buf, sizeof(buf)),
                          iface_up ? "" : " (stale)");
            TB45_PPP_TOGGLE_LABEL_COLOR();
            has_ipv4_addr = true;
        }

        if (!has_ipv4_addr) {
            shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "IPCP_IPV4_ADDR:");
            shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " <none>\n");
            TB45_PPP_TOGGLE_LABEL_COLOR();
        }

        shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "IPCP_GW:");
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %s%s\n",
                      net_addr_ntop(AF_INET, &iface->config.ip.ipv4->gw, buf, sizeof(buf)),
                      iface_up ? "" : " (stale)");
        TB45_PPP_TOGGLE_LABEL_COLOR();
    }

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "INTERNET:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " See 'tb45 show probe_info'\n");
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "BAUDRATE:");
    uint32_t baudrate = 0U;
    ret = tb45_get_runtime_baudrate(&baudrate);
    if (ret == 0) {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %u\n", (unsigned int)baudrate);
    } else {
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " <err %d>\n", ret);
    }
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "NETWORK_MODE:");
    int network_mode_code = tb45_current_network_mode_code;
    ret = tb45_get_runtime_network_mode(&network_mode_code);
    if (ret == 0) {
        tb45_current_network_mode_code = network_mode_code;
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d (%s)\n",
                      network_mode_code, tb45_cnmp_mode_to_str(network_mode_code));
    } else {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d (%s) [cached, err %d]\n",
                      tb45_current_network_mode_code,
                      tb45_cnmp_mode_to_str(tb45_current_network_mode_code), ret);
    }
    TB45_PPP_TOGGLE_LABEL_COLOR();

    shell_fprintf(shell, TB45_PPP_LABEL_COLOR(), "\n");
#undef TB45_PPP_LABEL_COLOR
#undef TB45_PPP_TOGGLE_LABEL_COLOR
    return 0;
}

static int cmd_tb45_show_summary(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show summary");
        return -EINVAL;
    }

    int cell_ret = cmd_tb45_cell_info(shell, 1, NULL);
    int ppp_ret = cmd_tb45_ppp_info(shell, 1, NULL);

    if (cell_ret != 0) {
        return cell_ret;
    }
    return ppp_ret;
}

static int cmd_tb45_restart_info(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show restart_info");
        return -EINVAL;
    }

    int ppp_restarts = atomic_get(&tb45_restart_count_ppp);
    int cell_restarts = atomic_get(&tb45_restart_count_cell);
    int full_bringup_restarts = atomic_get(&tb45_restart_count_full_bringup);
    int total = ppp_restarts + cell_restarts + full_bringup_restarts;

    shell_fprintf(shell, SHELL_VT100_COLOR_MAGENTA, "\nRESTART_INFO:\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "PPP_RESTART_COUNT:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", ppp_restarts);
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "CELL_RESTART_COUNT:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", cell_restarts);
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "FULL_BRINGUP_RESTART_COUNT:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", full_bringup_restarts);
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "TOTAL_RESTART_EVENTS:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n\n", total);

    return 0;
}

static int cmd_tb45_probe_info(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show probe_info");
        return -EINVAL;
    }

    int pass = atomic_get(&tb45_periodic_probe_pass_count);
    int fail = atomic_get(&tb45_periodic_probe_fail_count);
    int precheck_skip = atomic_get(&tb45_periodic_probe_precheck_skip_count);
    int gate_skip = atomic_get(&tb45_periodic_probe_gate_skip_count);
    int total_executed = pass + fail;
    int total_skipped = precheck_skip + gate_skip;
    int64_t active_time_ms = 0;
    int64_t active_time_s = 0;

    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) && (tb45_ppp_periodic_active_since_ms > 0)) {
        active_time_ms = k_uptime_get() - tb45_ppp_periodic_active_since_ms;
        if (active_time_ms < 0) {
            active_time_ms = 0;
        }
        active_time_s = active_time_ms / 1000;
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_MAGENTA, "\nINET_ISALIVE_PROBE_INFO:\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "PERIODIC_INTERVAL_MS:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "ACTIVE_TIME:");
    if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " disabled\n");
    } else if (tb45_ppp_periodic_active_since_ms <= 0) {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " not active\n");
    } else {
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %llds (%lld ms)\n",
                      (long long)active_time_s, (long long)active_time_ms);
    }
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "PASS_COUNT:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", pass);
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "FAIL_COUNT:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", fail);
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "TOTAL_EXECUTED:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", total_executed);
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "TOTAL_SKIPPED:");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, " %d\n", total_skipped);
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT,
                  "NOTE: pass/fail counts are only when periodic HTTP probe actually runs.\n\n");

    return 0;
}

struct tb45_network_mode_item {
    int code;
    const char *meaning;
};

static const struct tb45_network_mode_item tb45_network_modes[] = {
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

static int cmd_tb45_show_network_modes(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 show network_modes");
        return -EINVAL;
    }

    bool use_blue = true;
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "CNMP Code  Meaning\n");
    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "---------  ----------------------------------------------\n");

    for (size_t i = 0; i < ARRAY_SIZE(tb45_network_modes); i++) {
        enum shell_vt100_color code_color = use_blue ? SHELL_VT100_COLOR_BLUE : SHELL_VT100_COLOR_CYAN;
        shell_fprintf(shell, code_color, "%-9d", tb45_network_modes[i].code);
        shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "  %s\n", tb45_network_modes[i].meaning);
        use_blue = !use_blue;
    }

    shell_fprintf(shell, SHELL_VT100_COLOR_DEFAULT, "\n");
    return 0;
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

static bool tb45_http_send_all(int sock, const char *data, size_t len)
{
    if ((sock < 0) || (data == NULL) || (len == 0U)) {
        return false;
    }

    size_t sent_total = 0U;
    while (sent_total < len) {
        ssize_t sent = zsock_send(sock, data + sent_total, len - sent_total, 0);
        if (sent <= 0) {
            return false;
        }
        sent_total += (size_t)sent;
    }

    return true;
}

static int tb45_http_parse_status_code(const char *response)
{
    if (response == NULL) {
        return -1;
    }

    int status = -1;
    int parsed = sscanf(response, "HTTP/%*u.%*u %d", &status);
    if (parsed != 1) {
        return -1;
    }

    return status;
}

static bool tb45_periodic_http_probe_target(const struct tb45_periodic_http_target *target)
{
#if !defined(CONFIG_NET_SOCKETS)
    ARG_UNUSED(target);
    return false;
#else
    if ((target == NULL) || (target->host == NULL) || (target->path == NULL) ||
        (target->expected_status <= 0)) {
        return false;
    }

    TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP probe target %s%s (expect %d)",
                               target->host, target->path, target->expected_status);

    struct zsock_addrinfo hints = {0};
    struct zsock_addrinfo *results = NULL;

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    int ret = zsock_getaddrinfo(target->host, TB45_PERIODIC_HTTP_PROBE_PORT, &hints, &results);
    if ((ret != 0) || (results == NULL)) {
        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP DNS resolve failed for %s:%s (%d)",
                                   target->host, TB45_PERIODIC_HTTP_PROBE_PORT, ret);
        return false;
    }

    bool target_ok = false;

    for (struct zsock_addrinfo *ai = results; ai != NULL; ai = ai->ai_next) {
        int sock = zsock_socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock < 0) {
            continue;
        }

        struct zsock_timeval timeout = {
            .tv_sec = TB45_PERIODIC_HTTP_CONNECT_TIMEOUT_MS / 1000,
            .tv_usec = (TB45_PERIODIC_HTTP_CONNECT_TIMEOUT_MS % 1000) * 1000,
        };
        (void)zsock_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        (void)zsock_setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        ret = zsock_connect(sock, ai->ai_addr, ai->ai_addrlen);
        if (ret < 0) {
            TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP connect failed for %s%s (errno %d)",
                                       target->host, target->path, errno);
            (void)zsock_close(sock);
            continue;
        }

        char request[256];
        int req_len = snprintf(request, sizeof(request),
                               "GET %s HTTP/1.1\r\n"
                               "Host: %s\r\n"
                               "User-Agent: tb45-periodic-probe/1.0\r\n"
                               "Connection: close\r\n"
                               "\r\n",
                               target->path, target->host);
        if ((req_len <= 0) || (req_len >= (int)sizeof(request))) {
            (void)zsock_close(sock);
            continue;
        }

        if (!tb45_http_send_all(sock, request, (size_t)req_len)) {
            TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP send failed for %s%s",
                                       target->host, target->path);
            (void)zsock_close(sock);
            continue;
        }

        char response_head[256];
        size_t total = 0U;
        bool have_status_line = false;

        while (total < (sizeof(response_head) - 1U)) {
            ssize_t rcvd = zsock_recv(sock, response_head + total,
                                      (sizeof(response_head) - 1U) - total, 0);
            if (rcvd <= 0) {
                break;
            }
            total += (size_t)rcvd;
            response_head[total] = '\0';
            if (strstr(response_head, "\r\n") != NULL) {
                have_status_line = true;
                break;
            }
        }

        int status = -1;
        if (have_status_line) {
            status = tb45_http_parse_status_code(response_head);
        }

        /*
         * Drain any small response body before close to keep socket state tidy.
         * Example: msftconnecttest.com returns a short text payload.
         */
        char drain_buf[96];
        while (zsock_recv(sock, drain_buf, sizeof(drain_buf), 0) > 0) {
        }

        (void)zsock_close(sock);

        if (status == target->expected_status) {
            TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP probe PASS for %s%s (status %d)",
                                       target->host, target->path, status);
            target_ok = true;
            break;
        }

        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: HTTP status mismatch for %s%s (got %d expected %d)",
                                   target->host, target->path, status, target->expected_status);
    }

    zsock_freeaddrinfo(results);
    return target_ok;
#endif
}

static bool tb45_periodic_http_probe_with_fallback(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(tb45_periodic_http_targets); i++) {
        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: trying HTTP target %u/%u",
                                   (unsigned int)(i + 1U),
                                   (unsigned int)ARRAY_SIZE(tb45_periodic_http_targets));
        if (tb45_periodic_http_probe_target(&tb45_periodic_http_targets[i])) {
            return true;
        }
    }

    TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: all HTTP probe targets failed");
    return false;
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
        TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: precheck failed up=%d dormant=%d carrier=%d route=%d ipv4=%d",
                                   iface_up, iface_dormant, carrier_ok, default_route_is_ppp, has_ipv4_addr);
        return false;
    }

    TB45_PERIODIC_HTTP_LOG_DBG("TB45 periodic: precheck passed; running HTTP fallback probe");
    return tb45_periodic_http_probe_with_fallback();
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
    ret = tb45_ppp_restart_sleep_interruptible(shell, TB45_PPP_RESTART_STEP_PAUSE_MS);
    if (ret < 0) {
        sequence_ret = ret;
        goto cleanup;
    }

show_info:
    if (sequence_ret != -ECANCELED) {
        shell_print(shell, "  3) tb45 show ppp_info");
        atomic_set(&tb45_ppp_down_triggered, 0);
        ret = cmd_tb45_ppp_info(shell, 1, NULL);
    }

cleanup:
    atomic_set(&tb45_ppp_recovery_internal_call, 0);
    atomic_set(&tb45_ppp_recovery_in_progress, 0);
    if (sequence_ret < 0) {
        return sequence_ret;
    }
    return ret;
}

static int cmd_tb45_ppp_up(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 ppp up");
        return -EINVAL;
    }

    struct net_if *iface = NULL;
    ret = tb45_get_ppp_iface(shell, &iface);
    if (ret < 0) {
        return ret;
    }

    if (atomic_cas(&tb45_ppp_down_triggered, 1, 0)) {
        shell_print(shell, "PPP up detected prior ppp-down trigger: running ppp restart policy");
        return cmd_tb45_ppp_restart(shell, 1, NULL);
    }

    ret = tb45_ppp_ipcp_set_state(iface, true);
    if ((ret != 0) && (ret != -EALREADY)) {
        shell_error(shell, "Failed to bring PPP up (%d)", ret);
        return ret;
    }

    ret = tb45_wait_for_ppp_link_ready(shell, iface, TB45_WAIT_PPP_READY_TIMEOUT_MS);
    if (ret < 0) {
        shell_error(shell, "PPP up timeout waiting for PPP_LINK_STATE active/ready");
        return ret;
    }
    shell_print(shell, "PPP link is active/ready (idx=%d)", net_if_get_by_iface(iface));

    net_if_set_default(iface);
    if (net_if_get_default() != iface) {
        shell_error(shell, "Failed to set default traffic route to PPP");
        return -EIO;
    }

    shell_print(shell, "Default traffic route set to PPP (idx=%d)", net_if_get_by_iface(iface));
    return cmd_tb45_ppp_info(shell, 1, NULL);
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

static int cmd_tb45_ppp_down(const struct shell *shell, size_t argc, char **argv)
{
    (void)argv;
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 1) {
        shell_error(shell, "Usage: tb45 ppp down");
        return -EINVAL;
    }

    struct net_if *iface = NULL;
    ret = tb45_get_ppp_iface(shell, &iface);
    if (ret < 0) {
        return ret;
    }

    ret = tb45_ppp_ipcp_set_state(iface, false);
    if ((ret == 0) || (ret == -EALREADY)) {
        shell_print(shell, "PPP interface is down (idx=%d)", net_if_get_by_iface(iface));
        return cmd_tb45_ppp_info(shell, 1, NULL);
    }

    shell_error(shell, "Failed to bring PPP down (%d)", ret);
    return ret;
}

static struct net_if *tb45_get_ethernet_iface(void)
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

static void tb45_ppp_default_traffic_route_print_usage(const struct shell *shell, size_t argc, char **argv)
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

    const char *value = (argc >= 2 && argv[1] != NULL) ? argv[1] : "";
    if ((strcmp(value, "") != 0) && (strcmp(value, "on") != 0) && (strcmp(value, "off") != 0)) {
        shell_fprintf(shell, SHELL_NORMAL, "Invalid input: ");
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " %s\n", value);
    }

    shell_fprintf(shell, SHELL_NORMAL, "Current value: ");
    shell_fprintf(shell, SHELL_VT100_COLOR_YELLOW, " %s\n", current);

    shell_fprintf(shell, SHELL_NORMAL, "Usage: tb45 ppp default_traffic_route <");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "on");
    shell_fprintf(shell, SHELL_NORMAL, "|");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN, "off");
    shell_fprintf(shell, SHELL_NORMAL, ">\n");
}

static int cmd_tb45_ppp_default_traffic_route(const struct shell *shell, size_t argc, char **argv)
{
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if (argc != 2) {
        tb45_ppp_default_traffic_route_print_usage(shell, argc, argv);
        return (argc == 1) ? 0 : -EINVAL;
    }

    if (strcmp(argv[1], "on") == 0) {
        struct net_if *iface = NULL;
        int ret = tb45_get_ppp_iface(shell, &iface);
        if (ret < 0) {
            return ret;
        }

        net_if_set_default(iface);
        shell_print(shell, "Default traffic route set to PPP (idx=%d)", net_if_get_by_iface(iface));
        return cmd_tb45_ppp_info(shell, 1, NULL);
    }

    if (strcmp(argv[1], "off") == 0) {
        struct net_if *iface = tb45_get_ethernet_iface();
        if (iface == NULL) {
            shell_error(shell, "No Ethernet interface found");
            return -ENODEV;
        }

        net_if_set_default(iface);
        shell_print(shell, "Default traffic route set to Ethernet (idx=%d)", net_if_get_by_iface(iface));
        return cmd_tb45_ppp_info(shell, 1, NULL);
    }

    tb45_ppp_default_traffic_route_print_usage(shell, argc, argv);
    return -EINVAL;
}

static void tb45_ping_print_usage(const struct shell *shell, char **argv, int argv_index)
{
    if (argv_index >= 0) {
        shell_fprintf(shell, SHELL_NORMAL, "Invalid input: ");
        shell_fprintf(shell, SHELL_VT100_COLOR_RED, " %s\n", argv[argv_index]);
    }

    shell_fprintf(shell, SHELL_NORMAL, "Usage: tb45 net ping <");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN    , "ipv4-address");
    shell_fprintf(shell, SHELL_NORMAL, "> [");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "count");
    shell_fprintf(shell, SHELL_NORMAL, "]\n");

    shell_fprintf(shell, SHELL_NORMAL, "Example: tb45 net ping ");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN    , "8.8.8.8\n");

    shell_fprintf(shell, SHELL_NORMAL, "Example: tb45 net ping ");
    shell_fprintf(shell, SHELL_VT100_COLOR_CYAN    , "8.8.8.8");
    shell_fprintf(shell, SHELL_NORMAL, " ");
    shell_fprintf(shell, SHELL_VT100_COLOR_BLUE, "5\n");
    shell_fprintf(shell, SHELL_NORMAL, "");
}

struct tb45_ping_ctx {
    const struct shell *shell;
    struct k_sem reply_sem;
    struct net_icmp_ctx icmp_ctx;
    struct net_if *iface;
    struct sockaddr_in dst_addr;
    long ping_count;
    int sent;
    int received;
    uint16_t identifier;
    volatile bool reply_received;
    volatile bool cancelled;
};

#define TB45_PING_THREAD_STACK_SIZE 1792
K_THREAD_STACK_DEFINE(tb45_ping_thread_stack, TB45_PING_THREAD_STACK_SIZE);
static struct k_thread tb45_ping_thread;
static struct tb45_ping_ctx tb45_ping_ctx;
static atomic_t tb45_ping_running;

#define TB45_ASCII_CTRL_C 0x03

#if ZEPHYR_VERSION_CODE < ZEPHYR_VERSION(4, 3, 0)
static struct tb45_ping_ctx *tb45_ping_bypass_ctx;
#endif

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 3, 0)
static void tb45_ping_bypass(const struct shell *sh, uint8_t *data, size_t len, void *user_data)
{
    ARG_UNUSED(sh);

    struct tb45_ping_ctx *ping_ctx = (struct tb45_ping_ctx *)user_data;
    if (ping_ctx == NULL) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        if (data[i] == TB45_ASCII_CTRL_C) {
            ping_ctx->cancelled = true;
            k_sem_give(&ping_ctx->reply_sem);
            break;
        }
    }
}
#else
static void tb45_ping_bypass(const struct shell *sh, uint8_t *data, size_t len)
{
    ARG_UNUSED(sh);

    struct tb45_ping_ctx *ping_ctx = tb45_ping_bypass_ctx;
    if (ping_ctx == NULL) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        if (data[i] == TB45_ASCII_CTRL_C) {
            ping_ctx->cancelled = true;
            k_sem_give(&ping_ctx->reply_sem);
            break;
        }
    }
}
#endif

static void tb45_ping_set_bypass(const struct shell *shell, struct tb45_ping_ctx *ctx)
{
#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 3, 0)
    shell_set_bypass(shell, tb45_ping_bypass, ctx);
#else
    tb45_ping_bypass_ctx = ctx;
    shell_set_bypass(shell, tb45_ping_bypass);
#endif
}

static void tb45_ping_clear_bypass(const struct shell *shell)
{
#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 3, 0)
    shell_set_bypass(shell, NULL, NULL);
#else
    tb45_ping_bypass_ctx = NULL;
    shell_set_bypass(shell, NULL);
#endif
}

static int tb45_ping_reply_handler(struct net_icmp_ctx *ctx, struct net_pkt *pkt,
                                   struct net_icmp_ip_hdr *ip_hdr,
                                   struct net_icmp_hdr *icmp_hdr, void *user_data);

static void tb45_ping_finish(struct tb45_ping_ctx *ctx)
{
    tb45_ping_clear_bypass(ctx->shell);
    (void)net_icmp_cleanup_ctx(&ctx->icmp_ctx);

    if (ctx->cancelled) {
        shell_print(ctx->shell, "Ping interrupted");
    }

    shell_print(ctx->shell, "Ping statistics: sent=%d received=%d lost=%d",
                ctx->sent, ctx->received, ctx->sent - ctx->received);
    atomic_clear(&tb45_ping_running);
}

static void tb45_ping_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct tb45_ping_ctx *ctx = (struct tb45_ping_ctx *)p1;
    int ret = TB45_NET_ICMP_INIT_CTX(&ctx->icmp_ctx, tb45_ping_reply_handler);
    if (ret < 0) {
        shell_error(ctx->shell, "Failed to initialize ICMP context (%d)", ret);
        atomic_clear(&tb45_ping_running);
        tb45_ping_clear_bypass(ctx->shell);
        return;
    }

    for (long i = 1; i <= ctx->ping_count; i++) {
        if (ctx->cancelled) {
            break;
        }

        struct net_icmp_ping_params params = {
            .identifier = ctx->identifier,
            .sequence = (uint16_t)i,
            .tc_tos = 0U,
            .priority = -1,
            .data = NULL,
            .data_size = 4U,
        };

        ctx->reply_received = false;
        ret = net_icmp_send_echo_request(&ctx->icmp_ctx, ctx->iface,
                                         (struct sockaddr *)&ctx->dst_addr,
                                         &params, ctx);
        if (ret < 0) {
            shell_error(ctx->shell, "Failed to send ping (%d)", ret);
            continue;
        }

        ctx->sent++;
        int wait_step_count = 20;
        bool got_reply = false;
        while (wait_step_count-- > 0) {
            if (k_sem_take(&ctx->reply_sem, K_MSEC(100)) == 0) {
                if (ctx->reply_received) {
                    got_reply = true;
                }
                break;
            }
            if (ctx->cancelled) {
                break;
            }
        }

        if (got_reply) {
            ctx->received++;
        } else if (!ctx->cancelled) {
            shell_print(ctx->shell, "Request timeout for icmp_seq=%ld", i);
        }

        if ((i < ctx->ping_count) && !ctx->cancelled) {
            int sleep_step_count = 10;
            while (sleep_step_count-- > 0) {
                if (ctx->cancelled) {
                    break;
                }
                k_msleep(100);
            }
        }
    }

    tb45_ping_finish(ctx);
}

static int tb45_ping_reply_handler(struct net_icmp_ctx *ctx, struct net_pkt *pkt,
                                   struct net_icmp_ip_hdr *ip_hdr,
                                   struct net_icmp_hdr *icmp_hdr, void *user_data)
{
    ARG_UNUSED(ctx);
    ARG_UNUSED(pkt);
    ARG_UNUSED(icmp_hdr);

    struct tb45_ping_ctx *ping_ctx = (struct tb45_ping_ctx *)user_data;
    if ((ping_ctx == NULL) || (ping_ctx->shell == NULL) || (ip_hdr == NULL) || (ip_hdr->ipv4 == NULL)) {
        return -EINVAL;
    }

    char addr_buf[NET_IPV4_ADDR_LEN];
    shell_print(ping_ctx->shell, "Reply from %s",
                net_addr_ntop(AF_INET, &ip_hdr->ipv4->src, addr_buf, sizeof(addr_buf)));
    ping_ctx->reply_received = true;
    k_sem_give(&ping_ctx->reply_sem);

    return 0;
}

static int cmd_tb45_ping(const struct shell *shell, size_t argc, char **argv)
{
    int ret = tb45_check_shell_menu_loaded(shell);
    if (ret < 0) {
        return ret;
    }

    if ((argc != 2) && (argc != 3)) {
        tb45_ping_print_usage(shell, argv, -1);
        return -EINVAL;
    }

    struct in_addr dst = {0};
    ret = net_addr_pton(AF_INET, argv[1], &dst);
    if (ret < 0) {
        tb45_ping_print_usage(shell, argv, 1);
        return -EINVAL;
    }

    long ping_count = PING_COUNT_DEFAULT;
    if (argc == 3) {
        char *endp = NULL;
        errno = 0;
        ping_count = strtol(argv[2], &endp, 10);
        if ((errno != 0) || (endp == argv[2]) || (*endp != '\0') || (ping_count <= 0) ||
            (ping_count > INT32_MAX)) {
            tb45_ping_print_usage(shell, argv, 2);
            return -EINVAL;
        }
    }

    struct net_if *iface = net_if_get_default();
    if (iface == NULL) {
        shell_error(shell, "No default network interface found");
        return -ENODEV;
    }

    const struct device *iface_dev = net_if_get_device(iface);
    const char *iface_name = (iface_dev != NULL) ? iface_dev->name : "<unknown>";

    shell_fprintf(shell, SHELL_NORMAL, "Pinging %s via interface ", argv[1]);
    shell_fprintf(shell, SHELL_VT100_COLOR_YELLOW, "%s\n", iface_name);

    if (!atomic_cas(&tb45_ping_running, 0, 1)) {
        shell_error(shell, "Ping is already running");
        return -EBUSY;
    }

    memset(&tb45_ping_ctx, 0, sizeof(tb45_ping_ctx));
    tb45_ping_ctx.shell = shell;
    tb45_ping_ctx.iface = iface;
    tb45_ping_ctx.ping_count = ping_count;
    tb45_ping_ctx.identifier = (uint16_t)(k_uptime_get_32() & 0xFFFF);
    tb45_ping_ctx.dst_addr.sin_family = AF_INET;
    tb45_ping_ctx.dst_addr.sin_addr = dst;
    k_sem_init(&tb45_ping_ctx.reply_sem, 0, 1);
    tb45_ping_set_bypass(shell, &tb45_ping_ctx);

    (void)k_thread_create(&tb45_ping_thread, tb45_ping_thread_stack,
                          K_THREAD_STACK_SIZEOF(tb45_ping_thread_stack),
                          tb45_ping_thread_entry, &tb45_ping_ctx, NULL, NULL,
                          K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&tb45_ping_thread, "tb45_ping");

    return 0;
}

static bool tb45_startup_ppp_check_internet_reachability_probe_with_http_fallback(void)
{
    LOG_DBG("TB45 startup: ppp_check_internet_reachability running HTTP request fallback probe");

    bool ok = tb45_periodic_http_probe_with_fallback();
    if (ok) {
        LOG_DBG("TB45 startup: ppp_check_internet_reachability HTTP request probe PASS");
    } else {
        LOG_DBG("TB45 startup: ppp_check_internet_reachability HTTP request probe FAIL");
    }

    return ok;
}

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_cell_cmds,
    SHELL_CMD(resume, NULL, "Resume modem_cellular state machine", cmd_tb45_cell_resume),
    SHELL_CMD(suspend, NULL, "Suspend modem_cellular state machine", cmd_tb45_cell_suspend),
    SHELL_CMD(restart, NULL, "Force suspend then resume recovery", cmd_tb45_cell_restart),
    SHELL_CMD(puk, NULL, "SIM PUK unlock: tb45 cell puk <pukcode> <pincode>", cmd_tb45_cell_puk),
    SHELL_CMD(unlock_pin, NULL, "SIM PUK unlock: tb45 cell unlock_pin <pukcode> <pincode>", cmd_tb45_cell_unlock_pin),
    SHELL_CMD(info, NULL, "Alias for tb45 show cell_info", cmd_tb45_cell_info),
    // SHELL_CMD(reclaim, NULL, "Suspend modem driver + repower modem hardware", cmd_tb45_cell_reclaim),
    // SHELL_CMD_ARG(at, NULL,
    //               "Run AT through modem shell: tb45 cell at [command] [expected_response] "
    //               "(default command: AT)",
    //               cmd_tb45_cell_at, 1, 2),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_ppp_cmds,
    SHELL_CMD(up, NULL, "Bring PPP interface up", cmd_tb45_ppp_up),
    SHELL_CMD(restart, NULL, "Force PPP recovery: cell_restart/up/route_on/show", cmd_tb45_ppp_restart),
    SHELL_CMD(down, NULL, "Bring PPP interface down", cmd_tb45_ppp_down),
    SHELL_CMD_ARG(default_traffic_route, NULL,
                  "Set default traffic route: tb45 ppp default_traffic_route <on|off>",
                  cmd_tb45_ppp_default_traffic_route, 1, 1),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_show_cmds,
    SHELL_CMD(summary, NULL, "Print both modem and PPP status info", cmd_tb45_show_summary),
    SHELL_CMD(cell_info, NULL, "Print modem info via cellular API", cmd_tb45_cell_info),
    SHELL_CMD(ppp_info, NULL, "Print PPP interface status and IPv4 info", cmd_tb45_ppp_info),
    SHELL_CMD(network_modes, NULL, "Print CNMP codes and meanings", cmd_tb45_show_network_modes),
    SHELL_CMD(probe_info, NULL, "Print periodic HTTP probe counters", cmd_tb45_probe_info),
    SHELL_CMD(restart_info, NULL, "Print restart counters since boot", cmd_tb45_restart_info),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_system_cmds,
			       SHELL_CMD(reboot, NULL, "Reboot the device", cmd_tb45_system_reboot),
			       SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_net_cmds,
    SHELL_CMD_ARG(ping, NULL, "Ping IPv4 host: tb45 net ping <ipv4-address> [count]",
                  cmd_tb45_ping, 1, 2),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(tb45_cmds,
    SHELL_CMD(help, NULL, "Show TB45 available commands banner", cmd_tb45_help),
    SHELL_CMD(cell, &tb45_cell_cmds, "Cellular controls: tb45 cell <resume|suspend|restart|puk|unlock_pin>", NULL),
    SHELL_CMD(show, &tb45_show_cmds, "Read-only status/info commands: tb45 show <ppp_info|cell_info|summary|network_modes|restart_info|probe_info>", NULL),
    SHELL_CMD(net, &tb45_net_cmds, "Network controls: tb45 net <ping>", NULL),
    SHELL_CMD(ppp, &tb45_ppp_cmds, "PPP controls: tb45 ppp <up|down|default_traffic_route>", NULL),
    SHELL_CMD(sms, &tb45_sms_cmds, "SMS controls: tb45 sms send <phone_number> <message>", NULL),
    SHELL_CMD(system, &tb45_system_cmds, "System commands", NULL),
    SHELL_SUBCMD_SET_END
);


SHELL_CMD_REGISTER(tb45, &tb45_cmds, "TB45 modem cellular controls", NULL);
#endif /* CONFIG_SHELL */
