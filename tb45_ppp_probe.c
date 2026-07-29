#include "tb45_ppp_probe.h"

#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_l2.h>

#include "tb45_ping.h"

LOG_MODULE_DECLARE(modem_cellular_custom, CONFIG_MODEM_LOG_LEVEL);

#if defined(CONFIG_APP_TB45_GOOGLE_IPV4)
#define TB45_GOOGLE_IPV4 CONFIG_APP_TB45_GOOGLE_IPV4
#else
#define TB45_GOOGLE_IPV4 "8.8.8.8"
#endif

#define TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS 4
#ifndef CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#if defined(CONFIG_APP_TB45_MODEM_BRINGUP_MAX_RETRIES)
#define CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES CONFIG_APP_TB45_MODEM_BRINGUP_MAX_RETRIES
#else
#define CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES 30
#endif
#endif
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_PROBE_INTERVAL_MS)
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS CONFIG_APP_TB45_PPP_PERIODIC_PROBE_INTERVAL_MS
#else
#define TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS 30000
#endif
#define TB45_PPP_PERIODIC_HEALTH_UNREACHABLE_INTERVAL_MS 5000
#define TB45_PPP_PERIODIC_ARM_RETRY_MS 1000
#define TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD TB45_PPP_CHECK_INTERNET_REACHABILITY_MAX_ATTEMPTS
#define TB45_PPP_PERIODIC_FULL_RESTART_MAX_RETRIES CONFIG_MODEM_CELLULAR_BRINGUP_MAX_RETRIES
#if defined(CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS CONFIG_APP_TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS
#else
#define TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS 60000
#endif
#define TB45_PPP_RUNTIME_DEFAULT_RESTORE_DELAY_MS 500
#define TB45_WAIT_PPP_READY_TIMEOUT_MS 30000
#define TB45_PPP_RUNTIME_DEFAULT_RESTORE_MAX_ATTEMPTS \
    (TB45_WAIT_PPP_READY_TIMEOUT_MS / TB45_PPP_RUNTIME_DEFAULT_RESTORE_DELAY_MS)

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

static const struct tb45_ppp_probe_ops *tb45_probe_ops = NULL;

static void tb45_ppp_periodic_arm_work_handler(struct k_work *work);
static void tb45_ppp_periodic_health_work_handler(struct k_work *work);
static void tb45_ppp_runtime_defaults_work_handler(struct k_work *work);
static void tb45_ppp_periodic_recovery_work_handler(struct k_work *work);
static void tb45_probe_ping_quiet_progress_cb(const struct tb45_ping_progress *progress,
                                              void *user_data);
static void tb45_periodic_ping_probe_handle_result(bool probe_ok);
static void tb45_periodic_ping_probe_complete(int ret, void *user_data);
static int tb45_submit_periodic_recovery_work(void);
static void tb45_ppp_periodic_arm_schedule(int delay_ms);
static void tb45_ppp_periodic_health_schedule(int delay_ms);
static void tb45_ppp_periodic_health_stop(void);
static bool tb45_ppp_probe_ops_ready(void);

static K_WORK_DELAYABLE_DEFINE(tb45_ppp_periodic_arm_work, tb45_ppp_periodic_arm_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_ppp_periodic_health_work,
                               tb45_ppp_periodic_health_work_handler);
static K_WORK_DELAYABLE_DEFINE(tb45_ppp_runtime_defaults_work,
                               tb45_ppp_runtime_defaults_work_handler);
static K_WORK_DEFINE(tb45_ppp_periodic_recovery_work,
                     tb45_ppp_periodic_recovery_work_handler);

static atomic_t tb45_ppp_periodic_recovery_running = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_pass_count = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_fail_count = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_precheck_skip_count = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_gate_skip_count = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_enabled = ATOMIC_INIT(0);
static atomic_t tb45_periodic_probe_pending = ATOMIC_INIT(0);
static int tb45_ppp_periodic_consecutive_failures = 0;
static int64_t tb45_ppp_periodic_last_recovery_ms = 0;
static int64_t tb45_ppp_periodic_active_since_ms = 0;
static atomic_t tb45_ppp_periodic_cycle_timeout_ms = ATOMIC_INIT(0);
static atomic_t tb45_ppp_periodic_full_restart_attempts = ATOMIC_INIT(0);
static atomic_t tb45_ppp_runtime_default_restore_attempts = ATOMIC_INIT(0);

static bool tb45_ppp_probe_ops_ready(void)
{
    return (tb45_probe_ops != NULL) &&
           (tb45_probe_ops->reschedule_work != NULL) &&
           (tb45_probe_ops->submit_work != NULL) &&
           (tb45_probe_ops->restore_ppp_runtime_defaults != NULL) &&
           (tb45_probe_ops->ppp_iface_runtime_ready != NULL) &&
           (tb45_probe_ops->ppp_iface_runtime_defaults_ready != NULL) &&
           (tb45_probe_ops->trigger_restart != NULL);
}

static int tb45_ppp_probe_reschedule_ms(struct k_work_delayable *dwork, int delay_ms)
{
    if (!tb45_ppp_probe_ops_ready()) {
        return -EAGAIN;
    }

    if (delay_ms < 0) {
        delay_ms = 0;
    }

    return tb45_probe_ops->reschedule_work(dwork, K_MSEC(delay_ms));
}

static int tb45_ppp_probe_submit_work(struct k_work *work)
{
    if (!tb45_ppp_probe_ops_ready()) {
        return -EAGAIN;
    }

    return tb45_probe_ops->submit_work(work);
}

static bool tb45_ppp_probe_iface_runtime_ready(struct net_if *iface)
{
    return tb45_ppp_probe_ops_ready() && tb45_probe_ops->ppp_iface_runtime_ready(iface);
}

static bool tb45_ppp_probe_iface_runtime_defaults_ready(struct net_if *iface)
{
    return tb45_ppp_probe_ops_ready() && tb45_probe_ops->ppp_iface_runtime_defaults_ready(iface);
}

static int tb45_ppp_probe_restore_runtime_defaults(void)
{
    if (!tb45_ppp_probe_ops_ready()) {
        return -EAGAIN;
    }

    return tb45_probe_ops->restore_ppp_runtime_defaults();
}

static void tb45_ppp_periodic_handle_failure(const char *reason)
{
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms,
               TB45_PPP_PERIODIC_HEALTH_UNREACHABLE_INTERVAL_MS);
    tb45_ppp_periodic_consecutive_failures++;
    TB45_PPP_CHECK_LOG_WRN("%s (%d/%d)", reason,
                           tb45_ppp_periodic_consecutive_failures,
                           TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD);

    if (tb45_ppp_periodic_consecutive_failures < TB45_PPP_PERIODIC_HEALTH_FAIL_THRESHOLD) {
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    int64_t now_ms = k_uptime_get();
    if ((tb45_ppp_periodic_last_recovery_ms != 0) &&
        ((now_ms - tb45_ppp_periodic_last_recovery_ms) <
         TB45_PPP_PERIODIC_RECOVERY_COOLDOWN_MS)) {
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

static void tb45_periodic_ping_probe_handle_result(bool probe_ok)
{
    if (probe_ok) {
        atomic_inc(&tb45_periodic_probe_pass_count);
    } else {
        atomic_inc(&tb45_periodic_probe_fail_count);
    }

    if (probe_ok) {
        tb45_ppp_periodic_consecutive_failures = 0;
        atomic_set(&tb45_ppp_periodic_full_restart_attempts, 0);
        atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        tb45_ppp_periodic_health_schedule(-1);
        return;
    }

    tb45_ppp_periodic_handle_failure("TB45 periodic: internet check failed");
}

static void tb45_periodic_ping_probe_complete(int ret, void *user_data)
{
    ARG_UNUSED(user_data);

    atomic_set(&tb45_periodic_probe_pending, 0);

    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    if (ret < 0) {
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: ping probe failed (%d)", ret);
        tb45_periodic_ping_probe_handle_result(false);
        return;
    }

    tb45_periodic_ping_probe_handle_result(true);
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

    (void)tb45_ppp_probe_reschedule_ms(&tb45_ppp_periodic_health_work, delay_ms);
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

    (void)tb45_ppp_probe_reschedule_ms(&tb45_ppp_periodic_arm_work, delay_ms);
}

static void tb45_ppp_periodic_health_stop(void)
{
    tb45_ppp_periodic_consecutive_failures = 0;
    tb45_ppp_periodic_active_since_ms = 0;
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    atomic_set(&tb45_periodic_probe_pending, 0);
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
    bool ready = tb45_ppp_probe_iface_runtime_defaults_ready(iface);
    if (!ready && tb45_ppp_probe_iface_runtime_ready(iface)) {
        (void)tb45_ppp_probe_restore_runtime_defaults();
        ready = tb45_ppp_probe_iface_runtime_defaults_ready(iface);
    }

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
    ARG_UNUSED(work);

    atomic_inc(&tb45_ppp_periodic_full_restart_attempts);
    atomic_set(&tb45_ppp_periodic_recovery_running, 0);
    tb45_probe_ops->trigger_restart("periodic health failed");
}

static int tb45_submit_periodic_recovery_work(void)
{
    return tb45_ppp_probe_submit_work(&tb45_ppp_periodic_recovery_work);
}

static void tb45_ppp_periodic_health_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) ||
        (atomic_get(&tb45_periodic_probe_enabled) == 0)) {
        return;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    bool ready = tb45_ppp_probe_iface_runtime_defaults_ready(iface);
    if (!ready && tb45_ppp_probe_iface_runtime_ready(iface)) {
        (void)tb45_ppp_probe_restore_runtime_defaults();
        ready = tb45_ppp_probe_iface_runtime_defaults_ready(iface);
    }

    if (!ready) {
        tb45_ppp_periodic_active_since_ms = 0;
        atomic_inc(&tb45_periodic_probe_gate_skip_count);
        tb45_ppp_periodic_handle_failure("TB45 periodic: PPP/default-route not ready");
        return;
    }

    if (!atomic_cas(&tb45_periodic_probe_pending, 0, 1)) {
        return;
    }
    int ret = tb45_ping_enqueue_ex(TB45_GOOGLE_IPV4, 0U, 0U,
                                   TB45_PING_DEFAULT_PAYLOAD_SIZE,
                                   tb45_probe_ping_quiet_progress_cb,
                                   tb45_periodic_ping_probe_complete, NULL, NULL);
    if (ret < 0) {
        atomic_set(&tb45_periodic_probe_pending, 0);
        TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: failed to queue ping probe (%d)", ret);
        tb45_periodic_ping_probe_handle_result(false);
    }
}

int tb45_ppp_probe_init(const struct tb45_ppp_probe_ops *ops)
{
    if (ops == NULL) {
        return -EINVAL;
    }

    tb45_probe_ops = ops;
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    return tb45_ppp_probe_ops_ready() ? 0 : -EINVAL;
}

int tb45_ppp_probe_set_enabled(bool enabled)
{
    if (!enabled) {
        atomic_set(&tb45_periodic_probe_enabled, 0);
        atomic_set(&tb45_ppp_periodic_full_restart_attempts, 0);
        atomic_set(&tb45_ppp_runtime_default_restore_attempts, 0);
        tb45_ppp_periodic_health_stop();
        (void)k_work_cancel_delayable(&tb45_ppp_runtime_defaults_work);
        return 0;
    }

    atomic_set(&tb45_periodic_probe_enabled, 1);

    if (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS <= 0) {
        return 0;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    bool ready = tb45_ppp_probe_iface_runtime_defaults_ready(iface);
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

int tb45_ppp_probe_get_enabled(bool *enabled_out)
{
    if (enabled_out == NULL) {
        return -EINVAL;
    }

    *enabled_out = (atomic_get(&tb45_periodic_probe_enabled) != 0);
    return 0;
}

int tb45_ppp_probe_get_info(struct tb45_cellular_probe_info *info)
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

void tb45_ppp_probe_reset_runtime_state(void)
{
    atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
    atomic_set(&tb45_ppp_periodic_recovery_running, 0);
    atomic_set(&tb45_ppp_runtime_default_restore_attempts, 0);
    atomic_set(&tb45_ppp_periodic_full_restart_attempts, 0);
    tb45_ppp_periodic_last_recovery_ms = 0;
    tb45_ppp_periodic_health_stop();
}

void tb45_ppp_probe_on_startup_finalize(bool startup_check_done)
{
    tb45_ppp_periodic_consecutive_failures = 0;

    if (startup_check_done &&
        (TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) &&
        (atomic_get(&tb45_periodic_probe_enabled) != 0)) {
        tb45_ppp_periodic_active_since_ms = k_uptime_get();
        atomic_set(&tb45_ppp_periodic_cycle_timeout_ms, TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        LOG_DBG("TB45 health check\r\n"
            "  ppp_check_internet_reachability: checking...please wait\r\n"
            "  TB45 periodic: health check armed (interval=%d ms)",
            TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS);
        tb45_ppp_periodic_health_schedule(-1);
    } else if ((TB45_PPP_PERIODIC_HEALTH_INTERVAL_MS > 0) &&
               (atomic_get(&tb45_periodic_probe_enabled) != 0)) {
        LOG_WRN("TB45 periodic: health check not armed because startup internet check did not succeed");
    }
}

static void tb45_ppp_runtime_defaults_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int attempt = atomic_get(&tb45_ppp_runtime_default_restore_attempts) + 1;
    atomic_set(&tb45_ppp_runtime_default_restore_attempts, attempt);

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    bool ready = tb45_ppp_probe_iface_runtime_ready(iface);

    if (!ready) {
        if (attempt < TB45_PPP_RUNTIME_DEFAULT_RESTORE_MAX_ATTEMPTS) {
            (void)tb45_ppp_probe_reschedule_ms(&tb45_ppp_runtime_defaults_work,
                                               TB45_PPP_RUNTIME_DEFAULT_RESTORE_DELAY_MS);
        }
        return;
    }

    if (tb45_ppp_probe_iface_runtime_defaults_ready(iface)) {
        (void)tb45_ppp_probe_set_enabled(true);
        return;
    }

    int ret = tb45_ppp_probe_restore_runtime_defaults();
    if ((ret == 0) && tb45_ppp_probe_iface_runtime_defaults_ready(iface)) {
        (void)tb45_ppp_probe_set_enabled(true);
        return;
    }

    if (attempt < TB45_PPP_RUNTIME_DEFAULT_RESTORE_MAX_ATTEMPTS) {
        (void)tb45_ppp_probe_reschedule_ms(&tb45_ppp_runtime_defaults_work,
                                           TB45_PPP_RUNTIME_DEFAULT_RESTORE_DELAY_MS);
    }
}

int tb45_ppp_probe_on_ppp_ready_post_actions(void)
{
    atomic_set(&tb45_ppp_runtime_default_restore_attempts, 0);
    (void)tb45_ppp_probe_reschedule_ms(&tb45_ppp_runtime_defaults_work,
                                       TB45_PPP_RUNTIME_DEFAULT_RESTORE_DELAY_MS);
    return 0;
}

static void tb45_probe_ping_quiet_progress_cb(const struct tb45_ping_progress *progress,
                                              void *user_data)
{
    ARG_UNUSED(progress);
    ARG_UNUSED(user_data);
}

bool tb45_ppp_probe_is_healthy_reachable(void)
{
    if (!IS_ENABLED(CONFIG_NET_L2_PPP)) {
        return false;
    }

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
    if (iface == NULL) {
        return false;
    }

    bool runtime_ready = tb45_ppp_probe_iface_runtime_ready(iface);
    bool default_route_is_ppp = (net_if_get_default() == iface);
    if (!runtime_ready || !default_route_is_ppp) {
        atomic_inc(&tb45_periodic_probe_precheck_skip_count);
        TB45_PERIODIC_PROBE_LOG_DBG(
            "TB45 periodic: precheck failed runtime_ready=%d route=%d",
            runtime_ready, default_route_is_ppp);
        return false;
    }

    TB45_PERIODIC_PROBE_LOG_DBG("TB45 periodic: precheck passed; running ping probe to %s",
                                TB45_GOOGLE_IPV4);
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
