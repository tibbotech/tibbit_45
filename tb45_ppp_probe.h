#ifndef TB45_PPP_PROBE_H
#define TB45_PPP_PROBE_H

#include "tb45_cellular.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tb45_ppp_probe_ops {
    int (*reschedule_work)(struct k_work_delayable *dwork, k_timeout_t delay);
    int (*submit_work)(struct k_work *work);
    int (*restore_ppp_runtime_defaults)(void);
    bool (*ppp_iface_runtime_ready)(struct net_if *iface);
    bool (*ppp_iface_runtime_defaults_ready)(struct net_if *iface);
    void (*trigger_restart)(const char *reason);
};

#if IS_ENABLED(CONFIG_APP_TB45_PPP_PROBE_ENABLE)
int tb45_ppp_probe_init(const struct tb45_ppp_probe_ops *ops);
int tb45_ppp_probe_set_enabled(bool enabled);
int tb45_ppp_probe_get_enabled(bool *enabled_out);
int tb45_ppp_probe_get_info(struct tb45_cellular_probe_info *info);
void tb45_ppp_probe_reset_runtime_state(void);
void tb45_ppp_probe_on_startup_finalize(bool startup_check_done);
int tb45_ppp_probe_on_ppp_ready_post_actions(void);
bool tb45_ppp_probe_is_healthy_reachable(void);
#else
static inline int tb45_ppp_probe_init(const struct tb45_ppp_probe_ops *ops)
{
    ARG_UNUSED(ops);
    return 0;
}

static inline int tb45_ppp_probe_set_enabled(bool enabled)
{
    ARG_UNUSED(enabled);
    return 0;
}

static inline int tb45_ppp_probe_get_enabled(bool *enabled_out)
{
    if (enabled_out == NULL) {
        return -EINVAL;
    }

    *enabled_out = false;
    return 0;
}

static inline int tb45_ppp_probe_get_info(struct tb45_cellular_probe_info *info)
{
    if (info == NULL) {
        return -EINVAL;
    }

    memset(info, 0, sizeof(*info));
    return -ENOTSUP;
}

static inline void tb45_ppp_probe_reset_runtime_state(void)
{
}

static inline void tb45_ppp_probe_on_startup_finalize(bool startup_check_done)
{
    ARG_UNUSED(startup_check_done);
}

static inline int tb45_ppp_probe_on_ppp_ready_post_actions(void)
{
    return 0;
}

static inline bool tb45_ppp_probe_is_healthy_reachable(void)
{
    return true;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* TB45_PPP_PROBE_H */
