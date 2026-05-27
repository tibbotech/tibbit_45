#ifndef TB45_CELLULAR_H
#define TB45_CELLULAR_H

#include <zephyr/device.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct net_if;
struct k_work_q;
#ifdef CONFIG_SHELL
struct shell;
#endif

enum tb45_cellular_cpin_state {
	TB45_CELLULAR_CPIN_STATE_UNKNOWN = 0,
	TB45_CELLULAR_CPIN_STATE_READY,
	TB45_CELLULAR_CPIN_STATE_SIM_PIN,
	TB45_CELLULAR_CPIN_STATE_SIM_PUK,
	TB45_CELLULAR_CPIN_STATE_NOT_READY,
	TB45_CELLULAR_CPIN_STATE_NOT_INSERTED,
	TB45_CELLULAR_CPIN_STATE_PH_SIM_PIN,
	TB45_CELLULAR_CPIN_STATE_CME_ERROR_16,
};

/*
 * Runtime cellular configuration. NULL fields are treated as "not provided"
 * (same semantics as the previous weak-symbol contract used by
 * modem_cellular_custom).
 */
struct tb45_cellular_config {
    const char *apn;
    const char *username;
    const char *password;
    const char *sim_pin;
    const char *carrier_id;
    struct k_work_q *wq;
};

struct tb45_cellular_probe_info {
	int pass_count;
	int fail_count;
	int precheck_skip_count;
	int gate_skip_count;
	int periodic_interval_ms;
	int64_t active_since_ms;
};

struct tb45_cellular_restart_info {
	int ppp_restart_count;
	int cell_restart_count;
	int full_bringup_restart_count;
};

/*
 * Initialize the TB45 cellular helper layer. Stores a copy of the strings in
 * cfg, registers the modem event callback and arms the deferred shell banner.
 * Must be called once from the application (e.g. early in main()) before the
 * modem driver state machine queries APN/PIN/carrier settings. If cfg is NULL, APN/username/password/SIM PIN remain unset, carrier_id defaults to AUTO, and the probe scheduler defaults to low_priority_wq.
 */
int tb45_cellular_init(const struct tb45_cellular_config *cfg);
int tb45_cellular_probe_set_enabled(bool enabled);
int tb45_cellular_probe_get_enabled(bool *enabled_out);
int tb45_cellular_ppp_ready_post_actions(void);
const struct device *tb45_cellular_get_device(void);
int tb45_cellular_submit_sim_puk_unlock(const char *puk, const char *new_pin);
int tb45_cellular_get_cpin_state(int *state_out);
int tb45_cellular_get_runtime_baudrate(uint32_t *baudrate);
int tb45_cellular_get_runtime_network_mode(int *mode_code);
const char *tb45_cellular_network_mode_to_str(int code);
int tb45_cellular_get_probe_info(struct tb45_cellular_probe_info *info);
int tb45_cellular_get_restart_info(struct tb45_cellular_restart_info *info);

#ifdef CONFIG_SHELL
/* Shell adapter linkage APIs consumed by tb45_shell.c */
int tb45_cellular_shell_check_menu_loaded(const struct shell *shell);
int tb45_cellular_shell_cmd_cell_resume(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_cell_suspend(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_cell_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_ppp_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_ppp_get_iface(const struct shell *shell, struct net_if **iface_out);
int tb45_cellular_shell_ppp_set_state(struct net_if *iface, bool target_up);
int tb45_cellular_shell_ppp_wait_link_ready(const struct shell *shell, struct net_if *iface);
int tb45_cellular_shell_ppp_wait_ipv4_ready(const struct shell *shell, struct net_if *iface);
void tb45_cellular_shell_print_available_commands_menu(const struct shell *shell);
bool tb45_cellular_shell_ppp_consume_down_triggered(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TB45_CELLULAR_H */
