#ifndef TB45_CELLULAR_H
#define TB45_CELLULAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum tb45_cellular_auth_type {
    TB45_CELL_AUTH_NONE = 0,
    TB45_CELL_AUTH_PAP = 1,
    TB45_CELL_AUTH_CHAP = 2,
};

struct device;
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

enum cellular_stage_id {
	CELLULAR_STAGE_CPIN = 0,
	CELLULAR_STAGE_BAUDRATE,
	CELLULAR_STAGE_NETWORK_MODE,
	CELLULAR_STAGE_CMUX,
	CELLULAR_STAGE_DLCI,
	CELLULAR_STAGE_CONNECT,
	CELLULAR_STAGE_REG_READINESS,
};

enum cellular_stage_status {
	CELLULAR_STAGE_STATUS_VERIFIED = 0,
	CELLULAR_STAGE_STATUS_WARNING,
	CELLULAR_STAGE_STATUS_FAILED,
};

struct cellular_evt_stage_status {
	enum cellular_stage_id stage;
	enum cellular_stage_status status;
	bool retry_scheduled;
	const char *message;
};

/*
 * Runtime cellular configuration. NULL string fields are stored as "not
 * provided". When auth_type is PAP/CHAP, the modem driver normalizes missing
 * username/password fields to empty strings for AT+CGAUTH generation.
 * auth_type uses standard +CGAUTH values.
 */
struct tb45_cellular_config {
    const char *apn;
    const char *username;
    const char *password;
    uint8_t auth_type;
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
	int modem_restart_count;
	int full_bringup_restart_count;
};

/*
 * Initialize the TB45 cellular helper layer. Stores a copy of the strings in
 * cfg, registers the modem event callback and arms the deferred shell banner.
 * Must be called once from the application (e.g. early in main()) before the
 * modem driver state machine queries APN/auth/PIN/carrier settings. If cfg is
 * NULL, string fields remain unset, auth_type defaults to
 * TB45_CELL_AUTH_NONE, carrier_id defaults to AUTO semantics, and the probe
 * scheduler falls back to the low-priority work queue.
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
int tb45_cellular_shell_menu_require_loaded(const struct shell *shell);
int tb45_cellular_shell_check_menu_loaded(const struct shell *shell);
int tb45_cellular_shell_cmd_modem_on(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_off(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_modem_shutdown(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_ppp_restart(const struct shell *shell, size_t argc, char **argv);
int tb45_cellular_shell_cmd_system_shell_lock_override(const struct shell *shell, size_t argc,
					       char **argv);
int tb45_cellular_shell_ppp_get_iface(const struct shell *shell, struct net_if **iface_out);
int tb45_cellular_shell_ppp_set_state(struct net_if *iface, bool target_up);
int tb45_cellular_shell_ppp_wait_link_ready(const struct shell *shell, struct net_if *iface);
int tb45_cellular_shell_ppp_wait_ipv4_ready(const struct shell *shell, struct net_if *iface);
int tb45_cellular_shell_interrupt_begin(void);
void tb45_cellular_shell_interrupt_end(void);
bool tb45_cellular_shell_interrupt_cancel_pending(const struct shell *shell);
bool tb45_cellular_shell_modem_down_triggered(void);
bool tb45_cellular_shell_modem_off_in_progress(void);
int tb45_cellular_shell_modem_off_wait(const struct shell *shell);
void tb45_cellular_shell_print_available_commands_menu(const struct shell *shell);
bool tb45_cellular_shell_ppp_consume_down_triggered(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* TB45_CELLULAR_H */
