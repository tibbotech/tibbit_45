#ifndef TB45_SHELL_H_
#define TB45_SHELL_H_

#include <zephyr/shell/shell.h>

#if defined(CONFIG_SHELL) && defined(CONFIG_SHELL_BACKEND_SERIAL) && \
	defined(CONFIG_SHELL_LOG_BACKEND) && defined(CONFIG_UART_CONSOLE)
#define TB45_SHELL_COMMANDS_ENABLED 1
#else
#define TB45_SHELL_COMMANDS_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

int tb45_shell_cmd_sms_send(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_sms_init(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_net_ping(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_help(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_show_summary(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_show_network_modes(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_show_service_provider(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_cell_info(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_cell_resume(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_cell_suspend(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_cell_restart(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_cell_unlock_pin(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_ppp_up(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_ppp_restart(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_ppp_down(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_ppp_default_traffic_route(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_ppp_info(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_probe_info(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_restart_info(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_probe_on(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_probe_off(const struct shell *sh, size_t argc, char **argv);
int tb45_shell_cmd_system_reboot(const struct shell *sh, size_t argc, char **argv);
extern const union shell_cmd_entry tb45_sms_cmds;

#ifdef __cplusplus
}
#endif

#endif /* TB45_SHELL_H_ */
