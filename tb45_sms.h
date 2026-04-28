#ifndef TB45_SMS_H_
#define TB45_SMS_H_

#include <zephyr/shell/shell.h>

/**
 * @brief Send an SMS message via SIM7500.
 * * @param sh Shell context for logging (can be NULL).
 * @param phone Target phone number string.
 * @param text Message body string.
 * @return 0 on success, negative error code on failure.
 */
int tb45_sms_send(const struct shell *sh, const char *phone, const char *text);

/* Shell subcommand set to be included in tb45_cellular.c */
extern const union shell_cmd_entry tb45_sms_cmds;

#endif /* TB45_SMS_H_ */
