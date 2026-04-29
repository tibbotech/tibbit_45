#ifndef TB45_CELLULAR_H
#define TB45_CELLULAR_H

#include <zephyr/device.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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
};

/*
 * Initialize the TB45 cellular helper layer. Stores a copy of the strings in
 * cfg, registers the modem event callback and arms the deferred shell banner.
 * Must be called once from the application (e.g. early in main()) before the
 * modem driver state machine queries the APN/PIN. Passing cfg = NULL leaves
 * all fields unset (driver will see NULL getters).
 */
int tb45_cellular_init(const struct tb45_cellular_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* TB45_CELLULAR_H */
