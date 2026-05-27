#ifndef TB45_PING_H_
#define TB45_PING_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/net/net_ip.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TB45_PING_DEFAULT_PAYLOAD_SIZE 4U
#define TB45_PING_MAX_PAYLOAD_SIZE     1472U

struct tb45_ping_progress {
	const char *host;
	uint16_t sequence;
	uint32_t elapsed_ms;
	uint32_t timeout_ms;
	size_t payload_size;
	bool success;
	int status;
	struct in_addr reply_addr;
};

typedef void (*tb45_ping_progress_cb_t)(const struct tb45_ping_progress *progress,
					void *user_data);

int tb45_ping_enqueue(const char *host, uint16_t count, uint32_t timeout_ms);
/**
 * @brief Queue ping execution on async worker and wait for final result.
 *
 * Uses the same async dispatch path as tb45_ping_enqueue(), but blocks until
 * that queued job completes.
 */
int tb45_ping_enqueue_wait(const char *host, uint16_t count, uint32_t timeout_ms);
int tb45_ping_run(const char *host, uint16_t count, uint32_t timeout_ms);
int tb45_ping_run_ex(const char *host, uint16_t count, uint32_t timeout_ms,
		     size_t payload_size, tb45_ping_progress_cb_t progress_cb,
		     void *progress_user_data);

#ifdef __cplusplus
}
#endif

#endif /* TB45_PING_H_ */
