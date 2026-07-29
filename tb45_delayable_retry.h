#ifndef TB45_DELAYABLE_RETRY_H_
#define TB45_DELAYABLE_RETRY_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tb45_delayable_retry;

typedef int (*tb45_delayable_retry_run_fn)(struct tb45_delayable_retry *retry, uint8_t attempt);
typedef uint32_t (*tb45_delayable_retry_delay_ms_fn)(struct tb45_delayable_retry *retry, uint8_t attempt);
typedef void (*tb45_delayable_retry_attempt_failed_fn)(struct tb45_delayable_retry *retry, int ret,
					       uint8_t attempt, uint8_t max_attempts);
typedef void (*tb45_delayable_retry_complete_fn)(struct tb45_delayable_retry *retry, int ret);

struct tb45_delayable_retry_ops {
	tb45_delayable_retry_run_fn run_attempt;
	tb45_delayable_retry_delay_ms_fn retry_delay_ms;
	tb45_delayable_retry_attempt_failed_fn attempt_failed;
	tb45_delayable_retry_complete_fn complete;
};

struct tb45_delayable_retry {
	struct k_work_delayable dwork;
	const struct tb45_delayable_retry_ops *ops;
	uint8_t attempt;
	uint8_t max_attempts;
	bool pending;
	bool attempt_in_progress;
};

void tb45_delayable_retry_init(struct tb45_delayable_retry *retry,
			       const struct tb45_delayable_retry_ops *ops);
bool tb45_delayable_retry_start(struct tb45_delayable_retry *retry, uint8_t max_attempts);
bool tb45_delayable_retry_complete(struct tb45_delayable_retry *retry, int ret);
bool tb45_delayable_retry_is_pending(const struct tb45_delayable_retry *retry);

#ifdef __cplusplus
}
#endif

#endif /* TB45_DELAYABLE_RETRY_H_ */
