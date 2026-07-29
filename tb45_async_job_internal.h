#ifndef TB45_ASYNC_JOB_INTERNAL_H_
#define TB45_ASYNC_JOB_INTERNAL_H_

#include "tb45_async_job.h"

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct k_msgq tb45_async_job_msgq;

struct tb45_async_job_policy {
	uint8_t max_attempts;
};

struct tb45_async_job_policy tb45_async_job_policy_get(enum tb45_async_job_type type);
uint32_t tb45_async_job_retry_delay_ms(enum tb45_async_job_type type);
int tb45_async_job_execute(const struct tb45_async_job *job);
void tb45_async_job_complete(const struct tb45_async_job *job, int ret);
void tb45_async_dispatcher_complete_job(const struct tb45_async_job *job, int ret);
void tb45_async_dispatcher_complete_attempt(int ret);

#ifdef __cplusplus
}
#endif

#endif /* TB45_ASYNC_JOB_INTERNAL_H_ */
