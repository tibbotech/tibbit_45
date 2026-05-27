#ifndef TB45_ASYNC_JOB_INTERNAL_H_
#define TB45_ASYNC_JOB_INTERNAL_H_

#include "tb45_async_job.h"

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct k_msgq tb45_async_job_msgq;

int tb45_async_job_execute(const struct tb45_async_job *job);

#ifdef __cplusplus
}
#endif

#endif /* TB45_ASYNC_JOB_INTERNAL_H_ */
