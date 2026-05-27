#ifndef TB45_ASYNC_JOB_H_
#define TB45_ASYNC_JOB_H_

#include <stdint.h>

#include "tb45_sms.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TB45_ASYNC_JOB_PING_HOST_MAX_LEN CONFIG_APP_TB45_ASYNC_PING_HOST_MAX_LEN

enum tb45_async_job_type {
	TB45_ASYNC_JOB_TYPE_SMS_SEND = 0,
	TB45_ASYNC_JOB_TYPE_PING_RUN = 1,
	TB45_ASYNC_JOB_TYPE_COUNT
};

struct tb45_async_job_sms_send {
	struct tb45_sms_request request;
	uint8_t capture_result;
	void *completion_ctx;
};

struct tb45_async_job_ping_run {
	char host[TB45_ASYNC_JOB_PING_HOST_MAX_LEN + 1];
	uint16_t count;
	uint32_t timeout_ms;
	void *completion_ctx;
};

struct tb45_async_job {
	enum tb45_async_job_type type;
	union {
		struct tb45_async_job_sms_send sms_send;
		struct tb45_async_job_ping_run ping_run;
	} payload;
};

typedef int (*tb45_async_job_handler_t)(const struct tb45_async_job *job);

int tb45_async_job_register_handler(enum tb45_async_job_type type,
				    tb45_async_job_handler_t handler);
int tb45_async_job_enqueue(const struct tb45_async_job *job);

#ifdef __cplusplus
}
#endif

#endif /* TB45_ASYNC_JOB_H_ */
