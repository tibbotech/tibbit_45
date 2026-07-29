#ifndef TB45_ASYNC_JOB_H_
#define TB45_ASYNC_JOB_H_

#include <stdint.h>

#include <zephyr/kernel.h>

#include "tb45_ping.h"
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
#include "tb45_sms.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TB45_ASYNC_JOB_PING_HOST_MAX_LEN CONFIG_APP_TB45_ASYNC_PING_HOST_MAX_LEN

enum tb45_async_job_type {
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
	TB45_ASYNC_JOB_TYPE_SMS_SEND = 0,
	TB45_ASYNC_JOB_TYPE_SMS_SEND_CAPTURE_RESULT = 1,
	TB45_ASYNC_JOB_TYPE_SMS_SEND_WAIT = 2,
#endif
	TB45_ASYNC_JOB_TYPE_PING_RUN,
	TB45_ASYNC_JOB_TYPE_COUNT
};

#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
struct tb45_async_job_sms_send {
	struct tb45_sms_request request;
};

struct tb45_async_job_sms_send_wait {
	struct tb45_sms_request request;
	void *completion_ctx;
};
#endif

struct tb45_async_job_ping_run {
	char host[TB45_ASYNC_JOB_PING_HOST_MAX_LEN + 1];
	uint16_t count;
	uint32_t timeout_ms;
	uint16_t payload_size;
	tb45_ping_progress_cb_t progress_cb;
	tb45_ping_complete_cb_t complete_cb;
	void *progress_user_data;
	void *complete_user_data;
	void *completion_ctx;
};

struct tb45_async_job {
	enum tb45_async_job_type type;
	union {
#if defined(CONFIG_APP_TB45_SMS_ENABLE) && CONFIG_APP_TB45_SMS_ENABLE
		struct tb45_async_job_sms_send sms_send;
		struct tb45_async_job_sms_send_wait sms_send_wait;
#endif
		struct tb45_async_job_ping_run ping_run;
	} payload;
};

typedef int (*tb45_async_job_handler_t)(const struct tb45_async_job *job);
typedef void (*tb45_async_job_complete_cb_t)(const struct tb45_async_job *job, int ret);

struct tb45_async_wait_ctx {
	struct k_sem done;
	int result;
};

static inline void tb45_async_wait_ctx_init(struct tb45_async_wait_ctx *wait_ctx, int initial_result)
{
	if (wait_ctx == NULL) {
		return;
	}

	k_sem_init(&wait_ctx->done, 0, 1);
	wait_ctx->result = initial_result;
}

int tb45_async_job_register_handler(enum tb45_async_job_type type,
				    tb45_async_job_handler_t handler,
				    tb45_async_job_complete_cb_t complete_cb);
int tb45_async_job_enqueue(const struct tb45_async_job *job);
void tb45_async_job_complete_wait_ctx(struct tb45_async_wait_ctx *wait_ctx, int ret);

#ifdef __cplusplus
}
#endif

#endif /* TB45_ASYNC_JOB_H_ */
