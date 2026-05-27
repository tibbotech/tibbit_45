#include "tb45_ping.h"

#include "tb45_async_job.h"

#include <errno.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/icmp.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/version.h>

LOG_MODULE_REGISTER(tb45_ping, CONFIG_LOG_DEFAULT_LEVEL);

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 3, 0)
#define TB45_PING_NET_ICMP_INIT_CTX(ctx, handler) \
	net_icmp_init_ctx((ctx), NET_AF_INET, NET_ICMPV4_ECHO_REPLY, 0, (handler))
#else
#define TB45_PING_NET_ICMP_INIT_CTX(ctx, handler) \
	net_icmp_init_ctx((ctx), NET_ICMPV4_ECHO_REPLY, 0, (handler))
#endif

#define TB45_PING_BETWEEN_REQUESTS_MS 1000
#define TB45_PING_WAIT_POLL_MS        100

struct tb45_ping_run_ctx {
	struct k_sem reply_sem;
	struct net_icmp_ctx icmp_ctx;
	struct net_if *iface;
	struct sockaddr_in dst_addr;
	uint16_t identifier;
	uint16_t expected_identifier;
	uint16_t expected_sequence;
	uint32_t timeout_ms;
	size_t payload_size;
	tb45_ping_progress_cb_t progress_cb;
	void *progress_user_data;
	int sent;
	int received;
	struct in_addr reply_addr;
	volatile bool reply_received;
};

struct tb45_ping_wait_ctx {
	struct k_sem done;
	int result;
};


static int tb45_ping_parse_reply_id_seq(struct net_pkt *pkt, uint16_t *identifier,
					uint16_t *sequence)
{
	if ((pkt == NULL) || (identifier == NULL) || (sequence == NULL)) {
		return -EINVAL;
	}

	struct net_pkt_cursor cursor_backup;
	net_pkt_cursor_backup(pkt, &cursor_backup);

	int ret = 0;
	net_pkt_cursor_init(pkt);
	if (net_pkt_skip(pkt, net_pkt_ip_hdr_len(pkt)) < 0) {
		ret = -EINVAL;
		goto out;
	}

	if (net_pkt_skip(pkt, sizeof(struct net_icmp_hdr)) < 0) {
		ret = -EINVAL;
		goto out;
	}

	if (net_pkt_read_be16(pkt, identifier) < 0) {
		ret = -EINVAL;
		goto out;
	}

	if (net_pkt_read_be16(pkt, sequence) < 0) {
		ret = -EINVAL;
		goto out;
	}

out:
	net_pkt_cursor_restore(pkt, &cursor_backup);
	return ret;
}

static void tb45_ping_report_progress(const struct tb45_async_job_ping_run *req,
				      struct tb45_ping_run_ctx *ctx, uint16_t sequence,
				      uint32_t elapsed_ms, bool success, int status)
{
	if ((ctx == NULL) || (ctx->progress_cb == NULL) || (req == NULL)) {
		return;
	}

	struct tb45_ping_progress progress = {
		.host = req->host,
		.sequence = sequence,
		.elapsed_ms = elapsed_ms,
		.timeout_ms = ctx->timeout_ms,
		.payload_size = ctx->payload_size,
		.success = success,
		.status = status,
		.reply_addr = ctx->reply_addr,
	};

	ctx->progress_cb(&progress, ctx->progress_user_data);
}

static int tb45_ping_reply_handler(struct net_icmp_ctx *ctx, struct net_pkt *pkt,
				   struct net_icmp_ip_hdr *ip_hdr,
				   struct net_icmp_hdr *icmp_hdr, void *user_data)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(icmp_hdr);

	struct tb45_ping_run_ctx *ping_ctx = (struct tb45_ping_run_ctx *)user_data;
	if ((ping_ctx == NULL) || (ip_hdr == NULL) || (ip_hdr->ipv4 == NULL) || (pkt == NULL)) {
		return 0;
	}

	struct in_addr src_addr = {0};
	net_ipv4_addr_copy_raw(src_addr.s4_addr, ip_hdr->ipv4->src);
	if (!net_ipv4_addr_cmp(&src_addr, &ping_ctx->dst_addr.sin_addr)) {
		return 0;
	}

	uint16_t reply_identifier = 0U;
	uint16_t reply_sequence = 0U;
	if (tb45_ping_parse_reply_id_seq(pkt, &reply_identifier, &reply_sequence) < 0) {
		return 0;
	}

	if ((reply_identifier != ping_ctx->expected_identifier) ||
	    (reply_sequence != ping_ctx->expected_sequence)) {
		return 0;
	}

	if (ping_ctx->progress_cb == NULL) {
		char addr_buf[NET_IPV4_ADDR_LEN];
		const char *src = net_addr_ntop(AF_INET, &src_addr, addr_buf, sizeof(addr_buf));
		LOG_INF("PING: reply from %s", (src != NULL) ? src : "<invalid>");
	}
	ping_ctx->reply_addr = src_addr;
	ping_ctx->reply_received = true;
	k_sem_give(&ping_ctx->reply_sem);
	return 0;
}

static int tb45_ping_resolve_ipv4_host(const char *host, struct in_addr *out_addr)
{
	struct zsock_addrinfo hints = {0};
	struct zsock_addrinfo *results = NULL;
	struct zsock_addrinfo *entry;
	int ret;

	if ((host == NULL) || (out_addr == NULL)) {
		return -EINVAL;
	}

	ret = net_addr_pton(AF_INET, host, out_addr);
	if (ret >= 0) {
		return 0;
	}

	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	ret = zsock_getaddrinfo(host, NULL, &hints, &results);
	if ((ret != 0) || (results == NULL)) {
		LOG_ERR("PING: resolve failed for host '%s' (%d)", host, ret);
		return -EHOSTUNREACH;
	}

	for (entry = results; entry != NULL; entry = entry->ai_next) {
		if ((entry->ai_family != AF_INET) || (entry->ai_addr == NULL)) {
			continue;
		}

		const struct sockaddr_in *addr_in = (const struct sockaddr_in *)entry->ai_addr;
		*out_addr = addr_in->sin_addr;
		zsock_freeaddrinfo(results);
		return 0;
	}

	zsock_freeaddrinfo(results);
	LOG_ERR("PING: no IPv4 result for host '%s'", host);
	return -EHOSTUNREACH;
}

static int tb45_ping_run_internal
(const struct tb45_async_job_ping_run *req,
				  size_t payload_size,
				  tb45_ping_progress_cb_t progress_cb,
				  void *progress_user_data)
{
	struct in_addr dst = {0};
	struct tb45_ping_run_ctx ctx;
	int ret;

	ret = tb45_ping_resolve_ipv4_host(req->host, &dst);
	if (ret < 0) {
		LOG_ERR("PING: host resolution failed for '%s' (%d)", req->host, ret);
		return ret;
	}

	struct net_if *iface = net_if_get_default();
	if (iface == NULL) {
		LOG_ERR("PING: no default network interface");
		return -ENODEV;
	}

	const char *iface_l2 = "OTHER";
	if (net_if_l2(iface) == &NET_L2_GET_NAME(PPP)) {
		iface_l2 = "PPP";
	} else if (net_if_l2(iface) == &NET_L2_GET_NAME(ETHERNET)) {
		iface_l2 = "ETH";
	}

	memset(&ctx, 0, sizeof(ctx));
	ctx.iface = iface;
	ctx.identifier = (uint16_t)(k_uptime_get_32() & 0xFFFF);
	ctx.expected_identifier = ctx.identifier;
	ctx.timeout_ms = req->timeout_ms;
	ctx.payload_size = payload_size;
	ctx.progress_cb = progress_cb;
	ctx.progress_user_data = progress_user_data;
	ctx.dst_addr.sin_family = AF_INET;
	ctx.dst_addr.sin_addr = dst;
	k_sem_init(&ctx.reply_sem, 0, 1);

	ret = TB45_PING_NET_ICMP_INIT_CTX(&ctx.icmp_ctx, tb45_ping_reply_handler);
	if (ret < 0) {
		LOG_ERR("PING: failed to initialize ICMP context (%d)", ret);
		return ret;
	}

	if (ctx.progress_cb == NULL) {
		LOG_INF("PING: start host=%s count=%u timeout=%u ms iface=%s idx=%d",
			req->host, req->count, req->timeout_ms,
			iface_l2, net_if_get_by_iface(iface));
	}

	for (uint16_t i = 1U; i <= req->count; i++) {
		int64_t started_at_ms = k_uptime_get();
		ctx.expected_sequence = i;
		ctx.reply_received = false;
		memset(&ctx.reply_addr, 0, sizeof(ctx.reply_addr));
		k_sem_reset(&ctx.reply_sem);

		struct net_icmp_ping_params params = {
			.identifier = ctx.identifier,
			.sequence = i,
			.tc_tos = 0U,
			.priority = -1,
			.data = NULL,
			.data_size = ctx.payload_size,
		};

		ret = net_icmp_send_echo_request(&ctx.icmp_ctx, ctx.iface,
							 (struct sockaddr *)&ctx.dst_addr,
							 &params, &ctx);
		if (ret < 0) {
			LOG_WRN("PING: failed to send icmp_seq=%u (%d)", i, ret);
			tb45_ping_report_progress(req, &ctx, i, 0U, false, ret);
			continue;
		}

		ctx.sent++;
		bool got_reply = false;
		int64_t deadline_ms = started_at_ms + ctx.timeout_ms;

		while (k_uptime_get() < deadline_ms) {
			int64_t now_ms = k_uptime_get();
			int32_t remaining_ms = (int32_t)(deadline_ms - now_ms);
			if (remaining_ms <= 0) {
				break;
			}

			int32_t wait_ms = MIN(remaining_ms, TB45_PING_WAIT_POLL_MS);
			if (k_sem_take(&ctx.reply_sem, K_MSEC(wait_ms)) == 0) {
				if (ctx.reply_received) {
					got_reply = true;
					break;
				}
			}
		}

		if (got_reply) {
			ctx.received++;
			uint32_t elapsed_ms = (uint32_t)(k_uptime_get() - started_at_ms);
			tb45_ping_report_progress(req, &ctx, i, elapsed_ms, true, 0);
		} else {
			LOG_WRN("PING: timeout icmp_seq=%u", i);
			tb45_ping_report_progress(req, &ctx, i, ctx.timeout_ms, false, -ETIMEDOUT);
		}

		if (i < req->count) {
			k_msleep(TB45_PING_BETWEEN_REQUESTS_MS);
		}
	}

	(void)net_icmp_cleanup_ctx(&ctx.icmp_ctx);

	if (ctx.progress_cb == NULL) {
		LOG_INF("PING: stats host=%s sent=%d received=%d lost=%d",
			req->host, ctx.sent, ctx.received, ctx.sent - ctx.received);
	}

	if (ctx.sent == 0) {
		return (ret < 0) ? ret : -EIO;
	}

	if (ctx.received == 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int tb45_ping_prepare_request(const char *host, uint16_t count, uint32_t timeout_ms,
				     struct tb45_async_job_ping_run *req)
{
	size_t host_len;

	if ((host == NULL) || (req == NULL)) {
		return -EINVAL;
	}

	host_len = strlen(host);
	if ((host_len == 0U) || (host_len > TB45_ASYNC_JOB_PING_HOST_MAX_LEN)) {
		return -EINVAL;
	}

	if (count == 0U) {
		count = CONFIG_APP_TB45_ASYNC_PING_DEFAULT_COUNT;
	}

	if ((count == 0U) || (count > CONFIG_APP_TB45_ASYNC_PING_MAX_COUNT)) {
		return -EINVAL;
	}

	if (timeout_ms == 0U) {
		timeout_ms = CONFIG_APP_TB45_ASYNC_PING_DEFAULT_TIMEOUT_MS;
	}

	if ((timeout_ms < CONFIG_APP_TB45_ASYNC_PING_MIN_TIMEOUT_MS) ||
	    (timeout_ms > CONFIG_APP_TB45_ASYNC_PING_MAX_TIMEOUT_MS)) {
		return -EINVAL;
	}

	memcpy(req->host, host, host_len + 1U);
	req->count = count;
	req->timeout_ms = timeout_ms;
	req->completion_ctx = NULL;
	return 0;
}

static int tb45_ping_async_dispatch(const struct tb45_async_job *job)
{
	if ((job == NULL) || (job->type != TB45_ASYNC_JOB_TYPE_PING_RUN)) {
		return -EINVAL;
	}

	int ret = tb45_ping_run_internal(&job->payload.ping_run,
				      TB45_PING_DEFAULT_PAYLOAD_SIZE,
				      NULL, NULL);

	struct tb45_ping_wait_ctx *wait_ctx =
		(struct tb45_ping_wait_ctx *)job->payload.ping_run.completion_ctx;
	if (wait_ctx != NULL) {
		wait_ctx->result = ret;
		k_sem_give(&wait_ctx->done);
	}

	return ret;
}

static int tb45_ping_async_register_handler(void)
{
	int ret = tb45_async_job_register_handler(TB45_ASYNC_JOB_TYPE_PING_RUN,
						  tb45_ping_async_dispatch);
	if (ret < 0) {
		LOG_ERR("PING register failed (%d)", ret);
	}

	return ret;
}

SYS_INIT(tb45_ping_async_register_handler, POST_KERNEL, 99);

static int32_t tb45_ping_async_wait_timeout_ms(const struct tb45_async_job_ping_run *req)
{
	if (req == NULL) {
		return 1000;
	}

	uint64_t per_request_ms = (uint64_t)req->timeout_ms + TB45_PING_WAIT_POLL_MS + 200U;
	uint64_t one_attempt_ms = (uint64_t)req->count * per_request_ms;
	if (req->count > 1U) {
		one_attempt_ms += ((uint64_t)req->count - 1U) * TB45_PING_BETWEEN_REQUESTS_MS;
	}

	uint64_t attempts = (uint64_t)1U + CONFIG_APP_TB45_ASYNC_PING_JOB_EXTRA_RETRIES;
	uint64_t total_ms = one_attempt_ms * attempts;
	if (attempts > 1U) {
		total_ms += (attempts - 1U) * CONFIG_APP_TB45_ASYNC_PING_JOB_RETRY_DELAY_MS;
	}
	total_ms += 2000U;

	if (total_ms > 0x7fffffffULL) {
		total_ms = 0x7fffffffULL;
	}
	if (total_ms < 1000U) {
		total_ms = 1000U;
	}

	return (int32_t)total_ms;
}

int tb45_ping_enqueue(const char *host, uint16_t count, uint32_t timeout_ms)
{
	struct tb45_async_job job = {
		.type = TB45_ASYNC_JOB_TYPE_PING_RUN,
	};
	int ret = tb45_ping_prepare_request(host, count, timeout_ms, &job.payload.ping_run);
	if (ret < 0) {
		return ret;
	}

	return tb45_async_job_enqueue(&job);
}

int tb45_ping_enqueue_wait(const char *host, uint16_t count, uint32_t timeout_ms)
{
	struct tb45_async_job job = {
		.type = TB45_ASYNC_JOB_TYPE_PING_RUN,
	};
	struct tb45_ping_wait_ctx wait_ctx;
	k_sem_init(&wait_ctx.done, 0, 1);
	wait_ctx.result = -EIO;

	int ret = tb45_ping_prepare_request(host, count, timeout_ms, &job.payload.ping_run);
	if (ret < 0) {
		return ret;
	}

	job.payload.ping_run.completion_ctx = &wait_ctx;

	ret = tb45_async_job_enqueue(&job);
	if (ret < 0) {
		return ret;
	}

	ret = k_sem_take(&wait_ctx.done, K_MSEC(tb45_ping_async_wait_timeout_ms(&job.payload.ping_run)));
	if (ret < 0) {
		return ret;
	}

	return wait_ctx.result;
}

int tb45_ping_run(const char *host, uint16_t count, uint32_t timeout_ms)
{
	return tb45_ping_run_ex(host, count, timeout_ms,
				TB45_PING_DEFAULT_PAYLOAD_SIZE,
				NULL, NULL);
}

int tb45_ping_run_ex(const char *host, uint16_t count, uint32_t timeout_ms,
		     size_t payload_size, tb45_ping_progress_cb_t progress_cb,
		     void *progress_user_data)
{
	struct tb45_async_job_ping_run req;

	if (payload_size > TB45_PING_MAX_PAYLOAD_SIZE) {
		return -EINVAL;
	}

	int ret = tb45_ping_prepare_request(host, count, timeout_ms, &req);
	if (ret < 0) {
		return ret;
	}

	return tb45_ping_run_internal(&req, payload_size, progress_cb, progress_user_data);
}
