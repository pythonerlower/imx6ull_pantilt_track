/*
 * 板端视频链路：V4L2 采集 → 定长槽环形缓冲 → UDP 分包发送。
 *
 * 两条硬约束决定了本文件的形状：
 *
 * 1. rules §4「采集不得因发送而阻塞」。采集线程只做 DQBUF→rb_push→QBUF，
 *    发送线程从环缓冲取帧；发送端用**阻塞 socket**，于是网络压力表现为
 *    环缓冲的 drop_overwrite 计数上升（可观测），而不是采集循环变慢。
 *    这一点与"发送非阻塞 + 自行丢包"等效，但少一份状态机，且计数器更诚实。
 *
 * 2. rules §8「摄像头偶发故障：有序归还模块退出，再重载重试」。
 *    本程序遇到摄像头故障时只做有序归还并**以退出码 3 报告**，
 *    重载与重试交给 run_video_link.sh —— 因为重载内核模块不是采集程序该干的事。
 *
 * 3. `--fps` 是**输出目标帧率**，不是传感器请求（2026-10-07 定案 A+B）。
 *    OV5640 驱动不受理 VIDIOC_S_PARM 改帧率（请求 1/5，写回 1/15），
 *    所以降帧率只能自己在采集之后丢：DQBUF 判一次，丢掉的帧
 *    **既不 memcpy 也不进环缓冲、更不上网**，CPU 和链路一起省下来。
 *    out_fps >= 传感器帧率时每帧都放行，行为与不节流逐字节相同——
 *    默认值（15）因此不会动到 3B 已经验收过的那条路径。
 */
/*
 * 必须放在所有 include 之前：-std=c11 是严格 ISO 模式，不声明这个宏的话
 * glibc 不会给出 struct sigaction / sigaction()，编译会在 sigaction 处
 * 报"类型不完全"和"隐式声明"。pantilt_test.c 用的是同一个写法。
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "pantilt_proto.h"
#include "ringbuf.h"

/* 退出码：3 专门表示"摄像头故障，值得重载重试"，与参数/网络错误区分开。 */
#define EXIT_OK 0
#define EXIT_USAGE 1
#define EXIT_NET 2
#define EXIT_CAMERA 3

#define MAX_BUF 8
#define DEFAULT_DEV "/dev/video1"
#define DEFAULT_SLOTS RB_DEFAULT_SLOTS

struct cfg {
	const char *dev;
	const char *dst_host;
	uint16_t dst_port;
	unsigned width, height;
	unsigned out_fps; /* --fps：输出目标帧率，见文件头 §3 */
	int slots;
	int stats_ms;
	int dump_frames;
	const char *dump_dir;
};

struct v4l2_buf_mem {
	void *start;
	size_t length;
};

struct app {
	struct cfg cfg;
	int vfd;
	struct v4l2_buf_mem bufs[MAX_BUF];
	int nbuf;
	unsigned bytesperline;
	unsigned sizeimage;
	unsigned sensor_fps;  /* 传感器实际输出帧率（S_PARM 回读，G_PARM 兜底） */
	int sensor_fps_checked; /* 已用实测采集率核对过一次传感器帧率 */
	unsigned pace_acc;    /* 输出节流累加器，见 pace_allow() */
	uint64_t cap_frames;  /* DQBUF 到的总帧数（含被节流丢掉的） */
	uint64_t drop_throttle; /* 被节流丢掉的帧数（未进环缓冲，也未曾上网） */
	struct ringbuf rb;
	uint8_t *frame; /* 采集侧暂存，push 前用 */
	volatile sig_atomic_t stop;
	/* 发送侧统计（仅发送线程写，退出时读） */
	uint64_t send_pkts;
	uint64_t send_errs;
	uint64_t send_frame_fail;
};

static struct app g;

static uint64_t now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;

	do {
		r = ioctl(fd, req, arg);
	} while (r == -1 && errno == EINTR);
	return r;
}

static void on_signal(int sig)
{
	(void)sig;
	g.stop = 1;
}

static int wait_frame(int fd, int timeout_ms)
{
	struct timeval tv = { .tv_sec = timeout_ms / 1000,
			      .tv_usec = (timeout_ms % 1000) * 1000 };
	fd_set fds;
	int r;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	do {
		r = select(fd + 1, &fds, NULL, NULL, &tv);
	} while (r == -1 && errno == EINTR);
	return r;
}

static void usage(const char *p)
{
	fprintf(stderr,
		"用法: %s [选项]\n"
		"  --dev PATH       采集设备（默认 %s）\n"
		"  --dst HOST:PORT  视频目的地址（默认 192.168.137.100:%u）\n"
		"  --width/--height 分辨率（默认 320x240）\n"
		"  --fps N          输出目标帧率（默认 15）。低于传感器帧率时在\n"
		"                   DQBUF 之后按整数分频丢帧，被丢的帧不进环缓冲、\n"
		"                   不上网；高于传感器帧率时按传感器帧率输出。\n"
		"  --slots N        环缓冲槽数 %d..%d（默认 %d）\n"
		"  --stats-ms N     统计打印间隔（默认 1000，0 关闭）\n"
		"  --dump-frames N  额外把前 N 帧原始 YUYV 落盘（3C 离线调 HSV 用）\n"
		"  --dump-dir DIR   落盘目录（默认 ./frames）\n"
		"退出码: 0 正常 / 1 参数 / 2 网络 / 3 摄像头故障（可重载重试）\n",
		p, DEFAULT_DEV, PTP_VIDEO_PORT, RB_MIN_SLOTS, RB_MAX_SLOTS,
		DEFAULT_SLOTS);
}

static int parse_dst(const char *s, char *host, size_t hostsz, uint16_t *port)
{
	const char *colon = strrchr(s, ':');

	if (!colon || colon == s || (size_t)(colon - s) >= hostsz)
		return -1;
	memcpy(host, s, (size_t)(colon - s));
	host[colon - s] = '\0';
	*port = (uint16_t)atoi(colon + 1);
	return *port ? 0 : -1;
}

/*
 * 从 V4L2 回读的 timeperframe 里取帧率；不可信就返回 0。
 *
 * 为什么要有这道闸门：节流判据是 `out_fps < sensor_fps`。如果驱动回了一个
 * 垃圾值（0/0、或者算出个上万），"传感器帧率"就会被当成一个很大的数，
 * 节流器于是把几乎每一帧都丢掉——症状是"视频凭空掉帧"，看起来像网络问题，
 * 根本查不到这里。所以只接受 1..1000 这个区间，其余一律当未知，
 * 返回值 0 让调用方保留上一次的判断（最终退回不节流）。
 */
static unsigned parm_fps(const struct v4l2_streamparm *p)
{
	unsigned num = p->parm.capture.timeperframe.numerator;
	unsigned den = p->parm.capture.timeperframe.denominator;
	unsigned fps;

	if (num == 0 || den == 0)
		return 0;
	fps = den / num;
	return (fps >= 1 && fps <= 1000) ? fps : 0;
}

static int open_capture(struct app *a)
{
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_requestbuffers req;
	struct v4l2_streamparm parm;
	enum v4l2_buf_type type;
	int i;

	a->vfd = open(a->cfg.dev, O_RDWR);
	if (a->vfd < 0) {
		fprintf(stderr, "[ERROR] 打开 %s: %s\n", a->cfg.dev,
			strerror(errno));
		return -1;
	}
	if (xioctl(a->vfd, VIDIOC_QUERYCAP, &cap) < 0) {
		perror("[ERROR] VIDIOC_QUERYCAP");
		return -1;
	}
	{
		unsigned caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
					? cap.device_caps
					: cap.capabilities;

		if (!(caps & V4L2_CAP_VIDEO_CAPTURE) ||
		    !(caps & V4L2_CAP_STREAMING)) {
			fprintf(stderr,
				"[ERROR] %s 不支持 CAPTURE+STREAMING（caps=0x%x）\n",
				a->cfg.dev, caps);
			return -1;
		}
	}

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = a->cfg.width;
	fmt.fmt.pix.height = a->cfg.height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(a->vfd, VIDIOC_S_FMT, &fmt) < 0) {
		perror("[ERROR] VIDIOC_S_FMT");
		return -1;
	}
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
		fprintf(stderr, "[ERROR] 驱动不接受 YUYV，回给 0x%08x\n",
			fmt.fmt.pix.pixelformat);
		return -1;
	}
	a->bytesperline = fmt.fmt.pix.bytesperline;
	a->sizeimage = fmt.fmt.pix.sizeimage;

	/*
	 * 先 G_PARM 问出传感器**当前**的帧率，再 S_PARM 请求我们要的。
	 *
	 * 顺序很要紧：直接 memset 出一个空 parm 再 S_PARM 的话，一旦这次请求
	 * 被驱动拒掉（或干脆没写回），struct 里留着的就是我们自己填进去的
	 * 那个值——"传感器帧率"于是变成假数。3B 那次 `--fps 5` 仍然打印
	 * `帧率=15`（且没有 WARN），病因正是不加区分地信任了这里的回读。
	 */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(a->vfd, VIDIOC_G_PARM, &parm) < 0)
		perror("[WARN] VIDIOC_G_PARM 失败，传感器帧率未知");
	if (parm_fps(&parm))
		a->sensor_fps = parm_fps(&parm);

	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = a->cfg.out_fps;
	if (xioctl(a->vfd, VIDIOC_S_PARM, &parm) < 0) {
		/*
		 * 请求被拒时**绝不能**回读这一段：struct 里留着的是我们刚填进去的
		 * 请求值，不是驱动的回答。2026-10-07 实测踩到过——`--fps 20` 被
		 * OV5640 以 EINVAL 拒掉（它只接受离散档位），回读拿到 20，
		 * 于是 INFO 印"传感器 20 fps"而实际采集 30 fps；更糟的是节流判据
		 * `out_fps >= sensor_fps` 恰好判成"不节流"，整条链一路错得很安静。
		 * 保留 G_PARM 那次问到的值即可。
		 */
		perror("[WARN] VIDIOC_S_PARM 失败，沿用驱动当前帧率");
	} else if (parm_fps(&parm)) {
		a->sensor_fps = parm_fps(&parm);
	}
	/* 驱动一个字都没回：当作不节流，行为与旧版完全一致。 */
	if (a->sensor_fps == 0)
		a->sensor_fps = a->cfg.out_fps;

	printf("[INFO] %s %s: %ux%u YUYV bytesperline=%u sizeimage=%u\n",
	       a->cfg.dev, cap.card, fmt.fmt.pix.width, fmt.fmt.pix.height,
	       a->bytesperline, a->sizeimage);
	if (a->sensor_fps != a->cfg.out_fps)
		printf("[WARN] 传感器帧率请求 %u fps 未被受理，驱动回 %u fps —— "
		       "板端传感器帧率改不了，--fps 改由采集后输出节流实现\n",
		       a->cfg.out_fps, a->sensor_fps);
	if (a->cfg.out_fps > a->sensor_fps)
		printf("[WARN] 输出目标 %u fps 高于传感器 %u fps，按 %u fps 输出"
		       "（不能凭空造帧）\n", a->cfg.out_fps, a->sensor_fps,
		       a->sensor_fps);
	printf("[INFO] 传感器 %u fps，输出目标 %u fps%s\n", a->sensor_fps,
	       a->cfg.out_fps,
	       a->cfg.out_fps < a->sensor_fps ? "（按整数分频节流）" : "（不节流）");
	if (a->sizeimage > a->bytesperline * fmt.fmt.pix.height)
		printf("[WARN] sizeimage %u 大于 bytesperline*height %u，按 sizeimage 取\n",
		       a->sizeimage, a->bytesperline * fmt.fmt.pix.height);

	memset(&req, 0, sizeof(req));
	req.count = MAX_BUF;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(a->vfd, VIDIOC_REQBUFS, &req) < 0) {
		perror("[ERROR] VIDIOC_REQBUFS");
		return -1;
	}
	if (req.count < 2) {
		fprintf(stderr, "[ERROR] 驱动只给了 %u 个缓冲\n", req.count);
		return -1;
	}
	a->nbuf = (int)req.count;

	for (i = 0; i < a->nbuf; i++) {
		struct v4l2_buffer buf;

		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = (unsigned)i;
		if (xioctl(a->vfd, VIDIOC_QUERYBUF, &buf) < 0) {
			perror("[ERROR] VIDIOC_QUERYBUF");
			return -1;
		}
		a->bufs[i].length = buf.length;
		a->bufs[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
					MAP_SHARED, a->vfd, buf.m.offset);
		if (a->bufs[i].start == MAP_FAILED) {
			a->bufs[i].start = NULL;
			perror("[ERROR] mmap");
			return -1;
		}
	}
	for (i = 0; i < a->nbuf; i++) {
		struct v4l2_buffer buf;

		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = (unsigned)i;
		if (xioctl(a->vfd, VIDIOC_QBUF, &buf) < 0) {
			perror("[ERROR] VIDIOC_QBUF");
			return -1;
		}
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(a->vfd, VIDIOC_STREAMON, &type) < 0) {
		/* 既有故障模式之一：重载后首次 STREAMON 报 Timer expired / EPERM。 */
		perror("[ERROR] VIDIOC_STREAMON");
		return -1;
	}
	return 0;
}

static void close_capture(struct app *a)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int i;

	if (a->vfd < 0)
		return;
	/* 有序归还：先停流，再解映射，最后关设备（rules §8）。 */
	xioctl(a->vfd, VIDIOC_STREAMOFF, &type);
	for (i = 0; i < a->nbuf; i++) {
		if (a->bufs[i].start)
			munmap(a->bufs[i].start, a->bufs[i].length);
		a->bufs[i].start = NULL;
	}
	close(a->vfd);
	a->vfd = -1;
}

struct sender_arg {
	struct app *a;
};

static void *sender_thread(void *arg)
{
	struct app *a = ((struct sender_arg *)arg)->a;
	struct sockaddr_in dst;
	uint8_t *pkt;
	const uint8_t *frame;
	int fd;

	pkt = malloc(PTP_VIDEO_DGRAM);
	frame = malloc(a->sizeimage);
	if (!pkt || !frame) {
		fprintf(stderr, "[ERROR] 发送线程内存分配失败\n");
		free(pkt);
		free((void *)frame);
		return NULL;
	}

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		perror("[ERROR] socket");
		goto out_buf;
	}
	{
		int sndbuf = 1024 * 1024;
		struct timeval tv = { .tv_sec = 0, .tv_usec = 200 * 1000 };

		setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
		/*
		 * 发送超时：网络断掉时不让发送线程永远挂在 sendto 里，
		 * 否则退出信号也叫不醒它。超时后照常丢整帧并继续。
		 */
		setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
	}
	memset(&dst, 0, sizeof(dst));
	dst.sin_family = AF_INET;
	dst.sin_port = htons(a->cfg.dst_port);
	if (inet_pton(AF_INET, a->cfg.dst_host, &dst.sin_addr) != 1) {
		fprintf(stderr, "[ERROR] 目的地址 %s 不是合法 IPv4\n",
			a->cfg.dst_host);
		close(fd);
		goto out_buf;
	}
	printf("[INFO] 视频发往 %s:%u（%d 字节/包，含 %d 字节头）\n",
	       a->cfg.dst_host, a->cfg.dst_port, PTP_VIDEO_DGRAM,
	       PTP_VIDEO_HDR_SIZE);

	while (!a->stop) {
		struct rb_meta m;
		struct ptp_video_meta vm;
		uint32_t chunk_total, idx;
		bool failed = false;

		if (rb_pop(&a->rb, (void *)frame, &m) < 0)
			break; /* 已停止且队列空 */

		vm.frame_id = m.frame_id;
		vm.timestamp_us = m.timestamp_us;
		vm.width = m.width;
		vm.height = m.height;
		vm.format = m.format;
		vm.flags = (m.len < PTP_VIDEO_PAYLOAD) ? PTP_VIDEO_FLAG_SINGLE : 0u;

		chunk_total = (uint32_t)((m.len + PTP_VIDEO_PAYLOAD - 1) /
					 PTP_VIDEO_PAYLOAD);
		for (idx = 0; idx < chunk_total; idx++) {
			size_t off = (size_t)idx * PTP_VIDEO_PAYLOAD;
			size_t len = m.len - off;

			if (len > PTP_VIDEO_PAYLOAD)
				len = PTP_VIDEO_PAYLOAD;
			memcpy(pkt + PTP_VIDEO_HDR_SIZE, frame + off, len);
			/* 头字段由 pantilt_proto.h 的权威编码器写（与 PC 侧对拍）。 */
			ptp_video_hdr_encode(&vm, (uint16_t)idx,
					     (uint16_t)chunk_total,
					     pkt + PTP_VIDEO_HDR_SIZE,
					     (uint16_t)len, pkt);
			if (sendto(fd, pkt, PTP_VIDEO_HDR_SIZE + len, 0,
				   (struct sockaddr *)&dst, sizeof(dst)) < 0) {
				a->send_errs++;
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					/*
					 * 发送缓冲满：丢掉本帧剩余分包。
					 * 半帧送到对端毫无价值（缺片整帧作废），
					 * 继续硬发只会继续堵。
					 */
					failed = true;
					break;
				}
				if (errno == EINTR)
					continue;
				failed = true;
				break;
			}
			a->send_pkts++;
		}
		if (failed)
			a->send_frame_fail++;
	}

	close(fd);
out_buf:
	free(pkt);
	free((void *)frame);
	return NULL;
}

static void print_stats(struct app *a, uint64_t t_start)
{
	struct rb_stats st;
	uint64_t dt = now_us() - t_start;

	rb_get_stats(&a->rb, &st);
	if (dt == 0)
		return;
	/*
	 * 一次性对账：跑够 3 秒后，拿**实测**采集率核对驱动报的传感器帧率。
	 * 节流比例完全建立在那一个数上，它要是不对，输出帧率就会静悄悄地
	 * 偏掉（差得越多越看不出来，因为画面本身没有问题）。
	 * 采集率是 DQBUF 的实数数出来的，不依赖驱动是否诚实。
	 */
	if (!a->sensor_fps_checked && dt >= 3000000ull) {
		double got = (double)a->cap_frames * 1e6 / (double)dt;

		a->sensor_fps_checked = 1;
		if (got > a->sensor_fps * 1.2 || got < a->sensor_fps * 0.8)
			printf("[WARN] 实测采集 %.1f fps 与驱动的传感器 %u fps 不符，"
			       "节流比例可能有偏差（以实测为准）\n",
			       got, a->sensor_fps);
	}
	/*
	 * 口径（2026-10-07 起，`--fps` 变成输出目标帧率之后）：
	 *   采集 = 节流丢 + 入环          DQBUF 到的每一帧只归其中一处
	 *   入环 = 出环 + 覆盖丢 + 在环  环缓冲自己的账
	 * 左边一组是"板端没发"，右边一组是"PC 没跟上"，别混。
	 */
	printf("[stats] %.1fs 采集=%llu(%.1f fps) 节流丢=%llu "
	       "入环=%llu(%.1f fps) 出环=%llu(%.1f fps) 覆盖丢=%llu "
	       "深=%d 高水位=%d 拷贝=%.2fms/帧 等待=%.0fms "
	       "发包=%llu 发包错=%llu 丢帧(发送)=%llu\n",
	       dt / 1e6, (unsigned long long)a->cap_frames,
	       a->cap_frames * 1e6 / dt,
	       (unsigned long long)a->drop_throttle,
	       (unsigned long long)st.frames_in, st.frames_in * 1e6 / dt,
	       (unsigned long long)st.frames_out, st.frames_out * 1e6 / dt,
	       (unsigned long long)st.drop_overwrite, st.count,
	       st.high_watermark,
	       st.frames_out ? st.copy_ns_total / 1e6 / st.frames_out : 0.0,
	       st.wait_ns_total / 1e6, (unsigned long long)a->send_pkts,
	       (unsigned long long)a->send_errs,
	       (unsigned long long)a->send_frame_fail);
}

/*
 * 把一帧原始 YUYV 落盘。给 3C 用：HSV 阈值要在真实帧上标定，
 * 而不能靠"对着 320x240 想象"。文件名带上 stride，避免离线解析时
 * 误以为行宽就是 width*2（bytesperline 可能被对齐填充）。
 */
static int dump_frame(const struct app *a, const struct rb_meta *m,
		      const uint8_t *data)
{
	char path[512];
	FILE *fp;
	unsigned w = m->width ? m->width : a->cfg.width;
	unsigned h = m->height ? m->height : a->cfg.height;
	unsigned stride = h ? (unsigned)(m->len / h) : 0;

	snprintf(path, sizeof(path), "%s/frame_%05u_%ux%u_stride%u.yuyv",
		 a->cfg.dump_dir, m->frame_id, w, h, stride);
	fp = fopen(path, "wb");
	if (!fp) {
		fprintf(stderr, "[WARN] 落盘 %s 失败: %s\n", path,
			strerror(errno));
		return -1;
	}
	fwrite(data, 1, m->len, fp);
	fclose(fp);
	return 0;
}

/*
 * 输出节流：决定这一帧要不要进环缓冲。false = 丢掉。
 *
 * 用整数累加分频（Bresenham），而不是"距上次推送够不够 1/fps 秒"：
 * 后者在输出目标≈传感器帧率时会**拍频**——帧的到达时刻在周期上下抖动，
 * 判据大约有一半的帧会因为差几毫秒而不成立，输出反倒掉到一半
 * （15 fps 进、目标 15 fps 出，实测会变成 7.5）。累加器没有这个毛病，
 * 平均速率精确，抖动只有一个传感器周期。
 *
 * out_fps >= sensor_fps 时每帧都放行：默认值（15 vs 15）因此是**空操作**，
 * 3B 已验收的那条路径一个字节都不动。
 *
 * 返回 false 时调用方**仍然必须 QBUF**——被丢的帧也是 DQBUF 出来的，
 * 不还回去会把缓冲池抽干，采集直接停摆。
 */
static bool pace_allow(struct app *a)
{
	a->cap_frames++;
	if (a->sensor_fps == 0 || a->cfg.out_fps >= a->sensor_fps)
		return true;
	a->pace_acc += a->cfg.out_fps;
	if (a->pace_acc < a->sensor_fps) {
		a->drop_throttle++;
		return false;
	}
	a->pace_acc -= a->sensor_fps;
	return true;
}

int main(int argc, char **argv)
{
	static const struct option opts[] = {
		{ "dev", required_argument, NULL, 'd' },
		{ "dst", required_argument, NULL, 'o' },
		{ "width", required_argument, NULL, 'w' },
		{ "height", required_argument, NULL, 'h' },
		{ "fps", required_argument, NULL, 'f' },
		{ "slots", required_argument, NULL, 's' },
		{ "stats-ms", required_argument, NULL, 'i' },
		{ "dump-frames", required_argument, NULL, 'n' },
		{ "dump-dir", required_argument, NULL, 'D' },
		{ "help", no_argument, NULL, 'H' },
		{ NULL, 0, NULL, 0 }
	};
	char host[64] = "192.168.137.100";
	uint16_t port = PTP_VIDEO_PORT;
	struct sender_arg sarg;
	pthread_t sth;
	uint64_t t_start, t_last;
	int rc = EXIT_OK, opt, dumped = 0;
	struct sigaction sa = { .sa_handler = on_signal };

	/*
	 * stdout 重定向到文件时默认是**全缓冲**，而 stderr 从不缓冲。两者混进
	 * 同一份日志就会出现"WARN 落在它上面那行 INFO 之上"的错序（2026-10-07
	 * 的 fps20 日志就是这样：S_PARM 的 WARN 跑到了 INFO 前面，看起来像是
	 * 上一轮的遗留）。改成行缓冲，让日志按真实先后落盘。
	 */
	setvbuf(stdout, NULL, _IOLBF, 0);

	memset(&g, 0, sizeof(g));
	g.vfd = -1;
	g.cfg.dev = DEFAULT_DEV;
	g.cfg.width = 320;
	g.cfg.height = 240;
	g.cfg.out_fps = 15;
	g.cfg.slots = DEFAULT_SLOTS;
	g.cfg.stats_ms = 1000;
	g.cfg.dump_dir = "frames";
	g.cfg.dump_frames = 0;

	while ((opt = getopt_long(argc, argv, "", opts, NULL)) != -1) {
		switch (opt) {
		case 'd':
			g.cfg.dev = optarg;
			break;
		case 'o':
			if (parse_dst(optarg, host, sizeof(host), &port) < 0) {
				fprintf(stderr, "[ERROR] --dst 需为 HOST:PORT\n");
				return EXIT_USAGE;
			}
			break;
		case 'w':
			g.cfg.width = (unsigned)atoi(optarg);
			break;
		case 'h':
			g.cfg.height = (unsigned)atoi(optarg);
			break;
		case 'f':
			g.cfg.out_fps = (unsigned)atoi(optarg);
			break;
		case 's':
			g.cfg.slots = atoi(optarg);
			break;
		case 'i':
			g.cfg.stats_ms = atoi(optarg);
			break;
		case 'n':
			g.cfg.dump_frames = atoi(optarg);
			break;
		case 'D':
			g.cfg.dump_dir = optarg;
			break;
		default:
			usage(argv[0]);
			return opt == 'H' ? EXIT_OK : EXIT_USAGE;
		}
	}
	g.cfg.dst_host = host;
	g.cfg.dst_port = port;

	if (g.cfg.width == 0 || g.cfg.height == 0 || g.cfg.out_fps == 0) {
		fprintf(stderr, "[ERROR] width/height/fps 必须为正\n");
		return EXIT_USAGE;
	}

	if (open_capture(&g) < 0) {
		close_capture(&g);
		return EXIT_CAMERA;
	}

	/* 槽容量按驱动实际给的 sizeimage 分配，而不是按公式算的 w*h*2。 */
	if (rb_init(&g.rb, g.cfg.slots, g.sizeimage) <= 0) {
		fprintf(stderr, "[ERROR] 环缓冲初始化失败\n");
		close_capture(&g);
		return EXIT_USAGE;
	}
	g.frame = malloc(g.sizeimage);
	if (!g.frame) {
		fprintf(stderr, "[ERROR] 帧暂存分配失败\n");
		rb_destroy(&g.rb);
		close_capture(&g);
		return EXIT_USAGE;
	}

	if (g.cfg.dump_frames > 0 &&
	    mkdir(g.cfg.dump_dir, 0755) < 0 && errno != EEXIST)
		fprintf(stderr, "[WARN] 无法创建 %s: %s\n", g.cfg.dump_dir,
			strerror(errno));

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	sarg.a = &g;
	if (pthread_create(&sth, NULL, sender_thread, &sarg) != 0) {
		fprintf(stderr, "[ERROR] 发送线程创建失败\n");
		free(g.frame);
		rb_destroy(&g.rb);
		close_capture(&g);
		return EXIT_USAGE;
	}

	t_start = t_last = now_us();
	printf("[INFO] 开始采集，Ctrl-C 停止\n");

	while (!g.stop) {
		struct v4l2_buffer buf;
		struct rb_meta m;
		int w;

		w = wait_frame(g.vfd, 500);
		if (w < 0) {
			perror("[ERROR] select");
			rc = EXIT_CAMERA;
			break;
		}
		if (w == 0)
			continue; /* 超时只是用来周期性检查退出标志 */

		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (xioctl(g.vfd, VIDIOC_DQBUF, &buf) < 0) {
			if (errno == EAGAIN)
				continue;
			perror("[ERROR] VIDIOC_DQBUF");
			rc = EXIT_CAMERA;
			break;
		}

		memset(&m, 0, sizeof(m));
		m.frame_id = (uint32_t)buf.sequence;
		m.timestamp_us = now_us();
		/* bytesperline 可能大于 width*2（对齐填充），单帧按 sizeimage 整体传。 */
		m.len = buf.bytesused ? buf.bytesused : g.sizeimage;
		if (m.len > g.sizeimage)
			m.len = g.sizeimage;
		m.width = (uint16_t)g.cfg.width;
		m.height = (uint16_t)g.cfg.height;
		m.format = PTP_FMT_YUYV;

		/*
		 * 节流判定放在 memcpy 之前：被丢掉的帧连拷贝都不做——
		 * 省下来的正是 ① 想要的 CPU 和链路。frame_id 仍然用
		 * buf.sequence（不是自增的发送序号），所以被丢的帧在 PC 侧
		 * 表现为 frame_id 跳号，是**看得见**的，不会被悄悄吞掉。
		 */
		if (pace_allow(&g)) {
			memcpy(g.frame, g.bufs[buf.index].start, m.len);
			rb_push(&g.rb, g.frame, m.len, &m);

			if (dumped < g.cfg.dump_frames &&
			    dump_frame(&g, &m, g.frame) == 0)
				dumped++;
		}

		if (xioctl(g.vfd, VIDIOC_QBUF, &buf) < 0) {
			perror("[ERROR] VIDIOC_QBUF");
			rc = EXIT_CAMERA;
			break;
		}

		if (g.cfg.stats_ms > 0 &&
		    now_us() - t_last >= (uint64_t)g.cfg.stats_ms * 1000) {
			print_stats(&g, t_start);
			t_last = now_us();
		}
	}

	if (g.stop)
		printf("\n[INFO] 收到停止信号，有序归还中\n");

	/* 有序归还：先停发送线程（它会自己排空队列），再停采集，最后释放。 */
	rb_stop(&g.rb);
	pthread_join(sth, NULL);
	close_capture(&g);
	print_stats(&g, t_start);
	free(g.frame);
	rb_destroy(&g.rb);
	printf("[INFO] 退出码 %d\n", rc);
	return rc;
}
