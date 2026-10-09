/*
 * 真实驱动：把 drv_ops 直译成 /dev/pantilt0 的 ioctl。
 *
 * 这里刻意不做任何值域/状态判断。判断一旦分散到两层，就会出现
 * "A 以为 B 会拦、B 以为 A 拦过了" 的空白区，而那正是越限动作会钻过去的地方。
 * 值域由驱动钳位（最终权威），协议层的提前拦截由 daemon.c 负责。
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drv.h"

static int real_set_target(struct drv *d, uint16_t pan, uint16_t tilt)
{
	struct pantilt_target t;

	t.tick[PANTILT_AXIS_PAN] = pan;
	t.tick[PANTILT_AXIS_TILT] = tilt;
	if (ioctl(*(int *)d->ctx, PANTILT_IOC_SET_TARGET, &t) < 0)
		return -errno;
	return 0;
}

static int real_home(struct drv *d)
{
	if (ioctl(*(int *)d->ctx, PANTILT_IOC_HOME) < 0)
		return -errno;
	return 0;
}

static int real_stop(struct drv *d)
{
	if (ioctl(*(int *)d->ctx, PANTILT_IOC_STOP) < 0)
		return -errno;
	return 0;
}

static int real_keepalive(struct drv *d)
{
	if (ioctl(*(int *)d->ctx, PANTILT_IOC_KEEPALIVE) < 0)
		return -errno;
	return 0;
}

static int real_get_state(struct drv *d, struct pantilt_state *out)
{
	if (ioctl(*(int *)d->ctx, PANTILT_IOC_GET_STATE, out) < 0)
		return -errno;
	return 0;
}

static void real_tick(struct drv *d)
{
	/* 真实爬升由驱动的 delayed_work 推进，用户态不需要也不该插手。 */
	(void)d;
}

static void real_close(struct drv *d)
{
	if (!d)
		return;
	if (d->ctx) {
		int fd = *(int *)d->ctx;

		if (fd >= 0)
			close(fd);
		free(d->ctx);
	}
	free(d);
}

static const struct drv_ops real_ops = {
	.set_target = real_set_target,
	.home = real_home,
	.stop = real_stop,
	.keepalive = real_keepalive,
	.get_state = real_get_state,
	.tick = real_tick,
	.close = real_close,
};

struct drv *drv_open_real(const char *path)
{
	struct drv *d;
	int *fdp;
	int fd;

	fd = open(path, O_RDWR);
	if (fd < 0)
		return NULL;

	fdp = malloc(sizeof(*fdp));
	d = malloc(sizeof(*d));
	if (!fdp || !d) {
		free(fdp);
		free(d);
		close(fd);
		errno = ENOMEM;
		return NULL;
	}
	*fdp = fd;
	d->ops = &real_ops;
	d->ctx = fdp;
	return d;
}
