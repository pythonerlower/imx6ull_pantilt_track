/* ioctl_test_app.c - V4L2 ioctl test */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

static const char *err_str(int e)
{
    switch (e) {
    case 0: return "OK";
    case EBUSY: return "EBUSY";
    case EINVAL: return "EINVAL";
    case ENODEV: return "ENODEV";
    case ENOTTY: return "ENOTTY";
    default: return "ERR";
    }
}

static int test_v4l2(const char *dev)
{
    int fd, ret;
    struct v4l2_capability cap;
    struct v4l2_format fmt;

    printf("\n=== Test: %s ===\n", dev);

    fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        printf("open FAIL: %s\n", err_str(errno));
        return -1;
    }
    printf("open OK\n");

    memset(&cap, 0, sizeof(cap));
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        printf("QUERYCAP FAIL: %s\n", err_str(errno));
        close(fd);
        return -1;
    }
    printf("driver=%s card=%s\n", cap.driver, cap.card);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        printf("S_FMT FAIL: %s\n", err_str(errno));
        printf("Fallback: try 320x240\n");
        fmt.fmt.pix.width = 320;
        fmt.fmt.pix.height = 240;
        ret = ioctl(fd, VIDIOC_S_FMT, &fmt);
        if (ret < 0) {
            printf("S_FMT 320x240 FAIL: %s\n", err_str(errno));
            close(fd);
            return -1;
        }
    }
    printf("fmt=%ux%u\n", fmt.fmt.pix.width, fmt.fmt.pix.height);

    printf("Result: PASS\n");
    close(fd);
    return 0;
}

static void print_usage(const char *p)
{
    printf("Usage: %s [--v4l2] [--errors] [--help]\n", p);
}

static void show_errors(void)
{
    printf("\n=== V4L2 ioctl Error Handling ===\n");
    printf("EBUSY: Device busy - fix: STREAMOFF first\n");
    printf("EINVAL: Format unsupported - fix: ENUM_FMT first\n");
    printf("ENODEV: No device - check /dev/video*\n");
    printf("ENOTTY: Not V4L2 - wrong device node\n");
}

int main(int argc, char **argv)
{
    int i;
    int do_v4l2 = 0;
    int do_err = 0;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--v4l2") == 0) do_v4l2 = 1;
        if (strcmp(argv[i], "--errors") == 0) do_err = 1;
        if (strcmp(argv[i], "--help") == 0) { print_usage(argv[0]); return 0; }
    }

    if (!do_v4l2 && !do_err) do_v4l2 = 1;

    printf("=== V4L2 ioctl Test ===\n");

    if (do_err) show_errors();
    if (do_v4l2) {
        test_v4l2("/dev/video0");
        test_v4l2("/dev/video1");
    }

    printf("\nDone.\n");
    return 0;
}
