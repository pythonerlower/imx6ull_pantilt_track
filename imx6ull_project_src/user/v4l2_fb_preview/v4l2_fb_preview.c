#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#define MAX_BUF 8

enum app_mode {
    MODE_ONCE = 0,
    MODE_STREAM,
    MODE_PREVIEW,
    MODE_PREVIEW_STATS,
};

struct app_cfg {
    const char *vdev;
    const char *fbdev;
    const char *dump_path;
    unsigned int width;
    unsigned int height;
    enum app_mode mode;
    int sec;
};

struct v4l2_buf_mem {
    void *start;
    size_t length;
};

struct fb_ctx {
    int fd;
    int enabled;
    unsigned char *mem;
    size_t mem_len;
    unsigned int width;
    unsigned int height;
    unsigned int bpp;
    unsigned int line_length;
};

static int xioctl(int fd, int req, void *arg)
{
    int ret;
    do {
        ret = ioctl(fd, req, arg);
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static unsigned long long now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000000000ULL + (unsigned long long)ts.tv_nsec;
}

static int clamp8(int v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return v;
}

static unsigned short yuv_to_rgb565(int y, int u, int v)
{
    int c = y - 16;
    int d = u - 128;
    int e = v - 128;
    int r = (298 * c + 409 * e + 128) >> 8;
    int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    int b = (298 * c + 516 * d + 128) >> 8;

    r = clamp8(r);
    g = clamp8(g);
    b = clamp8(b);

    return (unsigned short)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

static void yuyv_to_fb_rgb565_scaled(const unsigned char *src,
                                     unsigned int src_w,
                                     unsigned int src_h,
                                     unsigned int src_stride,
                                     struct fb_ctx *fb)
{
    unsigned int y, x;

    if (!fb->enabled || fb->bpp != 16)
        return;

    for (y = 0; y < fb->height; y++) {
        unsigned int sy = (unsigned int)(((unsigned long long)y * src_h) / fb->height);
        unsigned short *dst_row = (unsigned short *)(fb->mem + y * fb->line_length);
        const unsigned char *src_row = src + sy * src_stride;

        for (x = 0; x < fb->width; x++) {
            unsigned int sx = (unsigned int)(((unsigned long long)x * src_w) / fb->width);
            unsigned int pair = (sx / 2) * 4;
            int y0 = src_row[pair + 0];
            int u = src_row[pair + 1];
            int y1 = src_row[pair + 2];
            int v = src_row[pair + 3];
            int yy = (sx & 1) ? y1 : y0;
            dst_row[x] = yuv_to_rgb565(yy, u, v);
        }
    }
}

static int wait_frame(int vfd, int timeout_ms)
{
    fd_set rfds;
    struct timeval tv;

    FD_ZERO(&rfds);
    FD_SET(vfd, &rfds);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    return select(vfd + 1, &rfds, NULL, NULL, &tv);
}

static int parse_size(const char *s, unsigned int *w, unsigned int *h)
{
    char *x = strchr(s, 'x');
    if (!x)
        return -1;
    *w = (unsigned int)atoi(s);
    *h = (unsigned int)atoi(x + 1);
    return (*w && *h) ? 0 : -1;
}

static int parse_mode(const char *s, enum app_mode *mode)
{
    if (!strcmp(s, "once")) {
        *mode = MODE_ONCE;
        return 0;
    }
    if (!strcmp(s, "stream")) {
        *mode = MODE_STREAM;
        return 0;
    }
    if (!strcmp(s, "preview")) {
        *mode = MODE_PREVIEW;
        return 0;
    }
    if (!strcmp(s, "preview+stats")) {
        *mode = MODE_PREVIEW_STATS;
        return 0;
    }
    return -1;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [--dev /dev/video1] [--fb /dev/fb0] [--size WxH] "
            "[--mode once|stream|preview|preview+stats] [--sec N] [--dump /tmp/frame.raw]\n",
            prog);
}

static void enum_formats(int vfd)
{
    struct v4l2_fmtdesc fmtdesc;

    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("capture formats:\n");
    while (xioctl(vfd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        printf("  [%u] %c%c%c%c : %s\n",
               fmtdesc.index,
               fmtdesc.pixelformat & 0xff,
               (fmtdesc.pixelformat >> 8) & 0xff,
               (fmtdesc.pixelformat >> 16) & 0xff,
               (fmtdesc.pixelformat >> 24) & 0xff,
               fmtdesc.description);
        fmtdesc.index++;
    }
}

static int fb_open_and_map(struct fb_ctx *fb, const char *fbdev)
{
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    memset(fb, 0, sizeof(*fb));
    fb->fd = -1;

    fb->fd = open(fbdev, O_RDWR);
    if (fb->fd < 0) {
        perror("open fb");
        return -1;
    }
    if (xioctl(fb->fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("FBIOGET_FSCREENINFO");
        close(fb->fd);
        fb->fd = -1;
        return -1;
    }
    if (xioctl(fb->fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        perror("FBIOGET_VSCREENINFO");
        close(fb->fd);
        fb->fd = -1;
        return -1;
    }

    fb->width = vinfo.xres;
    fb->height = vinfo.yres;
    fb->bpp = vinfo.bits_per_pixel;
    fb->line_length = finfo.line_length;
    fb->mem_len = finfo.line_length * vinfo.yres;
    fb->mem = mmap(NULL, fb->mem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0);
    if (fb->mem == MAP_FAILED) {
        perror("mmap fb");
        close(fb->fd);
        fb->fd = -1;
        fb->mem = NULL;
        return -1;
    }

    fb->enabled = 1;
    printf("fb %ux%u %ubpp line=%u\n", fb->width, fb->height, fb->bpp, fb->line_length);
    return 0;
}

static void fb_close_unmap(struct fb_ctx *fb)
{
    if (fb->mem && fb->mem != MAP_FAILED)
        munmap(fb->mem, fb->mem_len);
    if (fb->fd >= 0)
        close(fb->fd);
    memset(fb, 0, sizeof(*fb));
    fb->fd = -1;
}

int main(int argc, char **argv)
{
    struct app_cfg cfg;
    struct fb_ctx fb;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buf_mem bufs[MAX_BUF];
    enum v4l2_buf_type type;

    int vfd = -1;
    int ret = 1;
    int nbuf = 0;
    int i;

    cfg.vdev = "/dev/video1";
    cfg.fbdev = "/dev/fb0";
    cfg.dump_path = NULL;
    cfg.width = 640;
    cfg.height = 480;
    cfg.mode = MODE_ONCE;
    cfg.sec = 5;

    for (i = 0; i < MAX_BUF; i++) {
        bufs[i].start = NULL;
        bufs[i].length = 0;
    }

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc) {
            cfg.vdev = argv[++i];
        } else if (!strcmp(argv[i], "--fb") && i + 1 < argc) {
            cfg.fbdev = argv[++i];
        } else if (!strcmp(argv[i], "--size") && i + 1 < argc) {
            if (parse_size(argv[++i], &cfg.width, &cfg.height) < 0) {
                fprintf(stderr, "invalid --size\n");
                return 1;
            }
        } else if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            if (parse_mode(argv[++i], &cfg.mode) < 0) {
                fprintf(stderr, "invalid --mode\n");
                return 1;
            }
        } else if (!strcmp(argv[i], "--sec") && i + 1 < argc) {
            cfg.sec = atoi(argv[++i]);
            if (cfg.sec <= 0)
                cfg.sec = 1;
        } else if (!strcmp(argv[i], "--dump") && i + 1 < argc) {
            cfg.dump_path = argv[++i];
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    fb.fd = -1;
    fb.enabled = 0;
    fb.mem = NULL;

    vfd = open(cfg.vdev, O_RDWR);
    if (vfd < 0) {
        perror("open video");
        return 1;
    }

    if (xioctl(vfd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        goto out;
    }

    printf("video card=%s driver=%s bus=%s\n", cap.card, cap.driver, cap.bus_info);
    printf("caps=0x%x device_caps=0x%x\n", cap.capabilities, cap.device_caps);

    {
        unsigned int caps = cap.capabilities;
        if (caps & V4L2_CAP_DEVICE_CAPS)
            caps = cap.device_caps;

        if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "device is not VIDEO_CAPTURE\n");
            goto out;
        }
        if (!(caps & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "device does not support STREAMING\n");
            goto out;
        }
    }

    if (cfg.mode == MODE_PREVIEW || cfg.mode == MODE_PREVIEW_STATS) {
        if (fb_open_and_map(&fb, cfg.fbdev) < 0)
            goto out;
        if (fb.bpp != 16) {
            fprintf(stderr, "unsupported fb bpp=%u, expect 16\n", fb.bpp);
            goto out;
        }
    }

    enum_formats(vfd);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = cfg.width;
    fmt.fmt.pix.height = cfg.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(vfd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        goto out;
    }

    printf("capture fmt: %ux%u %c%c%c%c bytesperline=%u sizeimage=%u\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height,
           fmt.fmt.pix.pixelformat & 0xff,
           (fmt.fmt.pix.pixelformat >> 8) & 0xff,
           (fmt.fmt.pix.pixelformat >> 16) & 0xff,
           (fmt.fmt.pix.pixelformat >> 24) & 0xff,
           fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);

    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        goto out;
    }
    if (req.count < 2) {
        fprintf(stderr, "not enough buffers: %u\n", req.count);
        goto out;
    }
    if (req.count > MAX_BUF)
        req.count = MAX_BUF;
    nbuf = (int)req.count;

    for (i = 0; i < nbuf; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = (unsigned int)i;
        if (xioctl(vfd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            goto out;
        }

        bufs[i].length = buf.length;
        bufs[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, vfd, buf.m.offset);
        if (bufs[i].start == MAP_FAILED) {
            perror("mmap");
            bufs[i].start = NULL;
            goto out;
        }
    }

    for (i = 0; i < nbuf; i++) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = (unsigned int)i;
        if (xioctl(vfd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            goto out;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(vfd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        goto out;
    }

    if (cfg.mode == MODE_ONCE) {
        struct v4l2_buffer buf;
        FILE *fp = NULL;

        if (wait_frame(vfd, 2000) <= 0) {
            perror("select");
            goto stream_off;
        }

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(vfd, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            goto stream_off;
        }

        printf("capture once ok: idx=%u bytes=%u ts=%ld.%06ld\n",
               buf.index, buf.bytesused,
               (long)buf.timestamp.tv_sec, (long)buf.timestamp.tv_usec);

        if (cfg.dump_path) {
            fp = fopen(cfg.dump_path, "wb");
            if (fp) {
                fwrite(bufs[buf.index].start, 1, buf.bytesused, fp);
                fclose(fp);
                printf("dumped one frame to %s\n", cfg.dump_path);
            } else {
                perror("fopen dump");
            }
        }

        if (xioctl(vfd, VIDIOC_QBUF, &buf) < 0)
            perror("VIDIOC_QBUF");

        ret = 0;
    } else {
        unsigned long long start = now_ns();
        unsigned long long end = start + (unsigned long long)cfg.sec * 1000000000ULL;
        unsigned long long frames = 0;
        unsigned long long drop_est = 0;
        unsigned long long wait_timeout = 0;
        int dumped = 0;
        unsigned int last_seq = 0;
        int last_seq_valid = 0;

        while (now_ns() < end) {
            struct v4l2_buffer buf;
            int w = wait_frame(vfd, 2000);
            if (w < 0) {
                if (errno == EINTR)
                    continue;
                perror("select");
                break;
            }
            if (w == 0) {
                wait_timeout++;
                continue;
            }

            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            if (xioctl(vfd, VIDIOC_DQBUF, &buf) < 0) {
                perror("VIDIOC_DQBUF");
                continue;
            }

            frames++;
            if (last_seq_valid && buf.sequence > last_seq + 1)
                drop_est += (unsigned long long)(buf.sequence - last_seq - 1);
            last_seq = buf.sequence;
            last_seq_valid = 1;

            if ((cfg.mode == MODE_PREVIEW || cfg.mode == MODE_PREVIEW_STATS) && fb.enabled) {
                yuyv_to_fb_rgb565_scaled((const unsigned char *)bufs[buf.index].start,
                                         fmt.fmt.pix.width,
                                         fmt.fmt.pix.height,
                                         fmt.fmt.pix.bytesperline,
                                         &fb);
            }

            if (!dumped && cfg.dump_path) {
                FILE *fp = fopen(cfg.dump_path, "wb");
                if (fp) {
                    fwrite(bufs[buf.index].start, 1, buf.bytesused, fp);
                    fclose(fp);
                    printf("dumped first frame to %s\n", cfg.dump_path);
                } else {
                    perror("fopen dump");
                }
                dumped = 1;
            }

            if (xioctl(vfd, VIDIOC_QBUF, &buf) < 0) {
                perror("VIDIOC_QBUF");
                break;
            }
        }

        {
            double elapsed_s = (double)(now_ns() - start) / 1000000000.0;
            double fps = (elapsed_s > 0.0) ? ((double)frames / elapsed_s) : 0.0;
            printf("stream summary: sec=%d frames=%llu fps=%.2f drop_est=%llu wait_timeout=%llu\n",
                   cfg.sec, frames, fps, drop_est, wait_timeout);
            if (cfg.mode == MODE_PREVIEW || cfg.mode == MODE_PREVIEW_STATS)
                printf("preview output updated on %s\n", cfg.fbdev);
        }

        ret = 0;
    }

stream_off:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(vfd, VIDIOC_STREAMOFF, &type);

out:
    for (i = 0; i < nbuf; i++) {
        if (bufs[i].start && bufs[i].start != MAP_FAILED)
            munmap(bufs[i].start, bufs[i].length);
    }
    if (fb.enabled)
        fb_close_unmap(&fb);
    if (vfd >= 0)
        close(vfd);
    return ret;
}
