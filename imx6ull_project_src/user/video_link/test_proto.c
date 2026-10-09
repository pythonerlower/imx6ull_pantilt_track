/*
 * 线格式自测。两类作用：
 *
 * 1. 断言：编解码往返、坏 CRC / 错长度必须被拒。这些在开发机上跑，不碰硬件。
 * 2. 打固定向量：把三份报文的十六进制打到 stdout，供 scripts/proto_selftest.sh
 *    与 PC 侧 Python 实现逐字节对拍。
 *
 * 第 2 条才是真正防漂移的那一条。两侧各自断言 sizeof 只能抓住"改了尺寸"，
 * 抓不住"尺寸没变但字段顺序换了"——那种错靠人眼审代码几乎必漏，
 * 而十六进制一摆出来就是一眼的事。
 *
 * 向量值一旦定下就不要改：改了要让 pc/tracker/ptproto.py 的 test 向量同步，
 * 否则对拍脚本会红，而那种红是"改协议"的正常信号，不该被当成噪声。
 */
#include <stdio.h>
#include <string.h>

#include "pantilt_proto.h"

static const uint8_t VEC_PAYLOAD[] = "HELLO-IMX6ULL";

static void print_hex(const char *label, const uint8_t *buf, size_t len)
{
	printf("%-6s", label);
	for (size_t i = 0; i < len; i++)
		printf("%02x", buf[i]);
	printf("\n");
}

/* 与 ptproto.py 的 Ctrl 自检向量对应。 */
static void vector_ctrl(void)
{
	struct ptp_ctrl c = { 0 };
	uint8_t out[PTP_CTRL_SIZE];
	struct ptp_ctrl back;

	c.magic = PTP_CTRL_MAGIC;
	c.version = PTP_VERSION;
	c.mode = PTP_MODE_MANUAL;
	c.seq = 7;
	c.timestamp_us = 1234567890123ull;
	c.flags = PTP_FLAG_HEARTBEAT;
	c.pan_tick = 377;
	c.tilt_tick = 242;
	ptp_ctrl_encode(&c, out);

	if (ptp_ctrl_decode(out, sizeof(out), &back) != 0)
		printf("  [FAIL] ctrl 往返解码失败\n");
	/*
	 * 逐字段比较，不能 memcmp：结构体带填充字节（见 pantilt_proto.h 的说明），
	 * 填充区的内容是未定义的，memcmp 会因此在"字段其实全对"时报错。
	 */
	else if (back.magic != c.magic || back.version != c.version ||
		 back.mode != c.mode || back.seq != c.seq ||
		 back.timestamp_us != c.timestamp_us || back.flags != c.flags ||
		 back.pan_tick != c.pan_tick || back.tilt_tick != c.tilt_tick)
		printf("  [FAIL] ctrl 往返字段不一致\n");
	else
		printf("  [ok] ctrl 往返一致\n");

	/* 翻一个 bit 必须被 CRC 抓住。 */
	out[10] ^= 0x01;
	if (ptp_ctrl_decode(out, sizeof(out), &back) == 0)
		printf("  [FAIL] ctrl 坏 CRC 未被拒\n");
	else
		printf("  [ok] ctrl 坏 CRC 被拒\n");
	out[10] ^= 0x01;

	size_t short_len = PTP_CTRL_SIZE - 1;

	if (ptp_ctrl_decode(out, short_len, &back) == 0)
		printf("  [FAIL] ctrl 错长度未被拒\n");
	else
		printf("  [ok] ctrl 错长度被拒\n");

	ptp_ctrl_encode(&c, out);
	print_hex("ctrl", out, sizeof(out));
}

static void vector_telem(void)
{
	struct ptp_telem t;
	uint8_t out[PTP_TELEM_SIZE];

	memset(&t, 0, sizeof(t));
	t.magic = PTP_TELEM_MAGIC;
	t.version = PTP_VERSION;
	t.drv_status = PTP_DRV_SLEW;
	t.seq = 9;
	t.timestamp_us = 9876543210987ull;
	t.chip_state = PTP_CHIP_RUNNING;
	t.daemon_state = PTP_DST_TRACK;
	t.slew_ms = 20;
	t.wd_ms = 1500;
	t.daemon_wd_ms = 500;
	t.pan_cur = 377;
	t.tilt_cur = 242;
	t.pan_tgt = 400;
	t.tilt_tgt = 222;
	t.pan_home = 377;
	t.tilt_home = 242;
	t.pan_safe_min = 239;
	t.pan_safe_max = 515;
	t.tilt_safe_min = 162;
	t.tilt_safe_max = 342;
	t.ms_since_ctrl = 1234;
	t.last_latency_us = 20000;
	t.cmd_count = 1000;
	t.clamp_low_pan = 1;
	t.clamp_low_tilt = 2;
	t.clamp_high_pan = 3;
	t.clamp_high_tilt = 4;
	t.i2c_err = 5;
	t.watchdog_trips = 6;
	t.ctrl_rx_ok = 999;
	t.rej_crc = 7;
	t.rej_seq = 8;
	t.rej_stale = 9;
	t.rej_mode = 10;
	t.rej_range = 11;
	t.daemon_wd_trips = 12;

	ptp_telem_encode(&t, out);
	print_hex("telem", out, sizeof(out));
}

static void vector_video(void)
{
	struct ptp_video_meta m = { 0 };
	uint8_t hdr[PTP_VIDEO_HDR_SIZE];
	uint8_t dgram[PTP_VIDEO_DGRAM];
	size_t plen = sizeof(VEC_PAYLOAD) - 1; /* 不含结尾 NUL */

	m.frame_id = 4096;
	m.timestamp_us = 1122334455667ull;
	m.width = 320;
	m.height = 240;
	m.format = PTP_FMT_YUYV;
	m.flags = 0;

	memcpy(dgram + PTP_VIDEO_HDR_SIZE, VEC_PAYLOAD, plen);
	ptp_video_hdr_encode(&m, 3, 110, dgram + PTP_VIDEO_HDR_SIZE,
			     (uint16_t)plen, hdr);
	memcpy(dgram, hdr, PTP_VIDEO_HDR_SIZE);

	print_hex("video", dgram, PTP_VIDEO_HDR_SIZE + plen);
}

int main(void)
{
	printf("线格式自测（C 侧）\n");

	/*
	 * 这里**不**断言 sizeof(struct) 等于线上长度——两者本就不等，也不该等。
	 * 结构体是内存里的便利容器，编译器会为 u64 对齐插入填充字节
	 * （实测 sizeof(ptp_ctrl)=40、sizeof(ptp_video_hdr)=40），而线上格式由
	 * ptp_put_* 的显式编码定义。真正该锁的是"编码器只写线上那几十个字节"，
	 * 这一点由编码函数的 out[] 形参尺寸在编译期保证。
	 *
	 * 反过来断言 sizeof(struct)==线上长度的话，会把"编译器加了填充"误报成缺陷，
	 * 反而教人把断言删掉。这里改为把这个事实显式打出来，防止有人日后
	 * 误把结构体直接 memcpy 上网络。
	 */
	if (sizeof(struct ptp_ctrl) == PTP_CTRL_SIZE)
		printf("  [warn] sizeof(ptp_ctrl) 恰等于线上长度，纯属巧合，切勿依赖\n");
	if (PTP_VIDEO_DGRAM + 28 > 1500)
		printf("  [FAIL] 分包 %d 字节加 IP/UDP 头后超过 MTU 1500\n",
		       PTP_VIDEO_DGRAM);

	/* CRC 标准向量：这一条对不上，说明多项式或初值写错了。 */
	{
		uint32_t got = ptp_crc32("123456789", 9);

		if (got != 0xCBF43926u)
			printf("  [FAIL] crc32(\"123456789\")=0x%08x，应为 0xcbf43926\n",
			       got);
		else
			printf("  [ok] crc32 标准向量 0xcbf43926\n");
	}

	vector_ctrl();
	vector_telem();
	vector_video();
	return 0;
}
