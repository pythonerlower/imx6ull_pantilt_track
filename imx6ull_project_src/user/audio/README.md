# audio：WM8960 用户态 ALSA 录音 / 播放

Phase 5 的两个命令行程序，给后续 `securityd` 当手和脚用：`audio_capture` 录、`audio_play` 放。
**不含任何降噪/滤波**（Phase 5.4 明确不做——降噪会污染原始取证音频），也不碰 mixer 设置：
出厂状态 Speaker=101 / Capture=35 就够用，程序偷偷改 mixer 会让"验收通过"依赖一个自己改过的状态。

| 文件 | 作用 |
| --- | --- |
| `audio_common.c/.h` | PCM 打开与参数协商、读写 + XRUN 恢复、WAV 头读写 |
| `audio_capture.c` | 录音：WAV/裸 PCM，定时长或录到收到信号 |
| `audio_play.c` | 播放：回放 WAV 文件、生成单音、生成告警双音 |

## 录音

```bash
./audio_capture --output /root/imx6hub_demo/records/test.wav --duration 10   # 指定路径
./audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 30   # 时间戳命名
./audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 0    # 一直录到 SIGINT/SIGTERM
```

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `-D, --device` | `hw:0,0` | ALSA PCM 设备名 |
| `-r, --rate` | 44100 | 采样率 |
| `-c, --channels` | 2 | 声道数 |
| `-f, --format` | `S16_LE` | 采样格式 |
| `-d, --duration` | 10 | 秒；**0 = 一直录到收到 SIGINT/SIGTERM** |
| `-o, --output` | — | 输出路径，**原样使用，不做任何加工** |
| `--dir` / `--tag` | `/root/imx6hub_demo/records` / `rec` | 不给 `-o` 时的目录与标签，文件名 `YYYYMMDD_HHMMSS_<tag>.wav`（目录不存在会自动建） |
| `--period-size` / `--periods` | 1024 / 4 | 周期大小与周期数，设备可能取邻值 |
| `--raw` | 关 | 写裸 PCM，不写 WAV 头 |
| `--quiet` | 关 | 不打印每秒进度 |

## 播放

```bash
./audio_play --alert                                    # 告警双音（880/1320 Hz 交替）——不带参数时就是它
./audio_play --tone 1000 --duration 2                   # 1 kHz 单音 2 秒
./audio_play --file /root/imx6hub_demo/records/xxx.wav  # 回放录音（位置参数也接受）
```

回放文件时按**文件自己的**采样率/声道/格式开设备，不套默认值；`--gain PCT`（0~200）可调音量，
对 U8/S16_LE/S32_LE 有效。`--duration` 在文件模式下表示"只放前 N 秒"。

## 给 securityd 用

每个程序退出前打一行**纯 ASCII** 结果，直接解析这一行判断成败，不要去 grep 中文：

```
audio: RESULT op=capture dev=hw:0,0 frames=132300 rate=44100 xrun=0 suspend=0 short_io=0 bytes=529200 wall_s=3.05
```

`xrun`（欠载/过载，已自动恢复）、`suspend`、`short_io` 是累计计数，正常退出时也打在 stderr——
这是 `rules/imx6ull-pantilt-engineering.md` §6 "ALSA XRUN 必须可观察" 的落点。

典型用法：

```bash
./audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 0 &   # 持续录
kill -TERM $!                                                                 # 停止：正常收尾，WAV 头已回填
./audio_play --alert                                                          # 报警
```

**SIGTERM 停止是设计用法，不是兜底**：`sigaction` 不设 `SA_RESTART`，`snd_pcm_readi` 被打断后
走的是和正常退出**同一条**收尾路径（回填 `riff_size`/`data_size` 后 `fsync`），所以拿到的
一定是可直接播放的文件。反过来，被 `kill -9` 打断的录音 `audio_play` 也能放——它会用真实文件
长度兜底头里写歪的 `data_size`。

## 板端参数限制（声明支持 ≠ 能用）

`arecord --dump-hw-params` 声明的能力比设备实际能装上的 `hw_params` 乐观，实测撞过三个
（都用 `arecord` 独立复现过，不是本程序的 bug）：

| 声明 | 实际 |
| --- | --- |
| `RATE: [8000 48000]` | **8000 Hz 装不上**，`hw_params` 返回 EINVAL；11025 起正常 |
| `FORMAT: ... S20_3LE` | `set_format` 通过，`hw_params` EINVAL |
| `SAMPLE_BITS: [16 32]` | 只支持 `S16_LE` / `S24_LE` / `S32_LE`；`U8`、`S24_3LE` 在 `set_format` 就被拒 |

不支持的参数一律**明确报错退出非 0**，不偷偷转换或重采样。

一个容易误解的点：`-f S24_LE` 录出来的 WAV 头里 `bits` 写的是 **32**——这是对的，
规范的 44 字节 PCM WAV 头描述的是"容器"位宽，24-in-32 的打包方式它表达不了。
回放时按 32 位打开，设备同样接受，往返一致。

## 编译

```bash
make            # 交叉编译，产物 audio_capture / audio_play
make clean
```

需要在开发机上有 Yocto sysroot（`ALSA_SYSROOT` 可覆盖）。**不要**改用
`-L$(ALSA_SYSROOT)/usr/lib -lasound`：那个目录里的 `libc.so` 是带硬编码 `/lib/libc.so.6` 路径的
链接脚本，会让 ld 去找宿主机的 libc。Makefile 直接链接版本化的 `libasound.so.2.*` 全路径。

## 验收

`scripts/board_verify_audio.sh` 一次跑完 7 段（环境 / 默认目录录音 / 指定路径+参数 / 播放 /
声学回环 / XRUN / SIGTERM 停止）。其中"播放是否真的出声"用**声学回环 + Goertzel 频点检测**判，
不靠人耳也不靠程序自述。现场没喇叭时 `WITH_LOOPBACK=0` 跳过回环段。
