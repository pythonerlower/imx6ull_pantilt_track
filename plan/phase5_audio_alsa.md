# Phase 5 执行计划：WM8960 用户态 ALSA 录音/播放

本文件是 [总计划](plan_imx6ull_pantilit_final.md) 第 Phase 5 章的展开，服从 `rules/` 全部规则。
**状态：2026-10-07 已完成，板端合并验收 7/7 PASS（`logs/phase5/p5_verify_audio.log`）。**

## 0. 边界与协商结论

总计划给的是"补用户态 ALSA 调用程序"，本轮严格按此执行，开工前定下 4 条：

| # | 决策 | 理由 |
| --- | --- | --- |
| 1 | **不写 WM8960 Codec 驱动**，只写用户态 | WM8960 已由 BSP（`wm8960-audio` 声卡 + `snd_soc_wm8960`）提供，重写属于无收益的重复劳动 |
| 2 | **5.4 可选增强（DC removal / high-pass / noise gate / 频域降噪）本轮不做** | 用户明确"没有把握暂时不做"；且降噪会污染原始取证音频 |
| 3 | 一个录音程序 + 一个播放程序，共用 `audio_common.c` | PCM 打开/XRUN 恢复/WAV 头三件事两边完全一样，重复实现必然分叉 |
| 4 | 文件名与目录按总计划建议固定：`records/<时间戳>_<tag>.wav` | 便于后续 `securityd` 直接按目录扫描 |

## 1. 关键约束（已确认，不是推断）

- 声卡 `hw:0,0`：`0 [wm8960audio ]: wm8960-audio`，录音/播放共用同一 PCM 节点（`pcmC0D0c` / `pcmC0D0p`）。
- 板端 libasound 是 Yocto 1.1.0；开发机用 `/opt/fsl-imx-x11/4.1.15-2.1.0/sysroots/cortexa7hf-neon-poky-linux-gnueabi`
  下**同一份** sysroot 的头与库交叉编译，避免 ABI 猜谜。
- 链接**不能**写 `-L$SYSROOT/usr/lib -lasound`：该目录的 `libc.so` 是带硬编码 `/lib/libc.so.6` 路径的
  链接脚本，会让 ld 去找宿主机的 `/lib/libc.so.6`。改成直接写版本化 `.so` 全路径（见 `user/audio/Makefile`）。
- 不做 mixer 设置：出厂状态下 Speaker=101 / Capture=35 已经能被麦克风拾到，程序里一律不碰 mixer，
  免得"验收通过"依赖一个自己偷偷改过的状态。

## 2. 交付物

```
imx6ull_project_src/user/audio/  audio_capture.c  audio_play.c  audio_common.c  audio_common.h
                                 Makefile  README.md
scripts/board_verify_audio.sh    # 合并验收脚本（见 §6）
logs/phase5/                     # 验收日志与本目录索引
```

构建/部署接入：`scripts/build_all.sh` 第 `[8/8]` 步、`scripts/deploy_offline.sh` 的拷贝与上传清单、
根 `Makefile` 的 `audio` 目标与 `clean` 条目。

## 3. 实现要点

### 3.1 录音（`audio_capture`）

参数：`-D` 设备、`-r` 采样率、`-c` 声道、`-f` 格式、`-d` 时长（**0 = 一直录，直到 SIGINT/SIGTERM**）、
`-o` 输出路径、`--dir` / `--tag`（不给 `-o` 时拼文件名）、`--period-size` / `--periods`、`--raw`。

流程即总计划要求的顺序：建文件 → 写 WAV 头 → `snd_pcm_open` → `hw_params` → `snd_pcm_readi` →
XRUN 恢复 → 写数据 → 正常收尾关闭。

两个刻意的设计：

- **WAV 头先写占位（data_size=0），退出时回填真实长度。** 头保存在调用方内存里由
  `ac_wav_finalize()` 原样回写——**不能**"先把头读回来再改"：输出文件是 `O_WRONLY` 打开的，
  `pread()` 会直接 EBADF（这是本轮实际踩到并修掉的）。
- `sigaction` **不设 `SA_RESTART`**，让 `snd_pcm_readi` 被信号打断返回 `EINTR`；
  收到信号后走**同一条**正常收尾路径（回填头 + 关文件），所以 `securityd` 用 SIGTERM 停止录音
  拿到的一定是可直接播放的文件，而不是残缺头。

### 3.2 播放（`audio_play`）

三种来源：`-F/--file` 回放 WAV、`-t/--tone HZ` 生成单音、`--alert` 生成告警双音
（880/1320 Hz 交替，0.25 s 响 + 0.15 s 停，10 ms 上升/下降沿）。不带任何参数时默认播一次告警。

- 回放文件时按**文件自己的** 采样率/声道/格式开设备（不套用默认值），并对"`data_size` 声明为 0 或
  大于实际文件长度"做兜底（用真实文件长度）——这样被 `kill -9` 打断的录音也能放。
- 单音按"绝对时间函数"生成而非逐块累加相位，避免换块时相位跳变（听起来是咔哒声）。
- 正常放完调 `snd_pcm_drain()`，确保最后一段真的从喇叭出去而不是被丢掉。

### 3.3 securityd 接口（Phase 5.3）

按总计划建议，`securityd` 直接命令行驱动即可，无需额外协议：

```bash
# 开始录音（前台）或后台常录
./audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 30
./audio_capture --dir /root/imx6hub_demo/records --tag alert --duration 0 &   # 录到收到信号为止
kill -TERM <pid>          # 停止录音：正常收尾，WAV 头已回填
./audio_play --alert      # 播报警告音
./audio_play -F /root/imx6hub_demo/records/20261006_103700_alert.wav   # 回放取证录音
```

每个程序退出前打一行**纯 ASCII** 机器可读结果，供 `securityd` 判断成败：

```
audio: RESULT op=capture dev=hw:0,0 frames=132300 rate=44100 xrun=0 suspend=0 short_io=0 bytes=529200 wall_s=3.05
```

`xrun` / `suspend` / `short_io` 计数与 `rules/imx6ull-pantilt-engineering.md` §6
"ALSA XRUN 必须可观察"的要求对应，正常退出时同时打在 stderr。

## 4. 实测（板端，2026-10-07，未改任何 mixer）

| 项 | 结果 |
| --- | --- |
| 默认目录录音 | `records/20261007_180505_vfy.wav`，3.00 s / 132300 帧 / 529200 字节，`wall_s=3.04` |
| 录音非静音 | python `wave` 独立统计 `acrms=105.0 acpeak=4541`（数字静音应为 0） |
| `--output` + 参数 | 22050 Hz/1ch/16bit（`acrms=152.9`）与 48000 Hz/2ch/32bit（`acrms=202.8`）都原样落进 WAV 头 |
| 回放录音文件 | 退出码 0，`wall_s=3.02`（音频 3.00 s，实时速率消费，不是"写进去就返回"） |
| 单音 / 告警音 | 1000 Hz 2 s → `wall_s=2.02`；告警 3 s → `wall_s=3.02` |
| **声学回环** | 播告警音同时录音，880 Hz **+20.9 dB**（11 窗超阈）、1320 Hz **+21.6 dB**（12 窗），两频点任一超阈 23/50 窗 |
| XRUN 恢复 | SIGSTOP 1.2 s 制造 overrun → `xrun=1`，恢复后跑满 176400 帧（4 s @44100） |
| SIGTERM 停止 | 录到 2.46 s 收尾，头已回填，文件可直接回放 |

声学回环这一项是本轮**唯一能证明"声音真的出去了又回来了"**的判据：不靠人耳、不靠程序自述，
而是用麦克风录下喇叭的声音再做频点检测。

### 4.1 采集方向会进入持久坏状态，唯一手段是重启板子

2026-10-07 调试中撞到一次，记录备查（**不是本 Phase 代码的问题**）：

- **现象**：纯录音报 `read error: Input/output error`，`appl_ptr=0 hw_ptr=0`，只写出 44 字节空 WAV 头。
  `hw_params` 本身是成功的（`arecord -v` 打印完 setup 才在 `pcm_read` 失败）。
  **播放方向同时完全正常**，按实时速率消费帧——只看播放会误判声卡没问题。
- **边界**：只要有一个播放流开着，录音就正常；播放流关闭后约 5 s 内也正常，再往后就退化成上面那样。
- **排除**：`/sys/devices/platform/sound/*/pmdown_time` 设 0 或 5000 无差别；在用户态
  `snd_pcm_close()` 前补 `snd_pcm_hw_free()` 无差别；"先放一段再录"只是上面那条边界，不是修复。
  全表 mixer 与基准一致（Speaker=101 / Capture=35 全程没动）。
- **原因判断**：坏在内核 codec / `imx-wm8960` 的运行时状态。用户态既看不到也改不了——
  `/sys/kernel/debug/regmap/0-001a/*` 读不了（读一下就会触发内核 WARNING 并给内核打 taint），
  i2c-dev 对 codec 的读全部 `EIO`、写全部 NAK。
- **恢复**：**重启板子**。板端 `/sbin` 不在 `PATH`，必须写绝对路径 `/sbin/reboot`，否则命令静默找不到。
  重启后当天复跑验收 7/7 PASS，`tainted` 回到 0。

**重启后紧接着的第一次**跑验收脚本，声学回环会擦边不过（实测并集只有 4/50 窗，判据 ≥5，
但尖峰仍有 +16.0/+14.5 dB）；隔几分钟再跑同一脚本就是 34/50 窗、7/7 PASS。
判定前先热身跑一遍，别拿第一遍的结果下结论。

## 5. 板端三个"声明支持但其实不能用"的坑

`arecord --dump-hw-params` 声明的能力与设备实际能装上的 hw_params **不一致**，
三者都撞过，都**不是**本程序的 bug（用 `arecord` 独立复现过）：

| 声明 | 实际 | 复现方式 |
| --- | --- | --- |
| `RATE: [8000 48000]` | **8000 Hz 装不上**，`snd_pcm_hw_params` 返回 EINVAL；11025 起正常 | `arecord -f S16_LE -r 8000` 同样 "Unable to install hw params" |
| `FORMAT: ... S20_3LE` | `set_format` 通过，但 `hw_params` EINVAL | 本程序 `-f S20_3LE` |
| `SAMPLE_BITS: [16 32]` | 只支持 `S16_LE / S24_LE / S32_LE`；`U8`、`S24_3LE` 在 `set_format` 就被拒 | 本程序 `-f U8` / `-f S24_3LE` |

顺带一个容易误解的点：`-f S24_LE` 录出来的 WAV 头里 `bits` 写的是 **32**。
这是**对**的——规范的 44 字节 PCM WAV 头描述的是"容器"位宽，24-in-32 这种打包方式它表达不了；
回放时按 32 位打开，设备同样接受，往返一致。

## 6. 验证方式

`scripts/board_verify_audio.sh` 一次跑完 7 段（环境 / 默认目录录音 / 指定路径+参数 / 播放 / 声学回环 /
XRUN / SIGTERM 停止），每段只留 1~2 条硬判据，不做时长×格式矩阵（省时间，也避免把偶发当规律）。

判据尽量取**独立于被测程序的第三者**：

- WAV 合法性用板端 python3 的 `wave` 模块判，不信程序自述；
- **录音是否真有数据用 `nframes>0` 判** —— 44 字节的空 WAV 头在 size/rate/ch/bits 上是"自洽"的，
  只查头会漏判（2026-10-07 就因此把一次采集失效判成了 PASS，见 §4.1）；
- 录音退出码非 0 也计入判据 —— 半坏时会写出"头合法但只有一小段数据"的文件；
- 播放是否真的出声用声学回环 + Goertzel 判；
- "是不是真按实时速率跑"用 `wall_s≈音频时长` 判，防"写进缓冲就返回"的假成功。

**Goertzel 为什么要滑窗计数，而不是整段做一次 DFT**：告警音是门控的（0.25 s 响 / 0.15 s 停），
整段做变换会把静音间隙一起摊进去。同一段录音两种算法直接对比：

```text
整段 2.8s 做一次变换 : 880Hz= +7.7dB  1320Hz=+10.7dB   ← 按 10dB 判据会误判"没出声"
0.30s 子窗滑窗       : 880Hz=+18.9dB(5窗)  1320Hz=+21.8dB(10窗)   任一超阈 15/50 窗
```

但滑窗后又冒出第二个问题：只看"最好的一窗"会被多重比较带偏（几十个窗里总有一个噪声窗冒头）。
最终判据是**计数**，而且是**两个频点取并集**——告警音本来就是 880/1320 交替，
一个窗里只有其中一个在响，各算各的等于把要求凭空翻倍（实测同一轮里 880 只有 5 窗、
1320 有 10 窗，卡在"各 ≥5"的边界上）。**判据：任一频点 ≥14 dB 的窗口数 ≥5。**

两侧余量都实测过：同一手法跑 4 轮，并集窗口数 15 / 35 / 35 / 35（最弱一轮 3 倍余量）；
合成纯噪声最好一窗只有 +9.4 dB、0 窗；合成门控鸣叫 55/55 窗。验收连跑两轮也都 PASS
（并集 13 / 23 窗），判据收紧后复跑 22 窗。
**例外**：重启板子后紧接着的第一次跑会擦边不过（4/50 窗），原因与处置见 §4.1。

脚本可用 `WITH_LOOPBACK=0` 跳过回环段（现场没喇叭时），汇总记 NOT RUN。
全程不改 mixer、不碰 boot 分区。

## 7. Phase PASS 对照

| 总计划判据 | 结果 |
| --- | --- |
| 自定义程序可录音 | PASS（§4 前两行） |
| 可以通过命令参数指定保存路径 | PASS（`--output` 原样使用；`--dir/--tag` 走时间戳命名） |
| 录音文件可以正常回放 | PASS（`audio_play -F`，`wall_s` 与音频等长） |
| 自定义程序可播放测试音频 | PASS（单音 + 告警音 + 回环声学证明） |
| 能处理基本 ALSA XRUN/错误 | PASS（overrun 恢复并跑满帧数；格式/采样率不支持时明确报错退出非 0） |

## 8. 回退

本 Phase 全是**新增**用户态程序，不替换任何既有实现：

- 板端删掉 `audio_capture` / `audio_play` 即回到 Phase 4 状态，无残留；
- 内核模块、DTS、mixer 设置、boot 分区**全程未动**；
- 验收脚本默认不删任何文件，录音证据保留在 `/root/imx6hub_demo/records/`。

## 9. 明确不做（本轮 Non-goals）

- **5.4 全部可选增强**：DC removal、high-pass、noise gate、简单频域降噪。用户明确"没有把握暂时不做"。
- 不写 WM8960 Codec 驱动，不动 ASoC 机器驱动。
- 不做深度学习音频算法（总计划明令禁止）。
- 不做音频格式转换/重采样（设备不支持的参数直接报错，不偷偷转换）。
- 不做 `securityd` 本身——本 Phase 只把它要调的**接口**准备好（§3.3）。
