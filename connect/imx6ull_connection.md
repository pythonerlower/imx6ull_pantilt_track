# IMX6ULL 开发板连接说明

## 已验证的连接信息

从 Ubuntu 开发机 `/home/imx6ull` 连接开发板。2026-10-06 实测 SSH 登录成功，板端返回 `root`、主机名 `ATK-IMX6U`、内核 `4.1.15`，且 `/root/imx6hub_demo` 已存在。

| 项目 | 固定值 |
| --- | --- |
| Ubuntu 开发机网卡 | `ens38`，`192.168.137.100/24` |
| 开发板 | `192.168.137.50:22`，用户 `root` |
| 本机 SSH 别名 | `imx6ull-board` |
| 主机密钥 | RSA，`SHA256:YeKYFbLroW9SZrpmYAn1sCOmxlvNCCylfPaR4lKsqfI` |
| 主机密钥记录 | `/home/imx6ull/.ssh/known_hosts` |
| Ubuntu 项目目录 | `/home/imx6ull/work/imx6ull/imx6ull_project` |
| 板端部署目录 | `/root/imx6hub_demo` |

本机 `/home/imx6ull/.ssh/config` 已配置该别名：校验已记录的主机密钥，5 秒连接超时，15 秒保活且连续 3 次无响应后断开；仅为兼容此旧板端追加 `ssh-rsa` 算法。不要把 `StrictHostKeyChecking` 设为 `no`，也不要将 `known_hosts` 指向 `/dev/null`。如果板端重刷后主机密钥变化，应通过可信的板端控制台核对新指纹，再更新 `known_hosts`；不要直接跳过告警。

```bash
# 从 Ubuntu 开发机进行只读连接检查；超时后返回错误码，不会无限等待。
timeout 10 ssh imx6ull-board 'id -un; hostname; uname -r; uptime'

# 不依赖别名时，用已知主机密钥记录和旧板端所需算法直接连接。
ssh -F /dev/null -o BatchMode=yes -o ConnectTimeout=5 \
  -o StrictHostKeyChecking=yes \
  -o UserKnownHostsFile=/home/imx6ull/.ssh/known_hosts \
  -o HostKeyAlgorithms=+ssh-rsa \
  -o PubkeyAcceptedAlgorithms=+ssh-rsa \
  root@192.168.137.50
```

当前板端 SSH 会话报告认证方式为 `none`，本机也未发现用于该板的私钥文件。**这表示目前可免凭据以 root 登录，是现有板端的安全风险，不应误认为已经配置了密钥认证。**后续若收紧板端认证，需先确认新凭据可用，再调整本机 `BatchMode` 等选项，避免把开发入口锁死。不要把密码或私钥写进本文件。

## `work` 目录中的既有开发方式

1. 在 Ubuntu 项目目录交叉编译：`Makefile` 调用 `scripts/build_all.sh`；内核源码位于 `/home/imx6ull/work/imx6ull/src`。
2. `scripts/deploy_offline.sh` 默认使用 `BOARD_IP=192.168.137.50`、`BOARD_USER=root`，经 SSH 创建 `/root/imx6hub_demo`，再通过 SCP 上传编译产物。其默认先构建，`SKIP_BUILD=1` 才跳过构建。只在确认目标文件后运行部署脚本。
3. 板端运行模块、应用和诊断脚本；项目 `README.md` 记录了 `/dev/mx6hub0`、debugfs 等验证入口。
4. `scripts/remote_link/` 的视频预览使用 SSH、UDP/RTP、RTSP 或 HTTP MJPEG 链路；这些是视频传输方案，不是板端 SSH 管理入口。
5. `docs/ssh_dev_workflow.md` 主要描述从其他客户端 SSH 连接 **Ubuntu 源端** 的编辑流程，不能把其中“Ubuntu 源端”误读成开发板。`README.md` 提及 NFS/TFTP 场景，但本次在项目 `scripts` 下未找到其中所写的 `deploy_online.sh`。

```bash
# 在 Ubuntu 项目目录先检查环境，再交叉编译；这些命令只在准备部署时运行。
cd /home/imx6ull/work/imx6ull/imx6ull_project
make host-check
make build

# 确认产物和板端状态后，才执行会向板端写文件的离线部署。
BOARD_IP=192.168.137.50 BOARD_USER=root bash scripts/deploy_offline.sh
```

## 板端死机或重启时的检查顺序

```bash
# 先检查本机开发网口地址与路由；ping 丢包不能单独证明 SSH 不可用。
ip -brief address show ens38
ip route get 192.168.137.50

# 再做限时 SSH 检查；返回码非 0 时保留错误输出以区分网络、密钥和认证问题。
timeout 10 ssh imx6ull-board 'uptime; cat /proc/version'

# 恢复登录后只读查看本次启动信息；不要在尚未定位原因时反复自动重启。
ssh imx6ull-board 'cat /proc/uptime; dmesg | tail -n 80'
```

若网络不可达，检查开发板供电、网线、`ens38` 与板端地址；若提示主机密钥不匹配，先核验板端身份；若可登录但程序异常，再检查板端日志、模块及设备节点。SSH 无法连接时，它无法代替串口/板端控制台获取死机瞬间日志。每次重启后先核对板端 `uptime`，避免把旧会话或旧日志当成本次结果。

## Windows 宿主机（2026-10-07 起可 SSH 接入）

开发用的 VMware 宿主机本身也可 SSH 接入，用于排查那些**不在本仓库代码里**的问题
（首要的就是 ICS 造成的链路三倍复制，见 [`plan/phase3_status_20261007.md`](../plan/phase3_status_20261007.md)）。

| 项目 | 固定值 |
| --- | --- |
| 宿主机 | `DESKTOP-O5NKV99`，Windows 11 Home China |
| 接入地址 | `192.168.80.1`（VMnet1 主机模式，本机 `ens37` 侧） |
| 用户名 | `huawei` |
| 密钥 | `/home/imx6ull/.ssh/id_ed25519_win`（**无口令**，为 `BatchMode` 自动化；私钥权限 600） |
| 主机密钥 | ED25519，`SHA256:WfLpmdRIaIkqFRLHm/bdlHYxGxmzANv+gCiXLSa8rP8` |
| 默认 shell | `cmd.exe` —— 不接受 `;`，多命令要用 `&` |

```bash
# 只读探活
timeout 15 ssh -i ~/.ssh/id_ed25519_win -o BatchMode=yes huawei@192.168.80.1 'hostname'
```

**跑 PowerShell 脚本要用 stdin，不要用 `-EncodedCommand`**：后者在这台机器上会挂起
（最短脚本也卡住，原因未查明）。脚本内容只用 ASCII——stdin 按 OEM 代码页解码，
中文会乱码；中文只出现在输出里，输出侧显式设 UTF-8。

```bash
# 已封装成脚本，直接用；细节与两个坑的说明见该脚本头部注释。
printf '[Console]::OutputEncoding=[Text.Encoding]::UTF8\nGet-Service SharedAccess\n' \
  | bash scripts/host_win_ps.sh
```

**这台机器上 ICS 已关闭**（`SharedAccess` = `Stopped` / `Disabled`），因为它的 IP 转发
与同一张网卡上的 VMware 桥接叠加，会把上行包复制三份。**改动前原值与回滚命令**记在
[`plan/phase3_status_20261007.md`](../plan/phase3_status_20261007.md) §3.3。
两个已知副作用：关共享会释放 `192.168.137.1`（需手动补成静态），并瞬时打断
VMware 桥接（`ens38` 短暂 DOWN，会自行恢复）。
