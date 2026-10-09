# 项目规则索引

本目录依据 [RK3588-Blind](https://github.com/pythonerlower/RK3588-Blind/tree/main/rules) 的工程规则整理，并以 `plan/plan_imx6ull_pantilit_final.md` 为当前项目边界。

## 生效规则

- `constitution.md`：不可绕过的工程底线。
- `engineering-workflow.md`：开发、调试、测试、审查、验证和交付流程。
- `imx6ull-pantilt-engineering.md`：IMX6ULL 安防云台项目专用规则。
- `Hardware.md`：硬件限制约束（舵机行程等），随实测更新。

规则优先级：用户当前明确要求 > `constitution.md` > 项目专用规则 > 通用工作流 > 仓库既有约定。若规则与项目计划冲突，先说明冲突，不得静默选择。

## 与 RK3588-Blind 规则的取舍

| 处理 | 内容 |
| --- | --- |
| 保留并合并 | 证据优先、修改前理解、根因调试、最小变更、保护用户工作、新鲜验证、可复现交付、开源可追溯性 |
| 改造 | 目标板由 RK3588/ELF2 改为 IMX6ULL；主链路改为 OV5640→Ring Buffer→PC 跟踪→PCA9685→双 MG90S |
| 删除 | RKNN/NPU worker、RGA、Zero-Copy、双摄融合、RK3588 CPU/DDR 调优、板端 YOLO、VAD/ASR/TTS 专用规则 |
| 新增 | 机械/软件限位、上电小步标定、PC 与板端职责边界、过期控制命令拒绝、目标丢失 HOLD、第一版算法边界 |
| 收敛 | 不照搬多个内容重复的 Skill 文件；通用流程合并到单一工作流，避免规则本身制造文档负担 |

只有许可证明确且与当前技术路线匹配的上游代码才可进入复用候选；具体项目见 `reference/pantilt_tracking.md`。
