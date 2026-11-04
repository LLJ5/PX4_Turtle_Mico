# PX4_Turtle_Mico
微空AIO上添加了反乌龟模式，无人机可以翻转为正向

本仓库保存 MicoAir H743 V2 AIO 的自动右翻开发代码。当前快照包含工作区中全部翻身功能相关的 22 个源码、测试、消息和板级配置文件，按 PX4 原目录排列。

## 基础版本与使用

这是叠加到 PX4 工程的功能代码包，构建所需的其余 PX4 源码与子模块需要另行获取。

- PX4 基础版本：`v1.17.0`
- 基础提交：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`
- 完整功能补丁：[turtle_recovery_full.patch](patches/turtle_recovery_full.patch)
- 操作与实现说明：[TURTLE_RECOVERY.txt](docs/TURTLE_RECOVERY.txt)
- 调参说明：[TUNING.txt](docs/TUNING.txt)

在对应版本的 PX4 工程根目录执行（替换补丁的实际路径）：

```bash
git apply --check /path/to/PX4_Turtle_Mico/patches/turtle_recovery_full.patch
git apply /path/to/PX4_Turtle_Mico/patches/turtle_recovery_full.patch
git submodule update --init --recursive
make micoair_h743-v2_default -j4
```

也可以按相对路径覆盖本仓库中的 22 个实现文件；两种安装方式任选一种。已包含该功能的工程不要重复应用补丁。

## 当前功能

- 遥控 AUX 开关触发，结合四元数姿态、角速度、落地与静止条件判断是否开始右翻。
- DShot 专用输出使逻辑 Motor 1、4 临时反转，受控出力后停止并恢复转向。
- 包含请求超时、出力时限、无进展退出，以及 Commander 普通解锁互锁。
- 包含观察逻辑、翻身控制和 DShot 命令的测试源码。
- `TRTL_ENABLE` 默认关闭；板级启动脚本在其启用后自动启动模块。

首次验证应拆下全部螺旋桨。实现与软件测试不能替代电调、机架和带桨翻身实测；具体步骤和边界见操作说明。参数默认值与行为以本仓库源码为准。

## 快照范围与校验

同步日期：2026-11-04。本次从 PX4 工作区读取最新实现，不使用旧导出目录中已落后的源码副本。文件校验值见 [SHA256SUMS](SHA256SUMS)。同步时核对了 22 个文件与工作区逐字节一致，并检查完整补丁可反向匹配当前实现；本次上传没有重新编译固件或运行单元测试。

[第一阶段评审记录](turtle_recovery_review/turtle_recovery_stage1_review.txt)和[第一阶段补丁](turtle_recovery_review/turtle_recovery_stage1.patch)仅保留作历史资料。第一阶段只有观察功能，不能用其覆盖当前支持电机输出的实现。

不包含工作区中的 PID 学习注释、无关板级文件删除、工具链压缩包和编译产物。
