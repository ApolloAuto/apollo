# Cyber Recorder GUI Lite 使用说明

`cyber_recorder_gui_lite` 是一个基于 Qt 的 Apollo Cyber record 轻量级图形回放工具。它可以打开单个 record 文件，也可以加载目录中的分片 record 文件，显示 record 基础信息，并将消息重新发布到 Cyber 通道，方便调试依赖历史数据的模块。

本模块是 `modules/cyber_recorder_gui` 的缩减版。它使用独立的包名、节点名和可执行文件名，可以与完整版同时保留，避免安装目标冲突。

## 功能

- 打开单个 `.record` 文件。
- 打开包含 `.record` 和 `.record.*` 分片文件的目录。
- 预览 record 元信息和通道信息。
- 播放、暂停、继续和停止回放。
- 使用进度条在播放过程中跳转。
- 显示选定时间范围和当前播放状态。

缩减版不包含完整版中的地图选择器和 `log2worldsim` 导出流程。

## 编译

在 Apollo 工作空间根目录执行（容器内通常为 `/apollo_workspace`）：

```bash
# 使用 Apollo buildtool
buildtool build -p modules/tools/cyber_recorder_gui --cpu
```

如果 Apollo 依赖已经安装，也可以直接使用 Bazel：

```bash
bazel build //modules/tools/cyber_recorder_gui:cyber_recorder_gui_lite
```

生成的可执行文件路径为：

```text
bazel-bin/modules/tools/cyber_recorder_gui/cyber_recorder_gui_lite
```

## 启动

直接运行：

```bash
./bazel-bin/modules/tools/cyber_recorder_gui/cyber_recorder_gui_lite
```

或者使用 `cyber_launch`：

```bash
cyber_launch start modules/tools/cyber_recorder_gui/launch/cyber_recorder_gui_lite.launch
```

程序启动时会初始化 Cyber 节点 `cyber_recorder_gui_lite`，窗口标题为 **Cyber Recorder GUI Lite**。

## 基本使用流程

1. 点击 **File** 选择单个 record 文件，或点击 **Dir** 选择一个目录。
2. 在信息框中检查 record 元信息。
3. 点击 **Play** 开始回放。
4. 使用 **Pause** / **Resume** 暂停或继续。
5. 播放时拖动或点击上方进度条进行跳转。
6. 点击 **Stop** 停止播放并将进度重置到起点。

目录选择框默认打开 `/apollo_workspace/data/`。目录模式只加载文件名匹配 `*.record` 或 `*.record.*` 的文件，并按文件名排序。建议分片文件保持自然排序，例如：

```text
demo.record
demo.record.00000
demo.record.00001
```

## 默认播放参数

| 参数 | 默认值 |
| --- | --- |
| 播放速率 | `1.0` |
| 起始时间 | record 起点 |
| 结束时间 | record 末尾（默认边界使用 `uint64_t` 最大值） |
| 预加载时间 | `3` 秒 |
| 循环播放 | 关闭 |
| 通道选择 | 未配置过滤条件时播放全部通道 |

下方双端滑条目前只更新 `Range` 文本，不会真正限制播放器的播放起止时间。

## 常见问题

### 启动后没有窗口

请确认容器已经正确透传宿主机的 X11 或 Wayland 显示环境，并且 Qt 能够连接显示服务。

### 无法打开 record 文件

请确认路径指向 Apollo Cyber record 文件，文件没有损坏，且当前用户具有读取权限。可以先使用 Apollo 自带的 record 工具检查文件。

### 其他模块收不到回放消息

确认 record 中包含目标通道，并确认订阅模块和 GUI 运行在同一个 Cyber 环境中。可以执行：

```bash
cyber_channel list
cyber_channel echo /your/channel/name
```

### 进度条无法跳转

只有在播放状态下才会处理跳转。请先点击 **Play**，再拖动或点击上方进度条。

## 源码结构

| 路径 | 用途 |
| --- | --- |
| `main.cpp` | Qt 程序入口及 Cyber 初始化 |
| `mainwindow.ui` | 主窗口布局 |
| `mainwindow.cpp` / `mainwindow.h` | 文件选择、播放控制和状态更新 |
| `info.cc` / `info.h` | record 元信息读取 |
| `player/` | 播放任务、缓冲和消息发布 |
| `launch/cyber_recorder_gui_lite.launch` | `cyber_launch` 启动配置 |
| `BUILD` | Bazel 构建目标和安装规则 |
| `cyberfile.xml` | Apollo 包元数据（`cyber-recorder-gui-lite`） |
