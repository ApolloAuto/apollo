# cyber_recorder_gui_simple 使用说明

`cyber_recorder_gui_simple` 是一个基于 Qt 的 Apollo Cyber record 简易回放工具。它可以加载单个 `.record` 文件，或加载目录下按文件名排序的 `.record` / `.record.*` 文件，并把 record 中的消息重新发布到 Cyber 通道中，便于调试感知、定位、规划等依赖历史数据的模块。

## 适用场景

- 查看 record 文件基础信息，例如时间范围、大小、消息数、通道列表和消息类型。
- 通过图形界面播放、暂停、继续、停止 record。
- 在播放过程中拖动进度条跳转到指定进度。
- 一次加载一个目录中的连续分片 record 文件。

## 构建

在 Apollo 工作空间根目录执行：

```bash
bazel build //modules/cyber_recorder_gui_simple:cyber_recorder_gui_simple
```

如果工程使用 AEM / buildtool 工作流，请先进入 Apollo 容器并完成依赖安装，再执行上面的 Bazel 构建命令。

## 启动

构建完成后，可以直接运行二进制：

```bash
./bazel-bin/modules/cyber_recorder_gui_simple/cyber_recorder_gui_simple
```

也可以通过 `cyber_launch` 启动：

```bash
cyber_launch start modules/cyber_recorder_gui_simple/launch/cyber_recorder_gui_simple.launch
```

窗口标题为 `Cyber Recorder GUI Simple`。程序启动后会初始化 Cyber 节点 `cyber_recorder_gui_simple`。

## 界面说明

| 控件 | 说明 |
| --- | --- |
| `File` | 选择单个 record 文件。选择成功后会读取并显示该文件信息。 |
| `Dir` | 选择一个目录，自动加载目录下的 `.record` 和 `.record.*` 文件。默认打开路径为 `/apollo_workplace/data/`。 |
| `Play` | 开始播放已加载的 record。播放后按钮文字会变为 `Pause`。 |
| `Pause` | 暂停当前播放。暂停后按钮文字会变为 `Resume`。 |
| `Resume` | 从暂停位置继续播放。 |
| `Stop` | 停止播放，并将播放进度重置到起点。 |
| 上方进度条 | 显示当前播放进度；播放过程中可拖动或点击跳转。 |
| 下方双端滑条 | 显示选中的时间范围文本。当前版本只更新 `Range` 显示，不会实际裁剪播放区间。 |
| 信息框 | 显示 record 文件信息、通道列表和加载状态。 |
| 状态栏 | 显示 `Ready`、`Loading record files`、`Playing`、`Paused`、`Stopped` 等状态。 |

## 基本使用流程

1. 启动 GUI：

   ```bash
   ./bazel-bin/modules/cyber_recorder_gui_simple/cyber_recorder_gui_simple
   ```

2. 点击 `File` 选择单个 record 文件，或点击 `Dir` 选择 record 目录。

3. 确认信息框中显示 record 信息。目录模式下会显示加载文件数量，并预览第一个 record 文件的信息。

4. 点击 `Play` 开始回放。

5. 播放过程中可以：

   - 点击 `Pause` 暂停。
   - 点击 `Resume` 继续。
   - 拖动上方进度条跳转播放位置。
   - 点击 `Stop` 停止并回到起点。

6. 使用其他 Cyber 工具或业务模块订阅对应通道，检查回放消息。例如：

   ```bash
   cyber_channel list
   cyber_channel echo /your/channel/name
   ```

## record 文件加载规则

### 单文件模式

点击 `File` 后选择一个 record 文件。程序会：

1. 清空之前加载的文件列表。
2. 读取该 record 的基础信息和通道索引。
3. 初始化播放器。

### 目录模式

点击 `Dir` 后选择目录。程序会：

1. 搜索目录下的普通文件。
2. 只加载文件名匹配 `*.record` 或 `*.record.*` 的文件。
3. 按文件名排序后加入播放列表。
4. 预览第一个 record 文件的信息。

建议连续分片文件保持自然排序，例如：

```text
demo.record
demo.record.00000
demo.record.00001
demo.record.00002
```

## 当前默认播放参数

当前界面没有暴露命令行参数或高级设置，源码中的默认播放参数如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| 播放速率 | `1.0` | 原速播放。 |
| 起始时间 | `0` | 从 record 起点开始。 |
| 结束时间 | `uint64_t` 最大值 | 默认播放到 record 末尾。 |
| 预加载时间 | `3` 秒 | 播放任务预加载时间。 |
| 循环播放 | `false` | 默认不循环。 |
| 通道过滤 | 全通道 | 未配置白名单或黑名单时播放全部通道。 |

## 常见问题

### 启动后没有窗口

确认当前环境支持图形界面显示。如果在容器中运行，需要确保 X11 / Wayland 显示环境已经正确透传。

### 选择文件后提示读取失败

可能原因：

- 选择的文件不是 Apollo Cyber record 格式。
- 文件损坏或 record 索引不完整。
- 当前用户没有读取权限。

可以先用 Cyber 自带工具检查 record 文件：

```bash
cyber_recorder info /path/to/file.record
```

### 点击 `Play` 后其他模块收不到消息

检查以下内容：

- record 文件中是否包含目标通道。
- 订阅模块是否已经启动并连接到同一个 Cyber 环境。
- 使用 `cyber_channel list` 确认通道是否出现。
- 使用 `cyber_channel echo <channel>` 直接观察消息。

### 进度条跳转不生效

上方播放进度条的跳转只在播放状态下生效。先点击 `Play`，再拖动或点击进度条。

### 双端范围条没有裁剪播放

这是当前实现的限制。双端范围条目前只用于计算并显示 `Range: 开始时间 - 结束时间`，没有把选中范围写回播放器的 `begin_time_ns` / `end_time_ns`。

## 相关文件

| 文件 | 说明 |
| --- | --- |
| `main.cpp` | Qt 应用入口，并初始化 / 清理 Cyber。 |
| `mainwindow.ui` | 主窗口 UI 布局。 |
| `mainwindow.cpp` | 文件选择、播放控制、进度刷新和状态更新逻辑。 |
| `info.cc` | 读取并格式化 record 文件信息。 |
| `player/` | record 播放、任务生产、任务消费和缓冲逻辑。 |
| `launch/cyber_recorder_gui_simple.launch` | `cyber_launch` 启动配置。 |
| `BUILD` | Bazel 构建配置。 |
