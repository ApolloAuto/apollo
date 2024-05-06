## Cyber RT 介绍

Apollo Cyber RT 是一个开源、高性能的运行时框架，专为自动驾驶场景而设计。针对自动驾驶的高并发、低延迟、高吞吐量进行了大幅优化。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/image_36f0fb8.png)

使用 Apollo Cyber RT 的主要好处：

- 加速开发
  - 具有数据融合功能的定义明确的任务接口
  - 一系列开发工具
  - 大量传感器驱动程序
- 简化部署
  - 高效自适应的消息通信
  - 具有资源意识的可配置用户级调度程序
  - 可移植，依赖更少
- 赋能自动驾驶
  - 默认的开源运行时框架
  - 为自动驾驶搭建专用模块

### 常用术语

#### Component

在自动驾驶系统中，模块（如感知、定位、控制系统...）在 Cyber RT 下以 Component 的形式存在。不同 Component 之间通过 Channel 进行通信。Component 概念不仅解耦了模块，还为将模块拆分为多个子模块提供了灵活性。

#### Channel

Channel 用于管理 Cyber RT 中的数据通信。用户可以发布/订阅同一个 Channel，实现 p2p 通信。

#### Task

Task 是 Cyber RT 中异步计算任务的抽象描述。

#### Node

Node 是 Cyber RT 的基本组成部分；每个模块都包含一个 Node 并通过 Node 进行通信。通过在节点中定义 Reader/Writer 或 Service/Client，模块可以具有不同类型的通信形式。

#### Reader/Writer

Reader/Writer 通常在 Node 内创建，作为 Cyber RT 中的主要消息传输接口。

#### Service/Client

除了 Reader/Writer 之外，Cyber RT 还提供了用于模块通信的 Service/Client 模式。它支持节点之间的双向通信。当对服务发出请求时，客户端节点将收到响应。

#### Parameter

参数服务在 Cyber RT 中提供了全局参数访问接口。它是基于 Service/Client 模式构建的。

#### 服务发现

作为一个去中心化的框架，Cyber RT 没有用于服务注册的主/中心节点。所有节点都被平等对待，可以通过“服务发现”找到其他服务节点。使用`UDP`用来服务发现。

#### CRoutine

参考协程（Coroutine）的概念，Cyber RT 实现了 Coroutine 来优化线程使用和系统资源分配。

#### Scheduler

为了更好地支持自动驾驶场景，Cyber RT 提供了多种资源调度算法供开发者选择。

#### Message

Message 是 Cyber RT 中用于模块之间数据传输的数据单元。

#### Dag文件

Dag 文件是模块拓扑关系的配置文件。您可以在 dag 文件中定义使用的 Component 和上游/下游通道。

#### Launch文件

Launch 文件提供了一种启动模块的简单方法。通过在launch文件中定义一个或多个 dag 文件，可以同时启动多个模块。

#### Record文件

Record 文件用于记录从 Cyber RT 中的 Channel 发送/接收的消息。回放 Record 文件可以帮助重现Cyber RT之前操作的行为。

#### Mainboard

Cyber RT 的主入口，可以通过`mainboard -d xxx.dag`来启动一个模块进程。

### 开发工具

CyberRT 框架同时也提供了一系列实用的工具用来辅助日常开发, 包括命令行工具 cyber_monitor cyber_recorder 等。

> 说明：这些工具需要运行在 Apollo Docker 环境内。

所有上述工具都依赖于 CyberRT 软件库，所以在使用这些工具之前，你需要通过如下方式来配置 CyberRT 工具的运行环境：

```bash
username@computername:~$: source /apollo/cyber/setup.bash
```

下面将着重介绍 cyber_monitor 的使用方法。

#### 使用 Cyber Monitor 查看 Channel 数据实践

命令行工具`cyber_monitor`提供了终端中实时显示channel信息列表的功能。

##### 运行 Cyber Monitor

在 Apollo Docker 环境中执行如下命令运行`cyebr_monitor`：

```bash
username@computername:~$: cyber_monitor
```

##### 实用命令

可以通过 `-h` 选项来获取帮助信息：

```bash
username@computername:~$: cyber_monitor -h
```

使用 `-c` 选项，可以让 `cyber_monitor` 只监测一个指定的 channel 信息：

```bash
username@computername:~$: cyber_monitor -c ChannelName
```

##### 熟悉 Cyber Monitor 界面

在启动`cyber_monitor`之后，会在终端中显示一个交互界面。`cyber_monitor`自动从拓扑中收集所有 channel 的信息并分两列显示（channel 名称，数据频率）

channel 信息默认显示为红色，当有数据流经 channel 时，对应的行就会显示成绿色，如下图所示：

![cyber_monitor.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/cyber_monitor_4365395.png)

##### 与 Cyber Monitor 交互

###### 命令介绍

```
ESC | q key ---- 退出
Backspace ---- 后退
h | H ---- 显示帮助页
PageDown | Ctrl+d ---- 上一页
PageUp | Ctrl+u ---- 下一页
Up, down or w, s keys ---- 上下移动当前的高亮行
Right arrow or d key ---- 进入高亮行, 显示高亮行数据的详细信息
Left arrow or a key ---- 从当前界面返回上一层界面
Enter key ---- 与d键相同
f | F ---- 显示数据帧频率
t | T ---- 显示channel消息类型
Space ---- 关闭|开启 channel (仅在channel有数据到达时有效; channel关闭后会变成黄色)
i | I ---- 显示channel的Reader和Writer信息
b | B ---- 显示channel消息内容
n | N ---- 显示消息中RepeatedField的下一条数据
m | M ---- 显示消息中RepeatedField的上一条数据
```
