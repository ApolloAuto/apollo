# 软件包管理使用教程 - 基础概念与编译自定义组件和插件

前提条件

阅读使用该文档前请确保已经参考 [包管理安装方式](../安装指南/包管理安装方式.md) 文档 完成 Apollo 环境管理工具以及容器环境的安装

## 综述

本教程涵盖使用软件包管理下载，安装与构建 Apollo 各模块的基础知识。您将设置工作空间并构建一个简单的 C++ 项目，该项目将说明 Apollo 中软件包管理的关键概念，如 .workspace.json 和 .env 文件。完成本教程后，请参阅 [Apollo 研发工具 - buildtool](../框架设计/命令行工具/buildtool.md)，了解软件包管理中关键工具 buildtool 的高级用法。

在本教程中，您将学到：

- Apollo 软件包管理模式下的新增的配置信息
- 创建一个简单的示例组件，并且编译运行
- 安装现有软件包的源码到工作空间中，修改并进行编译
- 如何引入第三方库到工作空间中并且依赖该第三方库
- 将二次开发的模块打包，并部署到另一台主机上

## 开始之前

Apollo 在包管理模式下，以`工程`来组织和管理软件包，Apollo 官方提供了多个工程示例，这些工程可以在 Apollo 中的 github 主页中找到。在本教程中，我们使用 `application-pnc` 工程。您可以从 Apollo 的 github 库来获取示例工程：

```shell
git clone https://github.com/ApolloAuto/application-pnc.git
```

该工程声明依赖了和 pnc 相关的软件包，可以通过部署该工程来迅速调试、仿真 Apollo 的 pnc 软件包，该工程的目录结构如下：

```shell
application-pnc
├── .workspace.json
├── .env
├── WORKSPACE
├── core
│   ├── BUILD
│   └── cyberfile.xml
└── profiles
    └── default
```

如您所见，这个工程里面并无任何 Apollo 源码，仅有一些配置文件（.workspace.json和.env）与 core 模块，下面将通过部署该工程，逐步介绍上述提到的概念

## 部署 application-pnc

在部署整个工程之前，您需要先对工作空间的各个文件有一定了解。工作空间，即整个工程目录，存放着模块源码。它还包含一些供 Apollo 编译工具链识别的特殊文件：

`WORKSPACE`: 也即是 bazel 中的 WORKSPACE 文件，该文件会将该目录及其内容标识为 Bazel 工作区，并位于项目目录结构的根目录下。除非您需要额外引入可供bazel识别的第三方库，否则一般该文件不需要修改。

`.workspace.json`: 该文件同样位于项目的根目录下，标明了该工程从何处下载的软件包，以及软件包的版本，该文件内容如下：

```json
{ "repositories": [{ "name": "apollo-core", "version": "9.0.0-alpha2-r28" }] }
```

其中`repositories`字段中包含软件包仓库信息，`name`为软件包仓库名称，当前， Apollo 有两个仓库可供用户选择：

- apollo-core: Apollo 开源代码库的所有软件包仓库（X86架构）
- apollo-core-arm: Apollo 开源代码库的所有软件包仓库（ARM架构）

而`version`则是为该工程下载软件包版本。

当您想把整个工程从`9.0.0-alpha2-r28`版本升级到`9.0.0-alpha2-r29`版本时，简单将该文件修改至如下内容即可：

```json
{ "repositories": [{ "name": "apollo-core", "version": "9.0.0-alpha2-r29" }] }
```

然后重新部署工程，工程依赖的软件包就会自动升级至`9.0.0-alpha2-r29`版本。

> 注意：由于不同版本下 Apollo 各模块的对外接口可能会存在变更，因此升级版本后工程的源码可能需要做额外的适配工作

`.env`: Apollo 当前还需要运行在 docker 容器中，该文件指定了启动容器时使用的镜像的仓库以及tag等信息，该文件的内容如下：

```shell
APOLLO_ENV_NAME=pnc
APOLLO_ENV_WORKLOCAL=1
APOLLO_ENV_CONTAINER_REPO='registry.baidubce.com/apollo/apollo-env-gpu'
APOLLO_ENV_CONTAINER_TAG='9.0-latest'
```

其中：

- `APOLLO_ENV_NAME` 指定了容器的名称。实际开发中，经常会在一个开发环境中启动多个 Apollo 工程。通过配置该变量，解决了不同容器名称冲突的问题。
- `APOLLO_ENV_CONTAINER_REPO` 指定了镜像仓库
- `APOLLO_ENV_CONTAINER_TAG` 指定了镜像的tag

除上述提到的文件以外，该工程在 `core` 路径下还包括 `core` 这一模块的"源码"，尽管该模块不包括任何 c++ 传统意义上的源码。

在 Apollo 软件包管理模式中，一个模块的源码包括以下内容：

- `cyberfile.xml`(必须): 模块的描述文件，描述了该模块的名称、所在工作空间路径、依赖等版本信息
- `BUILD`（必须）: 即 bazel 的 BUILD 文件，描述了模块中 c++ 源码， python 源码，配置文件，数据文件等该组织成什么形式进行编译
- 一系列的源码，数据文件，配置文件等（可选）

接下来，将以 core 模块为例，简单对模块涉及的源码文件进行介绍

### 理解 cyberfile

cyberfile.xml 位于工作区模块的根目录下。包含 cyberfile.xml 文件的目录该模块的源码路径，其子目录不允许有 cyberfile.xml 文件，即软件包不支持嵌套。

core 的 cyberfile.xml 内容如下：

```xml
<package>
  <name>core</name>
  <version>local</version>
  <description>
      depends of apollo core
  </description>
  <maintainer email="apollo-support@baidu.com">Apollo Maintainer</maintainer>
  <type>module</type>
  <src_path>//core</src_path>
  <license>Apache License 2.0</license>
  <author>Apollo</author>

  <!-- basic -->
  <depend repo_name="common" type="binary">common</depend>
  <depend repo_name="cyber" type="binary">cyber</depend>
  <depend repo_name="common-msgs" type="binary">common-msgs</depend>

  <!-- dreamview && monitor -->
  <depend repo_name="dreamview" type="binary">dreamview</depend>
  <depend repo_name="monitor" type="binary">monitor</depend>
  <depend repo_name="studio-connector" type="binary">studio-connector</depend>
  <depend repo_name="sim-obstacle" type="binary">sim-obstacle</depend>

  <!-- external_command -->
  <depend repo_name="external-command-action" type="binary">external-command-action</depend>
  <depend repo_name="external-command-demo" type="binary">external-command-demo</depend>
  <depend repo_name="external-command-lane-follow" type="binary">external-command-lane-follow</depend>
  <depend repo_name="external-command-process" type="binary">external-command-process</depend>
  <depend repo_name="external-command-processor-base" type="binary">external-command-processor-base</depend>
  <depend repo_name="external-command-valet-parking" type="binary">external-command-valet-parking</depend>
  <depend repo_name="old-routing-adpter" type="binary">old-routing-adpter</depend>
  <depend repo_name="routing" type="binary">routing</depend>

  <!-- planning -->
  <depend repo_name="planning" type="binary">planning</depend>
  <depend repo_name="planning-scenario-bare-intersection-unprotected" type="binary">planning-scenario-bare-intersection-unprotected</depend>
  <depend repo_name="planning-scenario-emergency-pull-over" type="binary">planning-scenario-emergency-pull-over</depend>
  <depend repo_name="planning-scenario-emergency-stop" type="binary">planning-scenario-emergency-stop</depend>
  <depend repo_name="planning-scenario-lane-follow" type="binary">planning-scenario-lane-follow</depend>
  <depend repo_name="planning-scenario-park-and-go" type="binary">planning-scenario-park-and-go</depend>
  <depend repo_name="planning-lane-follow-map" type="binary">planning-lane-follow-map</depend>
  <depend repo_name="planning-scenario-pull-over" type="binary">planning-scenario-pull-over</depend>
  <depend repo_name="planning-scenario-stop-sign-unprotected" type="binary">planning-scenario-stop-sign-unprotected</depend>
  <depend repo_name="planning-scenario-traffic-light-protected" type="binary">planning-scenario-traffic-light-protected</depend>
  <depend repo_name="planning-scenario-traffic-light-unprotected-left-turn" type="binary">planning-scenario-traffic-light-unprotected-left-turn</depend>
  <depend repo_name="planning-scenario-traffic-light-unprotected-right-turn" type="binary">planning-scenario-traffic-light-unprotected-right-turn</depend>
  <depend repo_name="planning-scenario-valet-parking" type="binary">planning-scenario-valet-parking</depend>
  <depend repo_name="planning-scenario-yield-sign" type="binary">planning-scenario-yield-sign</depend>
  <depend repo_name="planning-task-fallback-path" type="binary">planning-task-fallback-path</depend>
  <depend repo_name="planning-task-lane-borrow-path" type="binary">planning-task-lane-borrow-path</depend>
  <depend repo_name="planning-task-lane-change-path" type="binary">planning-task-lane-change-path</depend>
  <depend repo_name="planning-task-lane-follow-path" type="binary">planning-task-lane-follow-path</depend>
  <depend repo_name="planning-task-open-space-fallback-decider" type="binary">planning-task-open-space-fallback-decider</depend>
  <depend repo_name="planning-task-open-space-pre-stop-decider" type="binary">planning-task-open-space-pre-stop-decider</depend>
  <depend repo_name="planning-task-open-space-roi-decider" type="binary">planning-task-open-space-roi-decider</depend>
  <depend repo_name="planning-task-open-space-trajectory-partition" type="binary">planning-task-open-space-trajectory-partition</depend>
  <depend repo_name="planning-task-open-space-trajectory-provider" type="binary">planning-task-open-space-trajectory-provider</depend>
  <depend repo_name="planning-task-path-decider" type="binary">planning-task-path-decider</depend>
  <depend repo_name="planning-task-path-reference-decider" type="binary">planning-task-path-reference-decider</depend>
  <depend repo_name="planning-task-path-time-heuristic" type="binary">planning-task-path-time-heuristic</depend>
  <depend repo_name="planning-task-piecewise-jerk-speed" type="binary">planning-task-piecewise-jerk-speed</depend>
  <depend repo_name="planning-task-piecewise-jerk-speed-nonlinear" type="binary">planning-task-piecewise-jerk-speed-nonlinear</depend>
  <depend repo_name="planning-task-pull-over-path" type="binary">planning-task-pull-over-path</depend>
  <depend repo_name="planning-task-reuse-path" type="binary">planning-task-reuse-path</depend>
  <depend repo_name="planning-task-rss-decider" type="binary">planning-task-rss-decider</depend>
  <depend repo_name="planning-task-rule-based-stop-decider" type="binary">planning-task-rule-based-stop-decider</depend>
  <depend repo_name="planning-task-speed-bounds-decider" type="binary">planning-task-speed-bounds-decider</depend>
  <depend repo_name="planning-task-speed-decider" type="binary">planning-task-speed-decider</depend>
  <depend repo_name="planning-task-st-bounds-decider" type="binary">planning-task-st-bounds-decider</depend>
  <depend repo_name="planning-traffic-rules-backside-vehicle" type="binary">planning-traffic-rules-backside-vehicle</depend>
  <depend repo_name="planning-traffic-rules-crosswalk" type="binary">planning-traffic-rules-crosswalk</depend>
  <depend repo_name="planning-traffic-rules-destination" type="binary">planning-traffic-rules-destination</depend>
  <depend repo_name="planning-traffic-rules-keepclear" type="binary">planning-traffic-rules-keepclear</depend>
  <depend repo_name="planning-traffic-rules-rerouting" type="binary">planning-traffic-rules-rerouting</depend>
  <depend repo_name="planning-traffic-rules-stop-sign" type="binary">planning-traffic-rules-stop-sign</depend>
  <depend repo_name="planning-traffic-rules-traffic-light" type="binary">planning-traffic-rules-traffic-light</depend>
  <depend repo_name="planning-traffic-rules-yield-sign" type="binary">planning-traffic-rules-yield-sign</depend>
  <depend repo_name="planning-traffic-rules-reference-line-end" type="binary">planning-traffic-rules-reference-line-end</depend>

  <!-- transform -->
  <depend repo_name="transform" type="binary">transform</depend>

  <!-- prediction -->
  <depend repo_name="prediction" type="binary">prediction</depend>

  <!-- control -->
  <depend repo_name="control-controller-demo-control-task" type="binary">control-controller-demo-control-task</depend>
  <depend repo_name="control-controller-lat-based-lqr-controller" type="binary">control-controller-lat-based-lqr-controller</depend>
  <depend repo_name="control-controller-lon-based-pid-controller" type="binary">control-controller-lon-based-pid-controller</depend>
  <depend repo_name="control-controller-mpc-controller" type="binary">control-controller-mpc-controller</depend>
  <depend repo_name="control" type="binary">control</depend>

  <!-- map -->
  <depend repo_name="map" type="binary">map</depend>

  <builder>bazel</builder>
</package>
```

其中，一些比较重要的属性：

- name: 模块的名称。
- version: 模块版本号，由于是本地源码，所以版本为local。
- type: 模块的类型，必须为module。
- src_path: 模块在工作空间下源码的路径。
- depend: 该模块的依赖，depend标签中有一些属性：
  - repo_name: 与依赖的模块名相同。
  - type: 该依赖是何种形式被引入的，分别有 src 与 binary 可选。src 为以源码形式引入该依赖，即编译工具链会将该依赖的源码拷贝到工作空间下，一并进行编译；binary 为以二进制形式引入该依赖，即编译工具链在编译当前模块时，会导入依赖模块的动态库和头文件进行编译。
- builder: 底层编译工具，当前仅支持 bazel。

### 理解 BUILD

BUILD 文件即 bazel 的 BUILD 文件，它告诉 Bazel 如何编译所需的输出，例如可执行的二进制文件或库。

在软件包管理模式中，Apollo 在 bazel 的基础上，提供了一些宏，规则，例如 core 的 BUILD 文件

```python
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_package()
```

其中 `apollo_package` 是软件包管理模式新增的宏，在软件包管理模式下，所有模块源码的BUILD文件中都需要添加该宏，保证模块源码可以正常的编译，部署。

除此之外，Apollo为c++源码、python源码、proto源码提供了相应的规则，例如在`cyber`源码的 BUILD 文件中：

```python
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

apollo_cc_library(
    name = "cyber_state",
    srcs = ["state.cc",],
    hdrs = ["state.h",],
    deps = ["//cyber/common:cyber_common",],
)
```

在软件包管理模式下，请使用 Apollo 提供的规则，这些规则分别有：

- apollo_cc_library：声明该 target 生成动态库，替代 bazel 原生 cc_library
- apollo_cc_binary：声明该 target 生成可执行文件，替代 bazel 原生 cc_binary
- apollo_cc_test：声明该 target 生成测试文件，替代 bazel 原生 cc_test
- apollo_component：声明该 target 生成一个 Apollo 组件
- apollo_plugin：声明该 target 生成一个 Apollo 插件

apollo_cc_library, apollo_cc_binary, apollo_cc_test规则的使用方式与 bazel 原生规则一致。

apollo_component, apollo_plugin将在下文中介绍如何使用这两个新增规则。

### 启动 Apollo 容器

启动 Apollo 容器需要使用 Apollo 环境管理工具 aem，该工具的安装可以查阅[包管理安装方式](../安装指南/包源码安装方式.md) 文档。

#### 启动容器

```shell
cd application-pnc && aem start
```

启动容器后 aem 会产生类似以下的输出：

```shell
The home directory `/home/apollo' already exists.  Not copying from `/etc/skel'.
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_neo_dev_pnc container, please run the following command:
[ OK ]   aem enter
[ OK ] Enjoy!
```

#### 进入容器环境

```shell
aem enter
```

当进入容器后，可以看见 `in-dev-docker` 字样。

#### 部署工程

上文提过，application-pnc 工程内部没有任何 c++ 源码，只有一个 core 模块的，并声明了该模块的依赖，通过编译该模块，就可以在当前环境下迅速下载需要的软件包，达到部署工程的目的。

通过以下命令来编译 core 模块：

```shell
buildtool build -p core
```

> -p 参数指定了要编译模块的路径，假如不指定该参数，buildtool将会编译工作空间下的所有模块

输入命令后，buildtool 将会自动下载依赖，这个过程根据网络状况，可能会持续 30 - 60 分钟，请耐心等待

```shell
[buildtool] 2023-10-27 14:15:04 INFO Import depends...
[buildtool] 2023-10-27 14:15:04 INFO Preprocess 3rd-osqp
[buildtool] 2023-10-27 14:15:04 INFO Install 3rd-osqp...
[buildtool] 2023-10-27 14:15:04 INFO install 3rd-osqp...
[buildtool] 2023-10-27 14:15:07 INFO 3rd-osqp successfully installed
[buildtool] 2023-10-27 14:15:07 INFO PostProcess 3rd-osqp
[buildtool] 2023-10-27 14:15:07 INFO Import depends...
[buildtool] 2023-10-27 14:15:07 INFO Preprocess libhdf5-dev
[buildtool] 2023-10-27 14:15:08 INFO Install libhdf5-dev...
[buildtool] 2023-10-27 14:15:16 INFO libhdf5-dev successfully installed
[buildtool] 2023-10-27 14:15:16 INFO PostProcess libhdf5-dev
```

buildtool 部署工程完毕后，当前环境已经安装了 pnc 仿真需要的软件和工具，可以启动 dreamview 来对 planning 进行调试

通过以下命令启动 dreamview

```shell
aem bootstrap
```

然后从浏览器访问 `localhost:8888` 就可以访问 dreamview ，启动仿真和 pnc 进行调试。

或者手动启动 planning component：

```shell
mainboard -d /apollo/modules/planning/planning_base/dag/planning.dag
```

#### 安装源码到工作空间

除部署以外，开发者可能想要查看或修改某个模块，插件的源码，可以通过以下命令安装某个模块：

```shell
buildtool install planning-task-lane-follow-path
```

该命令安装了 `planning-task-lane-follow-path` 的源码到工作空间下，源码可以在 `modules/planning/planning_base/tasks/lane_follow_path` 下找到。

您可以修改上述的代码，改变task的行为，然后编译使其生效。

```shell
buildtool build -p modules/planning/
```

上述命令会编译`modules/planning`路径下的所有模块。

然后再通过`dreamview`重启 planning ，或者命令行重新运行 planning 来进一步调试。

当您想舍弃本地编译的结果，恢复`planning-task-lane-follow-path`最原始的效果，可以通过以下命令来重装模块：

```shell
rm -f modules/planning/planning_base/tasks/lane_follow_path && buildtool reinstall planning-task-lane-follow-path
```

然后重启 planning 即可。

#### 创建一个 Apollo 组件

除了修改 Apollo 源码以外，您可能会想自己创建一个 Apollo 组件，也即是 component。

buildtool 提供了一个便捷的命令创建组建模版：

```shell
buildtool create --template component sample_component
```

> --template 参数声明了创建的模版是 component，sample_component 声明了该 component 的源码路径

执行完毕后，会有类似以下输出：

```shell
[buildtool] 2023-10-27 14:54:25 INFO Reconfigure apollo enviroment setup
template_vars:  {'package_path': 'sample_component', 'package_name': 'sample-component', 'target_name': 'sample_component', 'class_name': 'SampleComponent', 'base_class_name': 'SampleComponentBase', 'config_message_name': 'SampleComponentConfig', 'namespaces': [], 'includes': [], 'dependencies': [], 'build_dependencies': [], 'channel_name': '/apollo/sample_component', 'channel_message_type': 'apollo::SampleComponentMsg', 'package_type': 'module', 'email': 'sample@sample.com', 'author': 'Apollo Developer', 'description': 'This is a demo package', 'config_file_name': 'sample_component'}
[buildtool] 2023-10-27 14:54:25 INFO sample_component/proto/sample_component.proto create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/sample_component.h create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/sample_component.cc create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/proto/BUILD create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/BUILD create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/dag/sample_component.dag create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/launch/sample_component.launch create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/cyberfile.xml create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/conf/sample_component.pb.txt create success
[buildtool] 2023-10-27 14:54:25 INFO sample_component/conf/sample_component.conf create success
[buildtool] 2023-10-27 14:54:25 INFO create component package sample-component on sample_component success
```

工作空间下会多出了 sample_component 目录：

```shell
sample_component/
|-- BUILD
|-- conf
|   |-- sample_component.conf
|   `-- sample_component.pb.txt
|-- cyberfile.xml
|-- dag
|   `-- sample_component.dag
|-- launch
|   `-- sample_component.launch
|-- proto
|   |-- BUILD
|   `-- sample_component.proto
|-- sample_component.cc
`-- sample_component.h
```

其中：

- BUILD：告诉 bazel 如何编译这个组件
- conf：保存该 component 的配置文件，其中 sample_component.pb.txt 的内容会被解析成 sample_component.proto 定义的 SampleComponentConfig 类型；sample_component.conf 声明了一些 gflags
- dag, launch：保存该组件的启动文件
- proto：保存该组件配置，消息的proto message定义
- c++源码：描述该组件的初始化以及接收到相应消息的行为

该示例 component 会监听 `/apollo/sample_component` channel，接收到消息后在终端打出相应消息。

由于该示例是一个 component，因此在 BUILD 文件中使用了apollo_component规则：

```python
apollo_component(
    name = "libsample_component_component.so",
    srcs = [
        "sample_component.cc",
    ],
    hdrs = [
        "sample_component.h",
    ],
    deps = [
        "//cyber",
        "//sample_component/proto:sample_component_proto",
    ],
)
```

apollo_component规则声明了该 target 是一个组件，其中：

- name：该组件产出的动态库名称，与 dag 文件中读取的动态库名称一致
- srcs：该组件的源文件
- hdrs：该组件的头文件
- deps：该组件的依赖，在这里依赖了 cyber 的接口以及组件本身 proto 下的声明 target

该示例可以直接编译：

```shell
buildtool build -p sample_component
```

编译完成后直接运行即可：

```shell
mainboard -d sample_component/dag/sample_component.dag
```

您也可以修改该 component 初始化和接收到消息的行为，修改代码完毕后直接编译即可

#### 创建一个 Apollo 插件

与组件一致，可以使用 buildtool 的 create 命令来创建一个插件：

```shell
buildtool create --template plugin --base_class_name apollo::planning::Task sample_plugin --dependencies planning
```

> 插件需要继承一个插件基类，上述命令的 --base_class_name 参数声明继承 planning 中的 Task 类，sample_plugin 声明了该插件的源码路径，--dependencies 参数在 cyberfile.xml 中添加了对 planning 的依赖

工作空间会多出 sample_plugin 目录：

```shell
sample_plugin/
|-- BUILD
|-- conf
|   `-- sample_plugin.pb.txt
|-- cyberfile.xml
|-- plugin_sample_plugin_description.xml
|-- proto
|   |-- BUILD
|   `-- sample_plugin.proto
|-- sample_plugin.cc
`-- sample_plugin.h
```

相较于组件，插件增加了一个描述文件 `plugin_sample_plugin_description.xml`：

```xml
<library path="sample_plugin/libsample_plugin.so">
    <class type="apollo::SamplePlugin" base_class="apollo::planning::Task"></class>
</library>
```

该文件描述插件动态库的名称，以及插件的类名和集成的基类名，以便cyber能够正确加载该插件。

与组件不同，由于插件基类的定义不同，上述命令创建的插件模版仅是一个最基础的模版，无法直接编译，需要开发者补齐相应信息才能够编译，接下来将介绍开发者需要补齐的信息。

##### 引入的头文件

在 create 命令中，指定了继承的基类是 apollo::planning::Task，而实际上，sample_plugin.h 只是单纯继承了，apollo::planning::Task，并未引入相应的头文件：

```c++
#pragma once

#include <memory>
#include "cyber/plugin_manager/plugin_manager.h"

namespace apollo {

class SamplePlugin : public apollo::planning::Task {
 public:
  bool Init();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::SamplePlugin, apollo::planning::Task)

} // namespace apollo
```

开发者可以参考planning的接口文档来添加`apollo::planning::Task`对应的头文件，并根据基类定义修改函数签名：

```c++
#pragma once

#include <memory>
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/task_base/task.h"

namespace apollo {

class SamplePlugin : public apollo::planning::Task {
 public:
  bool Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<apollo::planning::DependencyInjector> &injector) override;

  apollo::common::Status Execute(apollo::planning::Frame *frame,
    apollo::planning::ReferenceLineInfo *reference_line_info) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::SamplePlugin, apollo::planning::Task)

} // namespace apollo
```

##### 修改源码实现文件

由于函数签名被修改了，相应的实现也需要对照修改，以下是 create 命令初始化的 cc 文件：

```c++
#include <memory>
#include "sample_plugin/sample_plugin.h"

namespace apollo {

bool SamplePlugin::Init() {
  return true;
}

} // namespace apollo
```

修改后的 cc 文件：

```c++
#include <memory>
#include "sample_plugin/sample_plugin.h"

namespace apollo {

bool SamplePlugin::Init(const std::string &config_dir, const std::string &name,
            const std::shared_ptr<apollo::planning::DependencyInjector> &injector) {
  return true;
}

apollo::common::Status SamplePlugin::Execute(apollo::planning::Frame *frame,
        apollo::planning::ReferenceLineInfo *reference_line_info) {
    apollo::planning::Task::Execute(frame, reference_line_info);
    return apollo::common::Status::OK();
}

} // namespace apollo
```

##### 修改BUILD文件，添加依赖的target

同样，插件的BUILD文件本身也没引入基类对应的target，开发者可以参考依赖模块的BUILD文件，来添加头文件对应的target

以下是 create 命令初始化的 BUILD 文件：

```python
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "sample_plugin_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libsample_plugin.so",
    srcs = [
        "sample_plugin.cc",
    ],
    hdrs = [
        "sample_plugin.h",
    ],
    description = ":plugin_sample_plugin_description.xml",
    deps = [
        "//cyber",
        "//sample_plugin/proto:sample_plugin_proto",

    ],
)

apollo_package()
```

`apollo::planning::Task`对应的target是`"//modules/planning/planning_base:apollo_planning_planning_base"`，因此修改 `BUILD` 文件后：

```python
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "sample_plugin_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libsample_plugin.so",
    srcs = [
        "sample_plugin.cc",
    ],
    hdrs = [
        "sample_plugin.h",
    ],
    description = ":plugin_sample_plugin_description.xml",
    deps = [
        "//cyber",
        "//sample_plugin/proto:sample_plugin_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
    ],
)

apollo_package()
```

这个 BUILD 文件中使用了 apollo_plugin 规则来声明该 target 生成一个插件，其中：

- name：该组件产出的动态库名称，与插件描述文件中读取的动态库名称一致
- description：插件描述文件的名称
- srcs：该组件的源文件
- hdrs：该组件的头文件
- deps：该组件的依赖，在这里依赖了 cyber 的接口以及组件本身 proto 下的声明 target

修改完毕后，就可以通过以下命令进行编译：

```shell
buildtool build -p sample_plugin
```

编译完毕后，该插件就可被 planning component 加载使用。按照上述步骤编译的插件只是一个空插件，插件本身没有任何的行为，要插件来规划车辆轨迹，还需要进一步编写相应的业务代码。另外，插件被 planning 加载并调用需要修改 planning 的配置文件，详细可参阅 planning 的入门教程，此处不再过多叙述。

#### 引入第三方库到工作空间中

目前为止，apollo支持引入以下类型的第三方库：

- 可通过apt下载的第三方库
- 以源码形式存在的第三方库
- 以头文件与动态库形式存在的第三方库

对于第一类第三方库，简单在需要依赖该第三方库模块的`cyberfile.xml`中添加 depend 的标签即可，假设您的模块依赖`curl`库，在`cyberfile.xml`中添加以下依赖信息：

```xml
<depend>libcurl4-openssl-dev</depend>
```

然后在该模块的 BUILD 文件中添加需要链接的动态库：

```python
apollo_cc_library(
    name = "sample_lib",
    srcs = [
        "sample.cc",
    ],
    hdrs = [
        "sample.h",
    ],
    copts = STUDIO_CONNECTOR_COPTS,
    linkopts = ["-lcurl"],
)
```

在源文件中添加相应"#include"即可：

```c
#include <curl/curl.h>
```

对于第二类第三方库，本质上和用户在工作空间开发的三方库无异，可当作普通模块处理。即用户为其编写`cyberfile.xml`与 `BUILD`文件即可正常编译与使用。

对于第三类第三方库，处理方式类似第二类，也可当作普通模块处理，但是在编写`BUILD`文件时，对于动态库需要额外的声明，例如以下的cancard驱动：

```shell
can_card
|-- BUILD
|-- cyberfile.xml
|-- include
|   `-- bcan.h
`-- lib
    `-- libbcan.so
```

`cyberfile.xml`中，需要依赖bazel-extend-tools, 3rd-bazel-skylib, 3rd-gpus这几个软件包：

```xml
<package format="2">
  <name>can-card</name>
  <version>local</version>
  <description>
    cancard Lib.
  </description>

  <maintainer email="apollo-support@baidu.com">Apollo</maintainer>
  <license>Apache License 2.0</license>

  <type>module</type>
  <depend>bazel-extend-tools</depend>
  <depend expose="False">3rd-bazel-skylib</depend>
  <depend expose="False">3rd-gpus</depend>
  <src_path url="https://github.com/ApolloAuto/apollo">//can_card</src_path>

</package>
```

`BUILD`中，对于动态库，需要使用原生的`cc_library`声明：
```python
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

native.cc_library(                  # 用 bazel 原生 cc_library 规则声明该第三方库的动态库
    name = "bcan",
    srcs = ["lib/libbcan.so"],
    tags = ["shared_library"],      # 标记该类型属于第三方动态库
    hdrs = ["include/bcan.h"],
)

apollo_cc_library(                  # 创建 apollo_cc_library 让其他模块可以依赖该 target
    name = "sample_can",
    deps = [":bcan"]                
)

apollo_package()
```

对于需要依赖该第三方库的模块，`cyberfile.xml`需添加对于该第三方库的依赖：
```xml
<depend type="binary" repo_name="can-card">can-card</depend>
```

`BUILD`文件中直接依赖该第三方库声明的apollo_cc_library即可：
```python
apollo_cc_library(
    name = "drivers-sample-canbus",
    hdrs = ["sample-canbus.h"],
    srcs = ["sample-canbus.cc"],
    deps = ["//can_card:sample_can"],
)
```

然后正常编译即可。

#### 将本地开发模块打包并部署到其他主机上

开发者在本地开发完毕后，可能需要将本机开发的模块进行打包，并部署到另一台主机的需求，这里以上述编译的插件为例，介绍软件包管理模式下如何打包与部署。

##### 打包

buildtool通过以下命令对模块进行打包：

```shell
buildtool release -p sample_plugin core
```

> -p 参数指明了要打包模块的源码路径，当不添加该参数时，buildtool 会尝试将工作空间下所有模块进行打包

打包完成后，buildtool 会有类似输出：

```shell
[buildtool] 2023-10-27 19:21:53 INFO update the local cache
[buildtool] 2023-10-27 19:21:53 INFO update complete
[buildtool] 2023-10-27 19:21:53 INFO Reconfigure apollo enviroment setup
[buildtool] 2023-10-27 19:21:53 INFO use src code of planning-task-lane-follow-path in workspace
[buildtool] 2023-10-27 19:21:56 INFO Analyzing dependencies topological graph...
[buildtool] 2023-10-27 19:21:56 INFO building package process started, it will take while...
dpkg-deb: building package 'apollo-neo-sample-plugin' in 'apollo-neo-sample-plugin_9.0.0-alpha2-r28_amd64.deb'.
dpkg-deb: building package 'apollo-neo-core' in 'apollo-neo-core_9.0.0-alpha2-r28_amd64.deb'.
[buildtool] 2023-10-27 19:21:57 INFO Done! Productions is in /apollo_workspace/.deb_local
[buildtool] 2023-10-27 19:21:57 INFO Compress the release files...
[buildtool] 2023-10-27 19:21:57 INFO Release complete, the output files: release.tar.gz
```

工作空间下会产生一个 release.tar.gz 文件，该文件可用于后续到其他主机上部署。

##### 部署

> 在部署之前，确保另外一台主机下载好了 aem 工具来启动 Apollo 容器

首先，创建一个用于部署用的工作空间：

```shell
mkdir deployment_workspace
```

然后将上一步的 release.tar.gz 与 .env 文件拷贝到新的主机中

接下来同样通过 aem 启动容器

```shell
cd deployment_workspace && aem start
```

启动完毕后，在容器内调用 buildtool 部署工程：

```shell
buildtool deploy -f release.tar.gz
```

这个过程buildtool会自动部署 sample-plugin 与 core 这两个模块，并自动从网络下载需要的依赖，根据网络状况，这个过程可能需要30-60分钟不等。

部署完毕后，就可以通过 `aem bootstrap start` 命令启动 dreamview，或手动通过 mainboard 启动 planning，就和刚才开发的环境一样。

## 接下来...

恭喜！通过阅读本文档，您已经掌握了 Apollo 软件包管理的一些基础概念与开发模式，接下来您可以：

- 阅读 [aem](../框架设计/命令行工具/aem.md) 研发工具介绍来进一步了解 aem 对容器管理的功能

- 阅读 [buildtool](../框架设计/命令行工具/buildtool.md) 研发工具来进一步了解 buildtool 的高级使用方式

- 阅读 Apollo 各模块的教程来学习如何二次开发
