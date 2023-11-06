# Apollo 研发工具 - buildtool

## 综述

Apollo buildtool是一个命令行工具，提供编译、测试、安装或运行 Apollo 模块等功能。基于 buildtool ，不仅可以方便地安装Apollo 中各个模块的二进制包，还可以对这些源代码进行二次开发、编译和测试，而不需要下载整个 Apollo。 buildtool 可以让开发者只关注需要开发的模块，提高整体开发效率。

## 安装

buildtool 目前只支持在 Apollo 环境容器中运行。如果你还没有启动容器，你可以使用 apollo 环境工具：`aem` 来启动或进入一个 Apollo 环境容器。

Apollo 环境容器已经预装了 Apollo buildtool。当你进入容器时，你可以输入以下命令来检查 buildtool 是否已经正确安装。

```shell
buildtool -v
```

如果一切顺利，你会看到 buildtool 当前安装的版本：

```shell
buildtool -v                                                                                
9.0.0-alpha3-r3
```

你也可以通过以下命令来重新安装或更新 buildtool

```shell
sudo apt update -y && sudo apt install --only-upgrade apollo-neo-buildtool
```

## 使用说明

### buildtool 子命令

buildtool 的子命令是 buildtool 提供的所有功能的入口。调用遵循以下形式：

```shell
buildtool <action> [action arguments or options]
```

buildtool 不同功能被组织成不同的 action ，类似于常见的命令行工具，如 git 或 apt。

action 涵盖了 buildtool 的所有功能，如 build 负责编译， install 负责安装， clean 负责清理编译缓存等。

一些 action 可能需要额外的参数，你可以在 action 后输入 `-h` 或 `--help` 来查看详细的参数

### 当前的子命令

- config - 配置 buildtool
- build - 编译并安装工作空间下的源码
- test - 运行工作空间下各模块的单元测试
- install - 安装特定模块的源码到工作空间
- reinstall - 重装特定模块的软件包
- clean - 清除工作空间下的编译缓存
- init - 初始化工作空间
- info - 查看特定软件包的信息
- bootstrap - 启动模块，目前仅支持 `dreamview` 和 `monitor`
- create - 按模板创建包
- release - 将编译产物打包成可分发的格式
- deploy - 部署由 `release` 打包而成所包

### login

login 用于企业用户需要安装非 apollo-core 仓库的软件包时的登陆鉴权

```shell
buildtool login username password
```

如果账号密码不匹配，buildtool会输出以下错误信息：

```shell
[apollo@in-dev-docker:~/tmp]$ buildtool login **** ****
[buildtool] 2023-10-25 14:42:48 INFO Reconfigure apollo enviroment setup
[buildtool] 2023-10-25 14:42:48 ERROR Encounter ErrCode.NetworkIoError
[buildtool] 2023-10-25 14:42:48 ERROR hint: login failed, status: 401
```

如果一切正常，buildtool会输出以下信息：

```shell
[apollo@in-dev-docker:~/tmp]$ buildtool login **** *******
[buildtool] 2023-10-24 12:12:26 INFO Reconfigure apollo enviroment setup
```

### config

config 用于给 buildtool 配置用户级别的设置，用户配置会覆盖默认配置

用户配置又分全局配置与本地配置

- 全局配置即对应 `$HOME/.apollo/buildtool/config` 配置文件中的设定
- 本地配置即对应 `$PWD/.buildtool.conf` 配置文件中的设定

#### 用法

##### 设置编译使用的指令集

```shell
buildtool config set -- compile.march -march=native
```

> 注：因为值带 `-` 前缀，避免参数解析异常，需要将 `set` 后增加 `--`

##### 详细参数

```shell
usage: buildtool config [-h] {set,get} ...

positional arguments:
  {set,get}   config command
    set       set buildtool config
    get       get buildtool config

optional arguments:
  -h, --help  show this help message and exit
```

### build

build 负责编译工作空间中一个或多个软件包的源码， build 只能在工作空间根目录下执行。如果在工作空间之外或在工作空间的非根目录下执行， build 将抛出一个错误

#### 基本用法

##### 编译工作空间下所有模块的源码

```shell
buildtool build
```

当没有参数时，buildtool将编译工作空间的所有源码。在编译过程中，buildtool 会自动创建 dev、third_party、tools 文件夹来保存编译信息。

##### 编译工作空间下指定路径的模块的源码

当接收到 `-p` 或 `--packages` 参数时， buildtool 可以单独编译指定路径下的源码

```shell
# 编译 modules 下所有模块
[apollo@in-dev-docker:~/tmp]$ mkdir demo && cd demo
[apollo@in-dev-docker:~/tmp]$ buildtool init
[apollo@in-dev-docker:~/tmp]$ tree
.
|-- WORKSPACE
|-- example_components
|   |-- BUILD
|   |-- common_component_example.cc
|   |-- common_component_example.h
|   |-- cyberfile.xml
|   |-- dag
|   |   `-- example.dag
|   |-- launch
|   |   `-- example.launch
|   |-- message_header_test.cc
|   |-- proto
|   |   |-- BUILD
|   |   `-- examples.proto
|   |-- timer_common_component_example.cc
|   `-- timer_common_component_example.h
`-- example_lib
    |-- BUILD
    |-- cyberfile.xml
    |-- external_lib.cc
    `-- external_lib.h
[apollo@in-dev-docker:~/tmp]$ buildtool build -p example_components example_lib
```

上述命令初始化了工作空间，创建了一个组件示例，并编译了示例源码，如果一切顺利，在编译结束后，你会看到以下提示：

```shell
...
-- Installing: /opt/apollo/neo/include/example_components/proto/examples.pb.cc
-- Installing: /opt/apollo/neo/include/example_components/proto/lib_examples_proto.a
-- Installing: /opt/apollo/neo/include/example_components/proto/lib_examples_proto.so
-- Installing: /opt/apollo/neo/python/example_components/proto/examples_pb2.py
-- Installing: /opt/apollo/neo/lib/example_components/proto/lib_examples_proto_ep_bin.so
[buildtool] 2023-10-25 14:49:05 INFO PostProcess example-components
[apollo@in-dev-docker:~/tmp]$
```

#### 进阶用法

##### 以opt gpu模式对源码进行编译

```shell
buildtool build --gpu --opt
```

##### 传递编译参数给 bazel

```shell
buildtool build --arguments="--linkopts=-lz4"
```

##### 清除前一次编译缓存后再进行编译

```shell
buildtool build --expunge
```

##### 指定编译使用的进程数和使用内存的百分比

```shell
buildtool build --jobs 4 --memories 0.5
```

上述命令使用了使用了4个进程与当前宿主机的50%内存进行编译

##### 详细参数

```shell
usage: buildtool build [-h] [-p [* [* ...]]] [-a [* [* ...]]] [--gpu] [--cpu]
                       [--dbg] [--opt] [--prof] [--teleop] [--expunge]
                       [-j JOBS] [-m MEMORIES] [--ci-force-gpu]
                       [--install_dep_only]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --packages [* [* ...]]
                        Specify the package path.
  -a [* [* ...]], --arguments [* [* ...]]
                        Pass arguments to the build system.
  --gpu                 Run build in GPU mode
  --cpu                 Run build in cpu mode
  --dbg                 Build with debugging enabled
  --opt                 Build with optimization enabled
  --prof                Build with profiler enabled
  --teleop              Run build with teleop enabled
  --expunge             Expunge the building cache before build
  -j JOBS, --jobs JOBS  Specifies the number of threads to compile in parallel
  -m MEMORIES, --memories MEMORIES
                        Specifies the percentage of memory used by compilation
  --ci-force-gpu        Force using gpu mode to build(ci only)
  --install_dep_only    Specifies only install depends
```

### test

这个命令对工作区的源代码进行单元测试，实际执行的是源码中BUILD文件定义的所有apollo_cc_test

#### 用法

进行单元测试:

```shell
buildtool init
buildtool install planning
buildtool test -p modules/planning
```

上述操作buildtool将下载planning模块，将planning模块的源码复制到工作空间，最后进行编译和测试。

##### 指定gpu模式编译和测试

```shell
buildtool test --gpu --p modules/planning
```

##### 传递参数到 bazel

```shell
buildtool test --a "--linkopts=-llz4" --p modules/planning
```

##### 详细参数

```shell
usage: buildtool test [-h] [-p [* [* ...]]] [--gpu] [--cpu]
                      [--arguments [* [* ...]]]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --package_paths [* [* ...]]
                        Specify the package path.
  --gpu                 Run build in gpu mode
  --cpu                 Run build in cpu mode
  --arguments [* [* ...]]
                        Pass arguments to the build system.
```

### install

这个命令负责安装特定模块的软件包/源码

#### 用法

```shell
buildtool install planning
```

上述操作 buildtool 将下载 planning 模块，将 planning 模块的源码复制到工作空间。

```shell
buildtool install --legacy planning
```

上述操作 buildtool 只下载 planning 模块，而不把源码复制到工作区。

##### 详细参数

```shell
usage: buildtool install [-h] [--legacy] [packages [packages ...]]

positional arguments:
  packages Install the packages

optional arguments:
  -h, --help show this help message and exit
  --legacy legacy way to install package
```

### reinstall

这个命令负责重新安装特定模块的软件包

#### 用法

```shell
buildtool reinstall planning
```

上述操作 buildtool 将下载 planning 模块，将 planning 模块的源码复制到工作空间。

##### 详细参数

```shell
usage: buildtool reinstall [-h] [packages [packages ...]]

positional arguments:
  packages    Reinstall the packages

optional arguments:
  -h, --help  show this help message and exit
```

### clean

这个action是用来清除工作空间源码的编译缓存和产出。

#### 用法

clean action 在不接受任何参数的情况下，会清除工作区的所有编译缓存，这相当于执行 `bazel clean --expunge`

```shell
buildtool clean
```

`packages_path` 参数可以指定删除相应路径下的模块源码的编译产出:

```shell
buildtool clean --packages_path modules/planning
```

上述操作将删除planning模块的编译输出。

`expunge` 参数将删除机器上所有模块源码的编译产出。

```shell
buildtool clean --expunge
```

##### 详细参数

```
usage: buildtool clean [-h] [-p [* [* ...]]] [-e]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --packages_path [* [* ...]]
                        clean specified module build production.
  -e, --expunge         clean the build cache including production
```

### init

这个 action 负责初始化一个工作空间。

#### 用法

当 init action 没有任何参数时，会在当前目录下创建一个基本工作空间:

```shell
buildtool init
```

也可以在init action中加入 -w 参数来把 example_component 源码复制到到工作空间：

```shell
buildtool init -w
```

`path` 参数可以指定工作空间的创建路径:

```shell
buildtool init --path ～/demo
```

##### 详细参数

```shell
usage: buildtool init [-h] [-p PATH] [-w]

optional arguments:
  -h, --help            show this help message and exit
  -p PATH, --path PATH  specify workspace path
  -w, --with-examples   init workspace with example component and lib
```

### info

这个 action 用于列出指定软件包的依赖。

#### 用法

##### 列出所请求软件包的直接依赖：

```shell
buildtool info planning-dev
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO using gpu: False
[buildtool] INFO using debug mode: False
[buildtool] INFO According parameters above to analysis depends of planning-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO planning-dev directly depends on the following packages:
[buildtool] INFO (3rd-gflags-dev|1.0.0.1), (3rd-absl-dev|1.0.0.1), (3rd-osqp-dev|1.0.0.1), (3rd-glog-dev|1.0.0.1)
[buildtool] INFO (3rd-proj-dev|1.0.0.1), (libtinyxml2-dev|None), (3rd-boost-dev|1.0.0.1), (3rd-opencv-dev|1.0.0.1)
[buildtool] INFO (3rd-ipopt-dev|1.0.0.1), (3rd-eigen3-dev|1.0.0.1), (libadolc-dev|None), (3rd-ad-rss-lib-dev|1.0.0.1)
[buildtool] INFO (cyber-dev|1.0.0.1), (common-dev|1.0.0.1), (map-dev|1.0.0.2), (common-msgs-dev|1.0.0.1)
[buildtool] INFO (bazel-extend-tools-dev|1.0.0.1), (3rd-mkl-dev|1.0.0.1), (3rd-libtorch-cpu-dev|1.0.0.1), (3rd-protobuf-dev|1.0.0.1)
[buildtool] INFO (3rd-rules-python-dev|1.0.0.1), (3rd-grpc-dev|1.0.0.1), (3rd-bazel-skylib-dev|1.0.0.1), (3rd-rules-proto-dev|1.0.0.1)
[buildtool] INFO (3rd-py-dev|1.0.0.1), (3rd-gpus-dev|1.0.0.1), (3rd-gtest-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

##### 列出所请求软件包的所有依赖：

```shell
buildtool info --with-indirect planning-dev
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO using gpu: False
[buildtool] INFO using debug mode: False
[buildtool] INFO According parameters above to analysis depends of planning-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO planning-dev depends on these following packages:
[buildtool] INFO (3rd-gflags-dev|1.0.0.1), (3rd-absl-dev|1.0.0.1), (3rd-osqp-dev|1.0.0.1), (3rd-glog-dev|1.0.0.1)
[buildtool] INFO (3rd-proj-dev|1.0.0.1), (libtinyxml2-dev|None), (3rd-boost-dev|1.0.0.1), (3rd-opencv-dev|1.0.0.1)
[buildtool] INFO (3rd-ipopt-dev|1.0.0.1), (3rd-eigen3-dev|1.0.0.1), (libadolc-dev|None), (3rd-ad-rss-lib-dev|1.0.0.1)
[buildtool] INFO (cyber-dev|1.0.0.1), (libncurses5-dev|None), (libuuid1|None), (3rd-rules-python-dev|1.0.0.1)
[buildtool] INFO (3rd-grpc-dev|1.0.0.1), (3rd-rules-proto-dev|1.0.0.1), (3rd-py-dev|1.0.0.1), (3rd-bazel-skylib-dev|1.0.0.1)
[buildtool] INFO (3rd-protobuf-dev|1.0.0.1), (3rd-fastrtps-dev|1.0.0.1), (common-msgs-dev|1.0.0.1), (bazel-extend-tools-dev|1.0.0.1)
[buildtool] INFO (common-dev|1.0.0.1), (libsqlite3-dev|None), (3rd-gtest-dev|1.0.0.1), (3rd-nlohmann-json-dev|1.0.0.1)
[buildtool] INFO (map-dev|1.0.0.2), (3rd-mkl-dev|1.0.0.1), (3rd-libtorch-cpu-dev|1.0.0.1), (3rd-gpus-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

##### 列出指定路径下软件包源码的依赖信息：

```shell
buildtool info --directory example_components/
[buildtool] INFO Reconfigure apollo enviroment setup
[buildtool] INFO Compile parameters:
[buildtool] INFO using gpu: False
[buildtool] INFO using debug mode: False
[buildtool] INFO According parameters above to analysis depends of example-components-dev
[buildtool] INFO Analyzing dependencies topological graph...
[buildtool] INFO example-components-dev directly depends on the following packages:
[buildtool] INFO (cyber-dev|1.0.0.1), (bazel-extend-tools-dev|1.0.0.1), (3rd-protobuf-dev|1.0.0.1)
[buildtool] INFO Done, Enjoy!
[buildtool] INFO apollo build tool exit.
```

##### 详细参数

```
usage: buildtool info [-h] [--depends-on] [--with-indirect] [--directory]
[--gpu] [--cpu] [--dbg]
[query]

positional arguments:
[query] the package name or stored path of which package's depends
you want to list

optional arguments:
-h, --help show this help message and exit
--depends-on list those packages information which are directly
dependent on the package
--with-indirect list the package all depends, direct and indirect
--directory list the package infomation which stored in the directory
--gpu with compilation in GPU parameter
--cpu with compilation in CPU parameter
--dbg with compilation in debugging parameter
```

### bootstrap

bootstrap 用于启动特定软件包的最新版本。最新版本指的是与软件包的最后一次操作相对应的版本。如果软件包的最后一次操作是编译，那么 bootstrap 将启动编译后的软件包的编译产出；如果软件包的最后一次操作是安装或重装 ，例如执行 `buildtool reinstall` 这样的命令，那么 `bootstrap` 将启动安装或重新安装的软件包的二进制文件或动态链接库。注意：目前，bootstrap可以启动的模块有：`dreamview`、`monitor`。

#### 用法

##### 启动dreamview

```shell
buildtool bootstrap start dreamview
```

##### 停止dreamview：

```shell
buildtool bootstrap stop dreamview
```

##### 详细参数

```shell
usage: buildtool bootstrap [-h] {start,stop} ...

positional arguments:
  {start,stop}  options
    start       start module
    stop        stop module

optional arguments:
  -h, --help    show this help message and exit
```

### create

这个 action 用于创建包，目前支持的包模板有 `component` 和 `plugin`

#### 用法

##### 创建一个组件

```shell
buildtool create --template component \
    --namespaces mycomponent \
    modules/mycomponent/xxx_yyy
```

> 上述命令将会在 `modules/mycomponent/xxx_yyy` 目录生成组件包相关的文件

##### 创建一个插件

```shell
buildtool create --template plugin \
    --namespaces planning \
    --dependencies planning:binary:planning \
    --build_dependencies //modules/planning/planning_base/scenario_base:stage //modules/planning/planning_base/common:planning_common \
    --includes modules/planning_base/scenario_base/scenario.h modules/planning/planning_base/common/frame.h \
    --base_class_name apollo::planning::Scenario \
    --class_name MyLaneFollowScenario \
    modules/scenarios/my_lany_follow
```

> 上述命令将会在 `modules/scenarios/my_lany_follow` 目录下生成插件相关的文件
>
> 其中：
>
> `--namespaces planning` 指定了代码的命名空间为 `planning` 生成的插件会在命名空间 `apollo::planning` 下
>
> `--dependencies planning:binary:planning` 会在生成的 `cyberfile.xml` 插入依赖项 `<depend type="binary" repo_name="planning">planning</depend>` > `--build_dependencies` 指定了在 `BUILD` 中要增加的编译依赖，上述例子中的依赖项是 `planning` 包所提供
>
> `--includes` 指定了要添加的头文件，会插件到生成的插件的头文件中，上述例子中则是 `modules/scenarios/my_lane_follow/my_lane_follow.h` 文件
>
> `--base_class_name apollo::planning::Scenario` 指定了插件所继承的基类为 `apollo::planning::Scenario` > `--class_name MyLaneFollowScenario` 指定了生成的插件类名为 `MyLaneFollowScenario` ，若不指定则会根据目录名生成

##### 详细参数

```shell
usage: buildtool create [-h] --template {component,plugin} [--name NAME]
                        [--class_name CLASS_NAME]
                        [--type {src,module,binary,wrapper}] [--author AUTHOR]
                        [--email EMAIL] [--description DESCRIPTION]
                        [--config_message_name CONFIG_MESSAGE_NAME]
                        [--channel_message_type CHANNEL_MESSAGE_TYPE]
                        [--channel_name CHANNEL_NAME]
                        [--namespaces [NAMESPACES [NAMESPACES ...]]]
                        [--includes [INCLUDES [INCLUDES ...]]]
                        [--dependencies [DEPENDENCIES [DEPENDENCIES ...]]]
                        [--build_dependencies [BUILD_DEPENDENCIES [BUILD_DEPENDENCIES ...]]]
                        [--base_class_name BASE_CLASS_NAME]
                        package_path

positional arguments:
  package_path          specify the path of this package needed to create,
                        default to `modules/demo_<template>`

optional arguments:
  -h, --help            show this help message and exit
  --template {component,plugin}
                        specify the template of this package
  --name NAME           specify the name of this package, default to
                        `demo-<template>`
  --class_name CLASS_NAME
                        override default class name
  --type {src,module,binary,wrapper}
                        specify the type of this package
  --author AUTHOR       specify the author of this package
  --email EMAIL         specify the contacted email
  --description DESCRIPTION
                        specify the description of this package
  --config_message_name CONFIG_MESSAGE_NAME
                        specify the proto message name
  --channel_message_type CHANNEL_MESSAGE_TYPE
                        specify the channel message type
  --channel_name CHANNEL_NAME
                        specify the channel name
  --namespaces [NAMESPACES [NAMESPACES ...]]
                        specify the namespace list
  --includes [INCLUDES [INCLUDES ...]]
                        specify the extra include file list
  --dependencies [DEPENDENCIES [DEPENDENCIES ...]]
                        specify the extra dependency list
  --build_dependencies [BUILD_DEPENDENCIES [BUILD_DEPENDENCIES ...]]
                        specify the extra build dependency list
  --base_class_name BASE_CLASS_NAME
                        specify the base class name of plugin
```

### release

这个命令用于打包已编译好的模块，会在当前工作空间下生成一个release.tar.gz，可供后续部署使用

#### 用法

```shell
buildtool release
```

上述操作buildtool将工作空间下所有源码的编译产出进行打包。

```shell
buildtool release -p modules/planning/planning_base
```
上述操作buildtool打包工作空间下planning源码的产出

##### 详细参数

```shell
usage: buildtool release [-h] [-p [* [* ...]]]

optional arguments:
  -h, --help            show this help message and exit
  -p [* [* ...]], --packages [* [* ...]]
                        Specify the package path.
```

### deploy

这个命令用于部署 release 生成的 release.tar.gz

#### 用法

```shell
buildtool deploy -f release.tar.gz
```