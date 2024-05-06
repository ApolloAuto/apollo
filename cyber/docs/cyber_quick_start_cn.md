# Apollo Cyber RT 快速开始

## 如何使用 Cyber RT 创建新的组件

Apollo 的 Cyber RT 框架是基于组件概念来构建的。每个组件都是 Cyber RT 框架的一个特定的算法模块， 处理一组输入并产生其输出
数椐。

要创建并启动一个算法组件，需要通过以下 4 个步骤：

- 初如化组件的目录结构
- 实现组件类
- 设置配置文件
- 启动组件

下面的例子展示了如何创建、编译和运行一个组件。想更深入地探索 Cyber RT 框架，了解其各种功能，可参考`cyber/examples/`目录
下的更多示例。

_Note: 这些例子必须运行在 Apollo Docker 环境内， 且需要通过 Bazel 来编译。_

### 初始化组件的目录结构

以`cyber/examples/common_component_example/`目录下的样例程序为例：

- C++头文件: common_component_example.h
- C++源文件: common_component_example.cc
- Bazel 构建文件: BUILD
- DAG 文件: common.dag
- Launch 文件: common.launch

### 实现组件类

#### 头文件

如何实现`common_component_example.h`:

- 继承 Component 类
- 定义自己的 `Init` 和 `Proc` 函数。Proc 需要指定输入数椐类型。
- 使用`CYBER_REGISTER_COMPONENT`宏定义把组件类注册成全局可用。

```cpp
#include <memory>

#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::proto::Driver;

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};
CYBER_REGISTER_COMPONENT(CommonComponentSample)
```

#### 源文件

对于源文件 `common_component_example.cc`, `Init` 和 `Proc` 这两个函数需要实现。

```cpp
#include "cyber/examples/common_component_example/common_component_example.h"

bool CommonComponentSample::Init() {
  AINFO << "Commontest component init";
  return true;
}

bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
  AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
        << msg1->msg_id() << "]";
  return true;
}
```

#### 创建 BUILD 文件

```python
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    linkshared = True,
    linkstatic = False,
    deps = [":common_component_example_lib"],
)

cc_library(
    name = "common_component_example_lib",
    srcs = ["common_component_example.cc"],
    hdrs = ["common_component_example.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)
cpplint()
```

### 设置配置文件

#### 配置 DAG 文件

在 DAG 依赖配置文件 （例如 common.dag) 中配置如下项：

- Channel names: 输入 Channel 的名称
- Library path: 该组件生成的共享库路径
- Class name: 此组件类的名称

```bash
# Define all components in DAG streaming.
module_config {
module_library : "/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so"
components {
    class_name : "CommonComponentSample"
    config {
        name : "common"
        readers {
            channel: "/apollo/prediction"
        }
        readers {
            channel: "/apollo/test"
        }
    }
  }
}
```

#### 配置 Launch 启动文件

在 launch 启动文件中 (`common.launch`), 配置下面的项：

- 组件的名字
- 上一步配置的 DAG 文件路径
- 运行组件时的进程名

```xml
<cyber>
    <component>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </component>
</cyber>
```

### 启动这个组件

通过下面的命令来编译组件：

```bash
cd /apollo
bash apollo.sh build
```

然后配置环境：

```bash
source cyber/setup.bash
# 从终端观察输出
export GLOG_alsologtostderr=1
```

有两种方法来启动组件：

- 使用 Launch 文件启动（推荐）

```bash
cyber_launch start cyber/examples/common_component_example/common.launch
```

- 使用 DAG 文件启动

```bash
mainboard -d cyber/examples/common_component_example/common.dag
```

#### 提供通道数据给组件处理

打开另一终端, 运行：

```bash
source cyber/setup.bash
export GLOG_alsologtostderr=1
/apollo/bazel-bin/cyber/examples/common_component_example/channel_test_writer
```

再打开一个终端并运行：

```bash
source cyber/setup.bash
export GLOG_alsologtostderr=1
/apollo/bazel-bin/cyber/examples/common_component_example/channel_prediction_writer
```

这时，如果成功，你会看到第一个终端有如下的输出：

```
I0331 16:49:34.736016 1774773 common_component_example.cc:25] [mainboard]Start common component Proc [1094] [766]
I0331 16:49:35.069005 1774775 common_component_example.cc:25] [mainboard]Start common component Proc [1095] [767]
I0331 16:49:35.402289 1774783 common_component_example.cc:25] [mainboard]Start common component Proc [1096] [768]
```

此即验证第一个样例组件已经跑通。
