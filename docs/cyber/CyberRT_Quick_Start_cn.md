
# 如何使用Cyber RT来创建一个新的组件

Apollo Cyber 运行时框架(Apollo Cyber RT Framework) 是基于组件概念来构建的. 每个组件都是Cyber框架的一个构建块, 它包括一个特定的算法模块， 此算法模块处理一组输入数椐并产生一组输出数椐。

要创建并启动一个算法组件, 需要通过以下4个步骤:

- 初如化组件的文件结构
- 实现组件类
- 设置配置文件
- 启动组件

下面的例子展示了如何创建，编译，运行一个组件，并观察组件在屏幕上的输出。 如果想更深入的探索Apollo Cyber RT框架, 可以在这个目录`/apollo/cyber/examples/`找到很多例子，这些例子详细展示了如何使用Cyber框架的各种功能。

*Note: 这些例子必须运行在Apollo docker环境， 且需要通过Bazel来编译.*


## 初始化组件文件结构
例如组件的根目录为`/apollo/cyber/examples/common_component_example/`需要创建以下文件:

- Header file: common_component_example.h
- Source file: common_component_example.cc
- Build file: BUILD
- DAG dependency file: common.dag
- Launch file: common.launch

## 实现组件类

### 实现组件头文件
如何实现`common_component_example.h`:

- 继承 Component 类
- 定义自己的 `Init` 和 `Proc` 函数. Proc 需要指定输入数椐类型。
- 使用`CYBER_REGISTER_COMPONENT`宏定义把组件类注册成全局可用。

```cpp
#include <memory>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Driver;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class CommonComponentSample : public Component<Driver, Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
};

CYBER_REGISTER_COMPONENT(CommonComponentSample)
```

### 实现组件源文件

对于源文件 `common_component_example.cc`,  `Init` 和 `Proc` 这两个函数需要实现.

```cpp
#include "cyber/examples/common_component_example/common_component_example.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

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

### 创建BUILD编译文件

创建 bazel BUILD 文件.

```bash
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcommon_component_example.so",
    deps = [":common_component_example_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "common_component_example_lib",
    srcs = [
        "common_component_example.cc",
    ],
    hdrs = [
        "common_component_example.h",
    ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
```
## 设置配置文件

### 配置DAG依赖文件

在DAG依赖配置文件 (例如common.dag)中配置下面的项:

 - Channel names: 输入输出数椐的Channel名字
 - Library path: 此组件最终编译出的库的名字
 - Class name: 此组件的入口类的名字

```bash
# Define all coms in DAG streaming.
component_config {
    component_library : "/apollo/bazel-bin/cyber/examples/common_component_example/libcommon_component_example.so"
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

### 配置launch启动文件

在launch启动文件中(common.launch), 配置下面的项:

  - 组件的名字
  - 上一步创建的dag配置的名字。
  - 组件运行时所在的进程目录。

```bash
<cyber>
    <component>
        <name>common</name>
        <dag_conf>/apollo/cyber/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </component>
</cyber>
```

## 启动这个组件

通过下面的命令来编译组件:

```bash
bash /apollo/apollo.sh build
```

Note: 确定组件正常编译成功

然后配置环境:

```bash
cd /apollo/cyber
source setup.bash
```

有两种方法来启动组件:

- 使用launch文件来启动 (推荐这种方式)

```bash
cyber_launch start /apollo/cyber/examples/common_component_example/common.launch
```

- 使用dag文件来启动

```bash
mainboard -d /apollo/cyber/examples/common_component_example/common.dag
```
