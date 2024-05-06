# 限速区域仿真调试

## 云实验

该实践内容已经上线 Apollo 云实验室，开快速启动进行实践体验：[新增 traffic rule 插件](https://apollo.baidu.com/community/course/24)。

## traffic rule 插件概述

planning 插件分为 scenario、task、traffic rule 三类，各类插件的使用场景如下图所示。本小节我们将针对如何新增一个 traffic rule 插件展开讲解。

三类插件的作用：

- Traffic rule 用于处理各种交通规则，并将各类规则转化为停车（生成虚拟障碍物实现）、限速两种输出类型。
- Scenario 用于判断车辆所在场景，而后依据不同的场景调用事先定义的 Task 组合。
- Task 用于执行执行具体任务，如生成借道路径、处理障碍物边界、轨迹优化等。
  三者的调用逻辑如下图所示。

![截屏2023-08-17 21.11.56.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/%E6%88%AA%E5%B1%8F2023-08-17%2021.11.56_32fca1a.png)

traffic rule 插件的生成与调用逻辑如下图所示。traffic rule 插件继承自 traffic rule 基类，而后 由`planning_base`中的`traffic_decider`对各个插件进行生成并调用。planning 每进行一次规划任务，会通过`traffic_decider`调用各个 traffic rule，从而使 traffic rule 插件生效。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_e1d84b3.png)

## 新增插件配置流程

以`region_speed_limit`插件为例，一个完整的 traffic rule 插件文件夹目录结构以及各个文件功能如下所示。

执行下述命令，在指定位置生成初始插件文件夹。

```bash
buildtool create --template plugin \
                 --namespaces planning \
                 --name region-speed-limit \
                 --base_class_name TrafficRule modules/planning/traffic_rules/region_speed_limit \
                 --config_message_name RegionSpeedLimitConfig
buildtool profile config init --package planning --profile=default && aem profile use default
```

生成的插件文件夹下文件结构如下所示。

```bash
└── region_speed_limit
    ├── region_speed_limit.cc  //region_speed_limit插件源码
    ├── region_speed_limit.h  //region_speed_limit插件源码
    ├── conf
    │   └── default_conf.pb.txt  //region_speed_limit插件参数配置文件
    ├── proto
    │   ├── region_speed_limit.proto  //region_speed_limit插件参数配置文件数据结构声明文件
    │   └── BUILD  //proto文件编译规则声明文件
    ├── BUILD  //region_speed_limit插件编译规则声明文件
    ├── plugin_region_speed_limit_description.xml  //region_speed_limit插件类声明文件
    └── cyberfile.xml  //region_speed_limit插件版本信息声明文件
```

在上述插件目录下，新建一个 traffic rule 插件流程如下图所示，主要包括了配置插件相关文件、在`traffic_rule_config.pb.txt`中加入新增插件。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_a014ac9.png)

## traffic rule 插件实践

**实践内容：**

新增交汇路口限速 traffic rule 插件：以车辆在行驶过程中，驶入需限速的交汇路口为例，新增交汇路口限速 traffic rule 插件，实现在指定区域的限速功能。

**实践目的：**

学习 traffic rule 新增插件流程，掌握开发 traffic rule 插件步骤，使开发者具有新建自定义插件能力。

**实践流程：**

新建交汇路口限速插件实践：

a. 指定位置新建插件`region_speed_limit`文件夹。

b. 根据要求，配置相应插件文件与`traffic_rule_config.pb.txt`。

- 配置`RegionSpeedLimit`类代码文件以及相应 BUILD 文件

- 配置参数文件以及 BUILD 文件，
- 配置插件参数文件`default_conf.pb.txt`，
- 配置`cyberfile.xml`，
- 配置`plugins.xml`，
- `traffic_rule_config.pb.txt`中加入新增插件。

c. 编译`region_speed_limit`插件。

d. 运行车辆规划模块，并在 dreamview 上观察车辆驶入交汇路口的速度变化。

### 生成插件文件模版

输入下述指令，在`modules/planning/traffic_rules/`下新建插件文件夹`region_speed_limit`。

```bash
buildtool create --template plugin \
                 --namespaces planning \
                 --name region-speed-limit \
                 --base_class_name TrafficRule modules/planning/traffic_rules/region_speed_limit \
                 --config_message_name RegionSpeedLimitConfig
```

### 写 RegionSpeedLimit 类代码文件以及配置相应 BUILD 文件

region_speed_limit.cc：

```bash
#include <memory>
#include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"

namespace apollo {
namespace planning {

/* 定义成员函数*/

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool RegionSpeedLimit::Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) {
    if (!TrafficRule::Init(name, injector)) {
        return false;
    }
    // Load the config this task.
    return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
}

Status RegionSpeedLimit::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
    ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
    const std::vector<PathOverlap>& pnc_junction_overlaps
            = reference_line_info->reference_line().map_path().pnc_junction_overlaps();
    for (const auto& pnc_junction_overlap : pnc_junction_overlaps) {
        reference_line->AddSpeedLimit(
                pnc_junction_overlap.start_s - config_.forward_buffer(),
                pnc_junction_overlap.end_s + config_.backward_buffer(),
                config_.limit_speed());
    }
    return Status::OK();
}

}  // namespace planning
}  // namespace apollo
```

> 说明：
>
> - `Init()`函数：初始化`RegionSpeedLimit`类，读取配置文件信息到`config_`；
> - `ApplyRule()`函数：traffic rule 类调用接口，在运行中实际调用的函数；
> - `reference_line_info->reference_line().map_path()`：获取道路信息，本插件获取了交汇路口信息`pnc_junction_overlaps()`；
> - 针对交汇路口的限速功能，调用了 `ReferenceLine::AddSpeedLimit(double start_s, double end_s, double speed_limit)`，实现了在 `start_s` 处到 `end_s` 处最高速度为 `speed_limit` 的约束。

region_speed_limit.h：

```bash
#pragma once

#include <memory>
#include "cyber/plugin_manager/plugin_manager.h"

/* 添加了相应的头文件*/
#include "modules/common/status/status.h"
#include "modules/planning/traffic_rules/region_speed_limit/proto/region_speed_limit.pb.h"
#include "modules/planning/planning_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class RegionSpeedLimit : public TrafficRule {
    /* 声明成员函数*/
    public:
        bool Init(const std::string& name, const std::shared_ptr<DependencyInjector>& injector) override;
        virtual ~RegionSpeedLimit() = default;

        common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info);

        void Reset() override {}

    private:
        RegionSpeedLimitConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit, TrafficRule)
}  // namespace planning
}  // namespace apollo
```

> 说明：`Reset()`函数：插件变量重置入口，清空上一次决策对插件内变量的更改。
> `CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit,TrafficRule)`:声明该类为插件。

BUILD 文件描述了源码的构建规则以及其依赖。

Build：

```bash
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools/proto:proto.bzl", "proto_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "region_speed_limit_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libregion_speed_limit.so",
    srcs = [
        "region_speed_limit.cc",
    ],
    hdrs = [
        "region_speed_limit.h",
    ],
    description = ":plugin_region_speed_limit_description.xml",
    deps = [
        "//cyber",
        # 添加该插件所需依赖
        "//modules/map/pnc_map:path",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/traffic_rules_base:traffic_rules",
        "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_proto",

    ],
)

apollo_package()

cpplint()
```

> 说明：
>
> - load()：声明编译中使用的相关依赖，
> - filegroup()：声明文件组，编译后会将其安装到相应位置。
> - apollo_plugin()：声明编译动态库、编译源码文件、插件描述文件、依赖。

### 配置参数文件

region_speed_limit.proto：

```bash
syntax = "proto2";

package apollo.planning;

message RegionSpeedLimitConfig {
  // 声明RegionSpeedLimitConfig中的数据结构
  optional double forward_buffer = 1 [default = 3];
  optional double backward_buffer = 2 [default = 2];
  optional double limit_speed = 3 [default = 5];
}
```

> 说明：
>
> - region_speed_limit.proto 文件声明了该插件配置文件的数据结构。
> - BUILD 文件声明了该proto文件的编译规则。

### 写插件参数文件default_conf.pb.txt

```bash
forward_buffer: 3.0
backward_buffer: 2.0
limit_speed: 15.0
```

### 配置cyberfile.xml

```bash
<package format="2">
  <name>region-speed-limit</name>
  <version>local</version>
  <description>
    This is a demo package
  </description>

  <maintainer email="sample@sample.com">Apollo Developer</maintainer>
  <license>Apache License 2.0</license>
  <url type="website">https://www.apollo.auto/</url>
  <url type="repository">https://github.com/ApolloAuto/apollo</url>
  <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>

  <type>module</type>
  <src_path>//modules/planning/traffic_rules/region_speed_limit</src_path>
  <builder>bazel</builder>

  <depend type="binary" repo_name="cyber">cyber</depend>
  <!-- add new dependency-->
  <depend type="binary" repo_name="planning">planning</depend>

  <depend>bazel-extend-tools</depend>
</package>
```

> 说明：声明文件版本等信息。

### traffic_rule_config.pb.txt中加入新增插件

将新建插件加入 traffic rule 配置文件中，从而使 planning 调用该 traffic rule。

```bash
rule {
  name: "BACKSIDE_VEHICLE"
  type: "BacksideVehicle"
}
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
rule {
  name: "DESTINATION"
  type: "Destination"
}
rule {
  name: "KEEP_CLEAR"
  type: "KeepClear"
}
rule {
  name: "REFERENCE_LINE_END"
  type: "ReferenceLineEnd"
}
rule {
  name: "REROUTING"
  type: "Rerouting"
}
rule {
  name: "STOP_SIGN"
  type: "StopSign"
}
rule {
  name: "TRAFFIC_LIGHT"
  type: "TrafficLight"
}
rule {
  name: "YIELD_SIGN"
  type: "YieldSign"
}
rule {
  name: "SPEED_SETTING"
  type: "SpeedSetting"
}
rule {
  name: "REGION_SPEED_SETTING"
  type: "RegionSpeedLimit"
}
```

### 编译插件

执行以下命令编译插件：

```bash
buildtool build -p modules/planning/traffic_rules/region_speed_limit/
```

### 运行场景并调试参数

终端执行以下命令运行 dreamview。

```bash
aem bootstrap start
```

设置`default_conf.pb.txt中limit_speed`为 3，使路口交汇处限速 3m/s。

如图所示，到达路口前速度为 23km/h，车辆正常行驶，

![image (21).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2821%29_394b62a.png)

到交汇路口后，如右图所示，速度降为 10km/h，实现了交汇路口限速功能。
![image (22).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2822%29_8703823.png)
