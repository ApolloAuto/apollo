## 实践内容

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e4e2b74.png)

以车辆在行进过程中，驶入需限速的交汇路口为例，向大家介绍 Planning 模块插件机制，以及针对该交汇路口限速场景，教大家新建自定义插件，实现在指定区域的限速功能。新建一个 traffic rule 插件流程如下图所示，主要包括了配置插件相关文件、在`traffic_rule_config.pb.txt`中加入新增插件。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4266c80.png)

Apollo9.0 中通过交通规则插件配置可以配置 planning 启用的交通规则插件，交通规则插件的所有业务逻辑对于所有场景均会生效，本实验将通过添加 traffic rule 插件，实现自动驾驶车辆在行驶到交汇路口限速区域时低于指定速度行驶。

## 实践目的

学习 traffic rule 新增插件流程，掌握开发 traffic rule 插件步骤，使学员具有新建自定义插件能力。在本实验中，你将学会：

- a. 指定位置新建插件`region_speed_limit`文件夹。

- b. 根据要求，配置相应插件文件与`traffic_rule_config.pb.txt`。
  
  - 配置`RegionSpeedLimit`类代码文件以及相应 BUILD 文件，
  
  - 配置参数文件以及 BUILD 文件，
  
  - 配置插件参数文件`default_conf.pb.txt`，
  
  - 配置`cyberfile.xml`，
  
  - 配置`plugins.xml`，
  
  - `traffic_rule_config.pb.txt`中加入新增插件。

- c. 编译`region_speed_limit`插件。

- d. 运行车辆规划模块，并在 dreamview+ 上观察车辆驶入交汇路口的速度变化。

## 启动仿真环境

1. 在终端中输入以下指令，启动 dreamview+。
   
   ```bash
   aem bootstrap start --plus
   ```
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e831cb0.png)

2. 运行成功后，点击左上角 dreamview+ 按钮。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_53acd5f.png)

3. 首次进入 Dreamview+ 界面，需要勾选 **Accept the User Agreement and Privacy Policy**，点击 **Enter this mode** 进入 Dreamview+。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_fa209e0.png)

4. 点击红框 **Skip** 跳过介绍。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_65dbda8.png)

5. 在 **Mode/模式** 中选择 **PnC** 模式。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6528f43.png)

6. 在 **Operations/操作** 中选择 **Sim Control/仿真**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_bc5b24d.png)

7. 在 **Modules/模块** 中打开 **Planning** 模块。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_1cdb2b2.png)

8. 在 **Vehicle Visualization/车辆可视化** 面板中点击 **Routing Editing/路径编辑** 进入路径编辑界面。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_144eb36.png)

9. 点击左上角第一个红框选择车辆起始点，第二个红框选择途经点与终点。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_938519d.png)

10. 点击左下角红框开始仿真。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8a83620.png)

11. 如图所示，红线为道路参考线，浅蓝色的轨迹是 planning 模块实时规划的轨迹。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5760e6c.png)

## 全局配置实践实验流程

### 全局配置参数同步

输入全局配置参数同步指令，系统将自动将全局配置参数复制到 profile 的 default 目录中，然后就可以在 profile 目录上轻松修改配置参数。

```bash
buildtool profile config init --package planning --profile=default && aem profile use default
```

### 新建 traffic rule 插件实践

#### 新建插件文件

输入下述指令，在`modules/planning/traffic_rules/`下新建插件文件夹`region_speed_limit`：

```bash
buildtool create --template plugin \
                 --namespaces planning \
                 --includes "modules/common/status/status.h" \
                     "modules/planning/planning_base/traffic_rules_base/traffic_rule.h" \
                     "modules/planning/traffic_rules/region_speed_limit/proto/default_conf.pb.h" \
                 --dependencies planning:planning \
                 --build_dependencies "//modules/planning/planning_base:apollo_planning_planning_base" \
                 --base_class_name apollo::planning::TrafficRule \
                 --config_message_name RegionSpeedLimitConfig \
                 --config_file_name default_conf \
                 modules/planning/traffic_rules/region_speed_limit
```

输入下述指令修改配置文件名为`default_conf.pb.txt`：

```bash
mv modules/planning/traffic_rules/region_speed_limit/conf/region_speed_limit.pb.txt modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt
```

#### 配置插件文件

1. 打开在线编辑器。
   
   点击红框所示位置，打开在线编辑器，对插件文件进行编辑。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9f3bcc8.png)

2. 编辑器配置。
   
   从左到右依次按红框所示点击，选择`/apollo_workspace/`目录，点击 **OK**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_2561ad7.png)
   
   进入后下图红框所示文件夹即为新建插件模版。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_df89756.png)
   
   region_speed_limit.h
   
   region_speed_limit.h文件位置：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3d035e7.png)
   
   region_speed_limit.h文件内容：
   
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
   
    CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit, apollo::planning::TrafficRule)
   
    }  // namespace planning
    }  // namespace apollo
   ```
   
   region_speed_limit.cc
   
   region_speed_limit.cc文件位置：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a6f122e.png)
   
   region_speed_limit.cc文件内容：
   
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
        const std::vector<PathOverlap>& junction_overlaps
                = reference_line_info->reference_line().map_path().junction_overlaps();
        for (const auto& junction_overlap : junction_overlaps) {
            reference_line->AddSpeedLimit(
                    junction_overlap.start_s - config_.forward_buffer(),
                    junction_overlap.end_s + config_.backward_buffer(),
                    config_.limit_speed());
        }
        return Status::OK();
    }
   
    }  // namespace planning
    }  // namespace apollo
   ```
   
   plugin_region_speed_limit_description.xml
   
   plugin_region_speed_limit_description.xml文件位置：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dde873a.png)
   
   plugin_region_speed_limit_description.xml文件内容
   
   ```bash
   <library path="modules/planning/traffic_rules/region_speed_limit/libregion_speed_limit.so">
        <class type="apollo::planning::RegionSpeedLimit" base_class="apollo::planning::TrafficRule">    </class>
    </library>
   ```
   
   cyberfile.xml
   
   cyberfile.xml文件位置
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_56c1a2e.png)
   
   cyberfile.xml文件内容
   
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
   
   BUILD
   
   BUILD 文件位置
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_b4ab4cd.png)
   
   BUILD 文件内容
   
   ```bash
   load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")
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
            "//modules/planning/planning_base:apollo_planning_planning_base",
            "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_cc_proto",
        ],
    )
   
    apollo_package()
   
    cpplint()
   ```
   
   proto/BUILD
   
   proto/BUILD 文件位置
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ee6dea6.png)
   
   proto/BUILD文件内容
   
   ```bash
   load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
   load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
   load("//tools/proto:proto.bzl", "proto_library")
    load("//tools:cpplint.bzl", "cpplint")
   
    package(default_visibility = ["//visibility:public"])
   
    proto_library(
        name = "region_speed_limit_proto",
        srcs = ["region_speed_limit.proto"],
    )
   
    apollo_package()
   
    cpplint()
   ```
   
   proto/region_speed_limit.proto
   
   proto/region_speed_limit.proto 文件位置
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_423f795.png)
   
   proto/region_speed_limit.proto 文件内容
   
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
   
   conf/default_conf.pb.txt
   
   conf/default_conf.pb.txt文件位置
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_de53185.png)
   
   修改`conf/default_conf.pb.txt`文件内容如下：
   
   ```bash
   forward_buffer: 3.0
   backward_buffer: 2.0
   limit_speed: 10.0
   ```

3. 配置插件流程文件。
   
   配置文件位置
   
   ![新增traffic rule插件.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/%E6%96%B0%E5%A2%9Etraffic%20rule%E6%8F%92%E4%BB%B6_bcf2f04.png)
   
   配置文件内容
   
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
      name: "REGION_SPEED_LIMIT"
      type: "RegionSpeedLimit"
    }
   ```

#### 编译插件

输入如下命令进行编译：

```bash
buildtool build -p modules/planning/traffic_rules/region_speed_limit/
```

#### 重启 planning

点击 **Modules/模块** 中的 Planning 模块，关闭 Planning。而后再次点击 Planning 模块，打开 Planning。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c59210d.png)

打开 **Routing Editing/路径编辑** 面板，设置起点终点：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cdebe8b.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c6a39be.png)

原始配置仿真：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d24e1f6.png)

修改插件配置参数：

在`apollo_workspace`工作目录找到`modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt`配置文件，调整经过路口速度。

1. 使用在线编辑工具修改`modules/planning/traffic_rules/region_speed_limit/conf/default_conf.pb.txt`配置文件，将`limit_speed`参数
   
   ```bash
   limit_speed: 10.0
   ```
   
   将配置参数的值修改为：
   
   ```bash
   limit_speed: 3.0
   ```
   
   修改之后保存代码，而后重新编译插件：
   
   ```bash
   buildtool build -p modules/planning/traffic_rules/region_speed_limit/
   ```

2. 修改好代码参数并重新编译后，在 **Modules/模块** 中重启 Planning 模块（必须步骤）。

3. 重新调整插件参数，观察车辆减速效果与减速起止点。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_fd85ca7.png)
