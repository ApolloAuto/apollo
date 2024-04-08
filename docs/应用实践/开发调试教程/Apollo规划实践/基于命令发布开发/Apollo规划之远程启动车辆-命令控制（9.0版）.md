## 实验内容

通过 5G 远程驾驶平台监控和接管自动驾驶车辆是确保自动驾驶安全行驶的重要手段，本实验将基于 Dreamview+ 进行 Sim_Control 仿真，实现对自动驾驶车辆的远程控制，包括启动、停车和靠边停车等操作。

## 实验目的

学习新增和使用新命令行插件的流程，掌握开发命令行插件的步骤，使学员具有新建自定义插件能力。

- 方法：通过使用Apollo 9.0的buildtool功能，新增一个命令控制的插件，实现通过终端控制车辆启动、停车和靠边停车的功能。
- 结果：通过命令行控制，在Dreamview +的界面上，可以观察到车辆启动、停车和靠边停车的仿真效果。

## 实验步骤

### 1. 新建远程控制车辆插件

1. 执行下面命令，新建插件文件夹`remote_control_command_demo`。

   ```bash
   buildtool create --template component remote_control_command_demo
   ```

   执行成功后，执行 ls 命令可以看到当前目录下生成了`remote_control_command_demo`文件夹，如下图所示：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c742c5c.png)

   查看目录结构：

   ```bash
   tree remote_control_command_demo
   ```

   目录结构：

   ```bash
   remote_control_command_demo
    |-- BUILD
    |-- conf
    |   |-- remote_control_command_demo.conf             -----------加载全局参数的文件
    |   `-- remote_control_command_demo.pb.txt           -----------proto的配置文件
    |-- cyberfile.xml                                    -----------编译的加载文件
    |-- dag
    |   `-- remote_control_command_demo.dag              -----------启动的dag文件
    |-- launch
    |   `-- remote_control_command_demo.launch           -----------启动的launch文件
    |-- proto
    |   |-- BUILD                                        -----------proto的编译文件
    |   `-- remote_control_command_demo.proto            -----------proto的定义文件
    |-- remote_control_command_demo.cc                   -----------功能实现文件
    `-- remote_control_command_demo.h                    -----------功能定义文件
   ```

2. 点击红框所示位置，打开在线编辑器，对插件文件进行编辑。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_73db513.png)

3. 将下列文件中的代码拷贝至`remote_control_command_demo`对应的文件中：

   remote_control_command_demo.cc：

   ```bash
   #include "remote_control_command_demo/remote_control_command_demo.h"

    #include <poll.h>

    #include <cctype>

    #include "cyber/common/file.h"
    #include "cyber/record/record_reader.h"

    using apollo::external_command::CommandStatus;

    RemoteCotrolCommandDemo::RemoteCotrolCommandDemo() : command_id_(0), module_name_("demo") {}

    bool RemoteCotrolCommandDemo::Init() {
        action_command_client_
                = std::make_shared<apollo::common::ClientWrapper<apollo::external_command::ActionCommand, CommandStatus>>(
                        node_, "/apollo/external_command/action");
        free_space_command_client_ = std::make_shared<
                apollo::common::ClientWrapper<apollo::external_command::FreeSpaceCommand, CommandStatus>>(
                node_, "/apollo/external_command/free_space");
        lane_follow_command_client_ = std::make_shared<
                apollo::common::ClientWrapper<apollo::external_command::LaneFollowCommand, CommandStatus>>(
                node_, "/apollo/external_command/lane_follow");

        return true;
    }

    bool RemoteCotrolCommandDemo::Proc() {
        int8_t revent = 0;  // short
        struct pollfd fd = {STDIN_FILENO, POLLIN, revent};
        switch (poll(&fd, 1, 100)) {
        case -1:
            std::cout << "Failed to read keyboard" << std::endl;
            return false;
        case 0:
            return true;
        default:
            char data[50];
            std::cin.getline(data, 50);
            std::string input_command_string = data;
            if (input_command_string == "pull_over") {
                // Pull over.
                // SendActionCommand(
                //     apollo::external_command::ActionCommandType::PULL_OVER);
                auto command = std::make_shared<apollo::external_command::ActionCommand>();
                FillCommandHeader(command);
                command->set_command(apollo::external_command::ActionCommandType::PULL_OVER);
                std::cout << "Sending action command: " << command->DebugString() << std::endl;
                auto response = action_command_client_->SendRequest(command);
                if (nullptr == response) {
                    std::cout << "Command sending failed, please check the service is on!\n" << std::endl;
                } else {
                    std::cout << "******Finish sending command.******\n" << std::endl;
                }
                //
            } else if (input_command_string == "stop") {
                // Stop planning.
                // SendActionCommand(apollo::external_command::ActionCommandType::STOP);
                auto command = std::make_shared<apollo::external_command::ActionCommand>();
                FillCommandHeader(command);
                command->set_command(apollo::external_command::ActionCommandType::STOP);
                std::cout << "Sending action command: " << command->DebugString() << std::endl;
                auto response = action_command_client_->SendRequest(command);
                if (nullptr == response) {
                    std::cout << "Command sending failed, please check the service is on!\n" << std::endl;
                } else {
                    std::cout << "******Finish sending command.******\n" << std::endl;
                }
                //
            } else if (input_command_string == "start") {
                // Start planning.
                // SendActionCommand(apollo::external_command::ActionCommandType::START);
                auto command = std::make_shared<apollo::external_command::ActionCommand>();
                FillCommandHeader(command);
                command->set_command(apollo::external_command::ActionCommandType::START);
                std::cout << "Sending action command: " << command->DebugString() << std::endl;
                auto response = action_command_client_->SendRequest(command);
                if (nullptr == response) {
                    std::cout << "Command sending failed, please check the service is on!\n" << std::endl;
                } else {
                    std::cout << "******Finish sending command.******\n" << std::endl;
                }
                //
            } else if (input_command_string == "free1") {
                apollo::external_command::Pose end_pose;
                end_pose.set_x(437556.02);
                end_pose.set_y(4432540.34);
                end_pose.set_heading(1.8);
                std::vector<apollo::external_command::Point> way_points;
                apollo::external_command::Point point1;
                apollo::external_command::Point point2;
                apollo::external_command::Point point3;
                apollo::external_command::Point point4;
                point1.set_x(437536.29);
                point1.set_y(4432560.69);
                point2.set_x(437536.29);
                point2.set_y(4432510.69);
                point3.set_x(437576.29);
                point3.set_y(4432510.69);
                point4.set_x(437576.29);
                point4.set_y(4432560.69);
                way_points.emplace_back(point1);
                way_points.emplace_back(point2);
                way_points.emplace_back(point3);
                way_points.emplace_back(point4);

                SendFreespaceCommand(way_points, end_pose);
            } else {
                std::cout << "Invalid input!" << input_command_string << std::endl;
            }
        }
        return true;
    }

       void RemoteCotrolCommandDemo::SendFreespaceCommand(
            const std::vector<apollo::external_command::Point>& way_points,
            const apollo::external_command::Pose& end) {
        auto command = std::make_shared<apollo::external_command::FreeSpaceCommand>();
        FillCommandHeader(command);
        // Copy way_points
        auto roi_point = command->mutable_drivable_roi();
        for (const auto& point : way_points) {
            roi_point->add_point()->CopyFrom(point);
        }
        // Copy end point
        command->mutable_parking_spot_pose()->CopyFrom(end);
        std::cout << "Sending lane follow command: " << command->DebugString() << std::endl;
        auto response = free_space_command_client_->SendRequest(command);
        if (nullptr == response) {
            std::cout << "Command sending failed, please check the service is on!\n" << std::endl;
        } else {
            std::cout << "******Finish sending command.******\n" << std::endl;
        }
        }
   ```

   remote_control_command_demo.h：

   ```bash
    #pragma once

    #include <memory>
    #include <string>
    #include <vector>

    #include "modules/common_msgs/external_command_msgs/action_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/command_status.pb.h"
    #include "modules/common_msgs/external_command_msgs/free_space_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/path_follow_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/speed_command.pb.h"
    #include "modules/common_msgs/external_command_msgs/valet_parking_command.pb.h"
    #include "modules/common_msgs/planning_msgs/planning.pb.h"

    #include "cyber/component/timer_component.h"
    #include "cyber/cyber.h"
    #include "modules/common/service_wrapper/client_wrapper.h"
    #include "modules/common/util/message_util.h"

    class RemoteCotrolCommandDemo final : public apollo::cyber::TimerComponent {
    public:
        RemoteCotrolCommandDemo();

        bool Init() override;

        bool Proc() override;

    private:
        template <typename T>
        void FillCommandHeader(const std::shared_ptr<T>& command);

        void SendFreespaceCommand(
                const std::vector<apollo::external_command::Point>& way_points,
                const apollo::external_command::Pose& end);

        std::shared_ptr<apollo::common::ClientWrapper<
                apollo::external_command::ActionCommand,
                apollo::external_command::CommandStatus>>
                action_command_client_;
        std::shared_ptr<apollo::common::ClientWrapper<
                apollo::external_command::FreeSpaceCommand,
                apollo::external_command::CommandStatus>>
                free_space_command_client_;
        std::shared_ptr<apollo::common::ClientWrapper<
                apollo::external_command::LaneFollowCommand,
                apollo::external_command::CommandStatus>>
                lane_follow_command_client_;

        uint64_t command_id_;
        const std::string module_name_;
    };

    template <typename T>
    void RemoteCotrolCommandDemo::FillCommandHeader(const std::shared_ptr<T>& command) {
        apollo::common::util::FillHeader(module_name_, command.get());
        command->set_command_id(++command_id_);
    }

    CYBER_REGISTER_COMPONENT(RemoteCotrolCommandDemo);
   ```

   cyberfile.xml：

   ```bash
    <package format="2">
    <name>external-command-demo</name>
    <version>local</version>
    <description> The package contains temp component for converting routing message to external command. </description>
    <maintainer email="apollo-support@baidu.com">Apollo</maintainer>
    <license>Apache License 2.0</license>
    <url type="website">https://www.apollo.auto/</url>
    <url type="repository">https://github.com/ApolloAuto/apollo</url>
    <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>
    <type>module</type>
    <src_path url="https://github.com/ApolloAuto/apollo">//remote_control_command_demo</src_path>
    <depend repo_name="com_github_gflags_gflags" lib_names="gflags">3rd-gflags</depend>
    <depend type="binary" repo_name="common" lib_names="common">common</depend>
    <depend repo_name="routing" type="binary">routing</depend>
   </package>
   ```

   BUILD：

   ```bash
   load("//tools:cpplint.bzl", "cpplint")
    load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

    package(default_visibility = ["//visibility:public"])

    apollo_component(
        name = "libremote_control_command_demo_component.so",
        hdrs = ["remote_control_command_demo.h"],
        srcs = ["remote_control_command_demo.cc"],
        copts = ["-DMODULE_NAME=\\\"external_command_demo\\\""],
        deps = [
            "//cyber",
            "//modules/common/util:message_util",
            "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
            "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
            "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
            "//modules/common_msgs/planning_msgs:planning_cc_proto",
            "//modules/common/service_wrapper:client_wrapper",
        ],
    )

    filegroup(
        name = "conf",
        srcs = glob([
            "conf/*",
            "dag/*",
            "launch/*"
        ]),
    )

    cpplint()
    apollo_package()
   ```

   dag/remote_control_command_demo.dag：

   ```bash
   module_config {
     module_library : "remote_control_command_demo/libremote_control_command_demo_component.so"
     timer_components {
       class_name : "RemoteCotrolCommandDemo"
       config {
         name: "remote_control_command_demo_component"
         interval: 200 # milliseconds
       }
     }
   }
   ```

   launch/remote_control_command_demo.launch：

   ```bash
   <cyber>
      <module>
        <name>remote_control_command_demo</name>
        <dag_conf>remote_control_command_demo/dag/remote_control_command_demo.dag</dag_conf>
        <process_name>remote_control_command_demo</process_name>
      </module>
    </cyber>
   ```

   conf/remote_control_command_demo.conf：

   ```bash
   --flagfile=/apollo/modules/common/data/global_flagfile.txt
   ```

4. 代码文件拷贝完毕后，点击左上角保存代码文件。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_558dca9.png)

5. 点击上一个网页链接，返回操作终端。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ee67a23.png)

6. 执行下面指令，执行代码编译操作；

   编译远程控制车辆模块：

   ```bash
    buildtool init
    buildtool build -p remote_control_command_demo
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_932118c.png)

7. 如下图所示，正常情况会编译成功；如果编译失败，请检查代码的替换内容是否正确（检查完毕后保存后，重新执行5编译操作指令即可）。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4d9df4b.png)

### 2. 启动远程控制车辆模块

1. 在终端中，执行下面命令启动远程控制车辆模块，执行成功后，如下图所示。

   远程命令模块：

   ```bash
   mainboard -d remote_control_command_demo/dag/remote_control_command_demo.dag
   ```

2. 执行成功后，如下图所示，此进程会被单独的挂起：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cc8e694.png)

### 3. 启动DreamView+

在新的终端中，执行`aem bootstrap start --plus`启动 DreamView+，执行成功后，点击菜单栏 Dreamview+ 按钮，进入 Dreamview+ 界面。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8c9001f.png)

启动 Dreamview+ 命令：

```bash
aem bootstrap start --plus
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5de4f6c.png)

当出现如下，即表示 Dreamview+ 启动成功了。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ebb7089.png)

点击左下角 **个人中心** > **设置** > **全局设置** ，可以选择界面语言类型。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a957a49.png)

### 4. 启动车辆的运行环境

1. 点击左侧菜单栏，选择 PnC 模式，操作选择 Sim_Control（首次进入异常，请点击刷新按钮）。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_911ffee.png)

2. 开启 Planning、Prediction 和 TaskManager 模块，此时 Dreamview+ 会启动这些模块。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a389f11.png)

3. 点击 **路由编辑** 按钮，此时开始为车辆进行构建相应的 routing 信息；

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_061d28f.png)

4. 点击 **初始位置** 按钮，选择车辆运行的初始位姿（目前仅支持选择在地图场景内）；

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a6a5018.png)

5. 点击 **轨迹点** 按钮，选择车辆运行的终点（可以选择多个），构建车辆运行的终点位置（目前仅支持选择在地图场景内），选择完毕后，点击 **保存编辑** 按钮，此时构建的运行场景信息被保存。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e00dd23.png)

6. 运行场景信息构建完毕后，点击左下角的运行按钮，车辆构建一条由初始位置到终点的轨迹线，车辆会正常的沿着轨迹线开始行驶。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_2116e2e.png)

7. 运行过程中如果需要暂停当前场景的运行，可以点击左下角的暂停按钮，此时车辆会暂停在当前位置，再次点击运行按钮车辆会继续沿着轨迹线行驶。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_2841cba.png)

### 5. 发送远程控制指令

1. 点击 Dreamview+ 左下角运行按钮，保证车辆沿着当前的轨迹线行驶，切换至远程控制终端，在终端输入 **STOP** 指令，车辆会由 **LANE_FOLLOW** 场景进入 **EMERGENCY_STOP** 场景（如 Dreamview+ 所示），车辆会在当前位置缓慢停止。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_2f3a594.png)

   切换至 Dreamview+ 界面，可以发现车辆已经执行急刹指令；

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_87e76c9.png)

2. 继续在终端输入 START 指令，车辆会再次进入 **LANE_FOLLOW** 场景，车辆会继续沿着当前的的 routing 线行驶，如下图 Dreamview+ 界面所示；

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c873f7b.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_777d320.png)

3. 在终端输入 **pull_over** 指令，车辆会进入 **PULL_OVER** 场景，车辆会执行靠边停车指令；

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3797af1.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_38e0af0.png)

4. 继续在终端输入 **START** 指令，车辆会由 **PULL_OVER** 场景再次进入 **LANE_FOLLOW** 场景，车辆会继续沿着当前的的 routing 线行驶。

5. 当我们向终端发送错误的指令时，例如： **aaaa** ，系统无法识别指令，会提醒我们是无效的指令，对当前车辆的运行场景无影响。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_0d0e863.png)
