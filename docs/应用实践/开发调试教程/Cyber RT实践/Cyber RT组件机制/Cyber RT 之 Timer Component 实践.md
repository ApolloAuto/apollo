## 实验内容

Component 是 Cyber RT 提供的用来构建功能模块的基础类，Component 有两种类型，分别为 Component 和 TimerComponent。相较于 Component，TimerComponent 不提供消息融合，也不由消息触发运行，而是由系统定时调用，可用来执行一些例行任务。本实验将编写一个 TimerComponent 实例，来模拟传感器产生数据。

## 体验地址

https://apollo.baidu.com/community/course/36

## 实验目的

- 深入理解 TimerComponent 的特性。

- 掌握实现 TimerComponent 的方法及流程。

## 实验流程

### 1. 创建TimerComponent模板实例

1. 在终端中，执行以下指令，在 component 文件夹下生成 timer_component_sensor 模板实例（功能包）。

   ```bash
   buildtool create --template timer_component component/timer_component_sensor
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3bc1b05.png)

2. 使用 cd 指令，切换目录到生成的`timer_component_sensor`目录下，使用 tree 命令查看生成的 component 功能包的目录结构。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_b89bc55.png)

   其中，BUILD 文件为 TimerComponent 的源码编译的规则文件；conf 目录下，`timer_component_sensor.conf`为全局变量配置文件，`timer_component_sensor.pb.txt`为用户在 proto 文件中定义的可配置项的配置文件；`cyberfile.xml`为`timer_component_sensor`功能包的描述文件；dag 目录下的`timer_component_sensor.dag`文件中描述了`timer_component_sensor`功能包的依赖关系；launch 文件夹下的`timer_component_sensor.launch`为`timer_component_sensor`功能包的 launch 启动文件；proto 文件夹下的 BUILD 为用户在 proto 文件夹下定义的 protobuffer 文件的编译规则文件，`timer_component_sensor.proto`文件中用户可以定义自己的消息结构；`timer_component_sensor.cc`和`timer_component_sensor.h`两个文件为 TimerComponent 的源文件和头文件。

### 2. 定义TimerComponent消息结构

1. 在`timer_component_sensor.proto`中，添加以下消息数据结构。

   ```bash
   syntax = "proto2";

    package apollo;

    // message type of channel, just a placeholder for demo,
    // you should use `--channel_message_type` option to specify the real message type
    message TimerComponentSensorMsg {
      optional string content = 1;
      optional uint64 msg_id = 2;
      optional uint64 timestamp = 3;

    }

    message TimerComponentSensorConfig {
      optional string name = 1;
      optional string sensor_topic = 2;
    };
   ```

   在 **TimerComponentSensorMsg** 中添加 content、msg_id 和 timestamp 3 项，分别表示消息内容，消息编号和消息发出时的时间戳。

   在 **TimerComponentSensorConfig** 中添加可配置项 name 和 sensor_topic，用来配置`timer_component_sensor`的名称和输出数据的 channel。

2. 在 proto 文件夹下的 BUILD 文件中，添加 protobuffer 文件的编译规则。

   ```bash
       load("//tools:apollo_package.bzl", "apollo_package")
    load("//tools/proto:proto.bzl", "proto_library")
    load("//tools:cpplint.bzl", "cpplint")

    package(default_visibility = ["//visibility:public"])

    proto_library(
        name = "timer_component_sensor_proto",
        srcs = ["timer_component_sensor.proto"],
    )

    apollo_package()

    cpplint()
   ```

   由于没有新增 proto 文件，这里无需修改 BUILD 文件，如若新增 proto 文件，可参考 BUILD 文件中已有的配置规则，添加对应 proto 的编译规则。

### 3. 配置TimerComponent的配置文件

由于本实验没有涉及全局变量，这里无需对`timer_component_sensor.conf`文件进行修改；在`timer_component_sensor.pb.txt`中，配置`timer_component_sensor.proto`文件中定义的可配置项。

```bash
name: "sensor_first"
sensor_topic:"/sensor/first"
```

将`timer_component_sensor.proto`文件中 **TimerComponentSensorConfig** 消息中定义的 name 配置为`sensor_first`，将`sensor_topic`配置为`/sensor/first`。

### 4. 编写TimerComponent的源文件

1.  修改`timer_component_sensor.h`文件：

    ```bash
    #pragma once
     #include <memory>

     #include "cyber/common/macros.h"
     #include "cyber/component/component.h"
     #include "cyber/component/timer_component.h"
     #include "cyber/cyber.h"
     #include "component/timer_component_sensor/proto/timer_component_sensor.pb.h"

     using apollo::cyber::Time;
     using apollo::cyber::Writer;

     namespace apollo {

     class TimerComponentSensor final
       : public apollo::cyber::TimerComponent {
      public:
       bool Init() override;
       bool Proc() override;

      private:
       apollo::TimerComponentSensorConfig config_;
       std::shared_ptr<Writer<TimerComponentSensorMsg>> sensor_writer_ = nullptr;
     };

     CYBER_REGISTER_COMPONENT(TimerComponentSensor)

     } // namespace apollo
    ```

        在`timer_component_sensor.h`文件中使用 **using** 引入将 **apollo::cyber** 命名空间下的 **Time** 和 **Write** 注入当前作用域中，在 **TimerComponentSensor** 类的中增加私有成员 **sensor_write**，**sensor_write** 是智能指针，指向 **Writer** 对象，**Writer** 对象可以写出 **TimerComponentSensorMsg** 数据。

2.  修改`timer_component_sensor.cc`文件。

    ```bash
    #include "component/timer_component_sensor/timer_component_sensor_component.h"

     namespace apollo {

     bool TimerComponentSensor::Init() {

       ACHECK(ComponentBase::GetProtoConfig(&config_))
           << "failed to load timer_component_sensor config file "
           << ComponentBase::ConfigFilePath();

       AINFO << "Load config succedded.\n" << config_.DebugString();
       // TODO: add your own init code here.
       sensor_writer_ = node_->CreateWriter<TimerComponentSensorMsg>(config_.sensor_topic().c_str());

       AINFO << "Init TimerComponentSensor succedded.";
       return true;
     }

     bool TimerComponentSensor::Proc() {
       AINFO << "Proc TimerComponentSensor triggered.";

       // TODO: add your own proc code here.
       static int i = 0;
       auto out_msg = std::make_shared<TimerComponentSensorMsg>();
       out_msg->set_msg_id(i++);
       out_msg->set_content(config_.name());
       out_msg->set_timestamp(Time::Now().ToNanosecond());
       sensor_writer_->Write(out_msg);

       return true;
     }

     } // namespace apollo
    ```

    在 Init 函数中增加智能指针 **sensor_write** 的初始化，

    在 Proc 函数中创建 **TimerComponentSensorMsg** 消息，并通过 **sensor_writer** 发出消息到对应的 channel。

3.  修改 BUILD 文件。

    ```bash
        load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_binary", "apollo_package", "apollo_component")
     load("//tools:cpplint.bzl", "cpplint")

     package(default_visibility = ["//visibility:public"])

     filegroup(
         name = "timer_component_sensor_files",
         srcs = glob([
             "dag/**",
             "launch/**",
             "conf/**",
         ]),
     )

     apollo_component(
         name = "libtimer_component_sensor_component.so",
         srcs = [
             "timer_component_sensor_component.cc",
         ],
         hdrs = [
             "timer_component_sensor_component.h",
         ],
         linkstatic = True,
         deps = [
             "//cyber",
             "//component/timer_component_sensor/proto:timer_component_sensor_proto",

         ],
     )

     apollo_package()

     cpplint()
    ```

    由于没有包含其它 proto 文件或者头文件，这里的 BUILD 文件无需修改。

### 5. 编译TimerComponent功能包

在终端中，执行以下指令，编译 **timer_component_sensor** 功能包。

```bash
buildtool build -p  component/timer_component_sensor
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6d9a3ac.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_acc4b93.png)

编译成功，没有报错，若编译报错，请按照报错提示，修改代码后，重新编译，直到编译成功。

### 6. 修改TimerComponent的dag文件

在`timer_component_sensor.dag`文件中，将 interval 项的值改为 500，即每 500ms 运行一次。interval 值为 TimerComponent 的运行时间间隔。

```bash
module_config {
  module_library : "component/timer_component_sensor/libtimer_component_sensor_component.so"
  timer_components {
    class_name : "TimerComponentSensor"
    config {
      name: "timer_component_sensor"
      config_file_path:  "component/timer_component_sensor/conf/timer_component_sensor.pb.txt"
      flag_file_path:  "component/timer_component_sensor/conf/timer_component_sensor.conf"
      interval: 500
    }
  }
}
```

### 7. 修改TimerComponent的launch文件

在<dag_conf>标签内的 dag 文件路径前加上“ ./ ” 。由于目前 cyber 与 apollo 绑定的比较紧密，编译完成后，系统会把编译产出及配置文件拷贝到 apollo 相应目录下，且执行文件时，系统会优先执行 apollo 目录下的文件，这样就会导致此处的配置无法起作用，这里加上“ ./ ”，就是告诉系统使用此处的 dag 文件来运行 component。

```bash
<cyber>
  <module>
    <name>timer_component_sensor</name>
    <dag_conf>./component/timer_component_sensor/dag/timer_component_sensor.dag</dag_conf>
    <process_name>timer_component_sensor</process_name>
  </module>
</cyber>
```

### 8. 运行TimerComponent

#### 方式一：使用mainboard运行timer_component_sensor.dag文件

在终端中，输入以下指令，使用 mainboard 工具运行`timer_component_sensor.dag`文件，运行`timer_component_sensor`功能包。

```bash
mainboard -d ./component/timer_component_sensor/dag/timer_component_sensor.dag
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_715e7a7.png)

> 注意：执行命令时，切记不要漏掉 " ./ "。
> 原因：由于目前 cyber 与 apollo 绑定的比较紧密，编译完成后，系统会把编译产出及配置文件拷贝到 apollo 相应目录下，且执行文件时，系统会优先执行 apollo 目录下的文件，这样就会导致此处的配置无法起作用，这里加上“ ./ ”，就是告诉系统使用此处的 dag 文件来运行 component。

#### 方式二：使用cyber_launch运行timer_component_sensor.launch文件

在终端中，也可以使用 cyber_launch 工具运行`timer_component_sensor.launch`文件，运行`timer_component_sensor`功能包。

```bash
cyber_launch start component/timer_component_sensor/launch/timer_component_sensor.launch
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cb25079.png)

### 9. 查看TimerComponent运行结果

#### 方式一：使用cyber_monitor工具查看timer_component_sensor运行结果。

新启一个终端窗口，在终端中输入执行 cyber_monitor 命令，启动cyber_monitor工具。

```bash
cyber_monitor
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_08e4142.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6f6ff13.png)

在 cyber_monitor 工具中，可以看到有一条 channel 名为 `/sensor/first`，帧率为 2Hz 的消息，这条消息即为`timer_component_sensor`功能包发出的数据，可以按键盘的 **右方向** 键进入此 channel，查看消息内容。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a60877d.png)

在`/sensor/first` channel 中，可以看到消息的类型、帧率、消息数据大小以及消息的具体内容。

#### 方式二：使用cyber_channel工具查看timer_component_sensor运行结果。

在终端中，也可以使用 cyber_channel 命令查看输出消息。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dab2f2c.png)

上图中，使用了`cyber_channel list`命令，列出存在的 channel；使用`cyber_channel hz /sensor/first`查看运行时间间隔；使用`cyber_channel echo /sensor/first`命令，打印消息内容。至于，其它 cyber_channel 指令，可以使用`cyber_channel -h`查看使用介绍。

好的，本次实验到此结束。
