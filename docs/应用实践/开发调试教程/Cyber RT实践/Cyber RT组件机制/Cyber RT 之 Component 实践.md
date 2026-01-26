## 实验内容

Component 是 Cyber RT 提供的用来构建功能模块的基础类，Component 有两种类型，分别为 Component 和 TimerComponent。Component 提供消息融合机制，最多可以支持 4 路消息融合，当 从多个 Channel 读取数据的时候，以第一个 Channel 为主 Channel。当主 Channel 有消息到达，Cyber RT会调用 Component的 Proc() 进行一次数据处理。本实验将编写一个 Component 消息融合实例，输入消息为上一个 TimerComponent 实验产生的数据，输出融合后的数据。

## 体验地址

https://apollo.baidu.com/community/course/42

## 实验目的

- 深入理解 Component 的特性。

- 掌握实现 Component 的方法及流程。

## 实验流程

### 1. 前期准备

由于本实验是以上一个 TimerCoponent 实验为基础的，本实验要使用到TimerCoponent实验的输出数据，对其输出数据进行数据融合，上一个实验源码已经为大家准备好了，请按照如下步骤进行实验前的准备工作：

1. 使用以下命令解压 TimerCoponent 源码压缩文件

   ```bash
   tar -zxvf component.tar.gz
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_58eeec9.png)

2. 使用以下命令编译解压的 TimerComponent 实验源码：

   ```bash
   buildtool build -p component/timer_component_sensor/
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d222781.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8c56f76.png)

3. 修改`timer_component_sensor`配置文件，产生一路新的消息：

1）新增`timer_component_sensor_second.pb.txt`文件。复制`timer_component_sensor.pb.txt`文件，并重命名为`timer_component_sensor_second.pb.txt`，修改该文件内的配置项。

```bash
name: "sensor_second"
sensor_topic:"/sensor/second"
```

将 name 值修改为 sensor_second；将 sensor_topic 值修改为 /sensor/second。

> 注意：这里timer_component_sensor_second.pb.txt文件与timer_component_sensor.pb.txt文件位于同一文件夹内。

2）新增`timer_component_sensor_second.dag`文件。复制`timer_component_sensor.dag`文件，并重命名为`timer_component_sensor_second.dag`，修改该文件内的配置项。

```bash
module_config {
  module_library : "component/timer_component_sensor/libtimer_component_sensor_component.so"
  timer_components {
    class_name : "TimerComponentSensor"
    config {
      name: "timer_component_sensor_second"
      config_file_path:  "component/timer_component_sensor/conf/timer_component_sensor_second.pb.txt"
      flag_file_path:  "component/timer_component_sensor/conf/timer_component_sensor.conf"
      interval: 800
    }
  }
}
```

将 config 中的 name 值修改为`timer_component_sensor_second`；

将 config_file_path 值修改为`timer_component_sensor_second.pb.txt`文件所在路径；

将 intercal 值修改 800 。

> 注意：这里`timer_component_sensor_second.dag`文件与`timer_component_sensor.dag`文件位于同一文件夹内。

3）新增`timer_component_sensor_second.launch`文件，复制`timer_component_sensor.launch`文件，并重命名为`timer_component_sensor_second.launch`，修改该文件内的配置项。

```bash
<cyber>
  <module>
    <name>timer_component_sensor</name>
    <dag_conf>./component/timer_component_sensor/dag/timer_component_sensor_second.dag</dag_conf>
    <process_name>timer_component_sensor</process_name>
  </module>
</cyber>
```

将 dag_conf 修改为`timer_component_sensor_second.dag`文件所在的路径。

> 注意：这里`timer_component_sensor_second.launch`文件与`timer_component_sensor.launch`文件位于同一文件夹内。

4. 使用以下命令，运行`timer_component_sensor_second.launch`文件。

   ```bash
   cyber_launch start component/timer_component_sensor/launch/timer_component_sensor_second.launch
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_2e0d265.png)

5. 使用 cyber_monitor 工具验证，配置是否生效。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_bf22ba3.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d2cf9b1.png)

   若 /sensor/second channel 有数据、数据帧率为 1.25、且数据内容与配置的一致，表示配置成功。

6. 最后，在终端中，使用 **Ctrl+C** 组合键，结束程序运行。

### 2. 创建Component模板实例

1. 在终端中，执行以下指令，在 component 文件夹下生成 component_fusion_message 模板实例（功能包）。

   ```bash
   buildtool create --template component component/component_fusion_message
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d97f121.png)

2. 切换目录到生成的`component_fusion_message`目录下，使用 tree 命令查看生成的 component 功能包的目录结构。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e5320f8.png)

   其中，`BUILD`、`component_fusion_message.cc`和`component_fusion_message.h`三个文件分别为 component_fusion_message 功能包的源码编译规则文件、源文件和头文件；conf 目录下，`component_fusion_message.conf`为全局变量配置文件，`component_fusion_message.pb.txt`为用户在`protobuffer`文件中定义的可配置项的配置文件；`cyberfile.xml`为`component_fusion_message`功能包的描述文件；dag 目录下的`component_fusion_message.dag`文件中描述了`component_fusion_message`功能包的依赖关系；launch 文件夹下的`component_fusion_message.launch`为`component_fusion_message`功能包的 launch 启动文件；proto 文件夹下的 BUILD 为用户在 proto 文件夹下定义的`protobuffer`文件的编译规则文件，`component_fusion_message.proto`文件中用户可以定义自己的消息结构。

### 3. 定义Component消息结构

1. 在`component_fusion_message.proto`中，添加以下消息数据结构。

   ```bash
   syntax = "proto2";

    package apollo;

    // message type of channel, just a placeholder for demo,
    // you should use `--channel_message_type` option to specify the real message type
    message ComponentFusionMessageMsg {
      optional string fusion_content = 1;
      optional uint64 fusion_msg_id = 2;
      optional uint64 timestamp = 3;

    }

    message ComponentFusionMessageConfig {
      optional string name = 1;
      optional string fusion_topic = 2;
    };
   ```

   在`ComponentFusionMessageMsg`中添加 fusion_content、fusion_msg_id 和 timestamp 3 项，分别表示消息内容，消息编号和消息发出时的时间戳。

   在`ComponentFusionMessageConfig`中添加可配置项 name 和 fusion_topic ，用来配置 `ComponentFusionMessageConfig`的名称标识和输出数据的 channel。

2. 在 proto 文件夹下的 BUILD 文件中，添加`protobuffer`文件的编译规则。

   ```bash
   load("//tools:apollo_package.bzl", "apollo_package")
    load("//tools/proto:proto.bzl", "proto_library")
    load("//tools:cpplint.bzl", "cpplint")

    package(default_visibility = ["//visibility:public"])

    proto_library(
        name = "component_fusion_message_proto",
        srcs = ["component_fusion_message.proto"],
    )

    apollo_package()

    cpplint()
   ```

   由于没有新增`protobuffer`文件，这里无需修改 BUILD 文件，如若新增`protobuffer`文件，可参考 BUILD 文件中已有的配置规则，添加对应`protobuffer`的编译规则。

### 4. 配置Component的配置文件

由于本实验没有涉及全局变量，这里无需对component_fusion_message.conf文件进行修改；在`component_fusion_message.pb.txt`中，配置`proto`文件中定义的可配置项。

```bash
name: "fusion_message"
fusion_topic: "/fusion/message"
```

将`component_fusion_message.proto`文件中`ComponentFusionMessageConfig`消息中定义的 name 配置为 fusion_message，将 sensor_topic 配置为 /fusion/message。

### 5. 编写Component的源文件

1. 修改`component_fusion_message.h`文件。

   ```bash
   #pragma once
    #include <memory>

    #include "cyber/cyber.h"
    #include "cyber/component/component.h"
    #include "component/component_fusion_message/proto/component_fusion_message.pb.h"
    #include "component/timer_component_sensor/proto/timer_component_sensor.pb.h"

    using apollo::cyber::Time;
    using apollo::cyber::Writer;

    namespace apollo {

    class ComponentFusionMessage final
      : public cyber::Component<apollo::TimerComponentSensorMsg, apollo::TimerComponentSensorMsg> {
     public:
      bool Init() override;
      bool Proc(const std::shared_ptr<apollo::TimerComponentSensorMsg>& msg0, const std::shared_ptr<apollo::TimerComponentSensorMsg>& msg1) override;

     private:
      apollo::ComponentFusionMessageConfig config_;
      std::shared_ptr<Writer<ComponentFusionMessageMsg>> fusion_writer_ = nullptr;
    };

    CYBER_REGISTER_COMPONENT(ComponentFusionMessage)

    } // namespace apollo
   ```

   由于 component 要融合的消息类型是`TimerComponentSensorMsg`，这里需要包含（include）定义`TimerComponentSensorMsg`的头文件（timer_component_sensor.pb.h）。

   使用 **using** 引入将 **apollo::cyber** 命名空间下的 **Time** 和 **Write** 注入当前作用域中；

   修改类继承 component 的模板参数；将模板参数改为要融合的消息类型，两个参数。

   修改 Proc 函数的输入参数；将 Proc 函数的输入参数修改为 **TimerComponentSensorMsg** 的指针。

   ComponentFusionMessage 类增加私有成员 fusion*writer*，fusion*writer* 是智能指针，指向 Writer 对象，Writer 对象可以写出`ComponentFusionMessageMsg`数据。

2. 修改`component_fusion_message.cc`文件。

   ```bash
   #include "component/component_fusion_message/component_fusion_message_component.h"

    namespace apollo {

    bool ComponentFusionMessage::Init() {

      ACHECK(ComponentBase::GetProtoConfig(&config_))
          << "failed to load component_fusion_message config file "
          << ComponentBase::ConfigFilePath();

      AINFO << "Load config succedded.\n" << config_.DebugString();

      fusion_writer_ = node_->CreateWriter<ComponentFusionMessageMsg>(config_.fusion_topic().c_str());

      AINFO << "Init ComponentFusionMessage succedded.";
      return true;
    }

    bool ComponentFusionMessage::Proc(const std::shared_ptr<apollo::TimerComponentSensorMsg>& msg0, const std::shared_ptr<apollo::TimerComponentSensorMsg>& msg1) {
      static int i = 0;
      auto out_msg = std::make_shared<ComponentFusionMessageMsg>();
      out_msg->set_fusion_msg_id(i++);

      std::string merged_content = msg0->content() + ":msg" + std::to_string(msg0->msg_id()) +
                                  " <---> " +
                                  msg1->content() + ":msg" + std::to_string(msg1->msg_id());

      out_msg->set_fusion_content(merged_content);
      out_msg->set_timestamp(Time::Now().ToNanosecond());
      fusion_writer_->Write(out_msg);

      return true;
    }

    } // namespace apollo
   ```

   在 Init 函数中增加智能指针 fusion*write* 的初始化；

   修改 Proc 函数的输入参数与头文件中的定义一致；

   在 Proc 函数中创建`ComponentFusionMessageMsg`消息，并通过 fusion*writer* 发出消息到对应的 channel。

3. 修改`BUILD`文件。

   ```bash
    load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_binary", "apollo_package", "apollo_component")
    load("//tools:cpplint.bzl", "cpplint")

    package(default_visibility = ["//visibility:public"])

    filegroup(
        name = "component_fusion_message_files",
        srcs = glob([
            "dag/**",
            "launch/**",
            "conf/**",
        ]),
    )

    apollo_component(
        name = "libcomponent_fusion_message_component.so",
        srcs = [
            "component_fusion_message_component.cc",
        ],
        hdrs = [
            "component_fusion_message_component.h",
        ],
        linkstatic = True,
        deps = [
            "//cyber",
            "//component/component_fusion_message/proto:component_fusion_message_proto",
            "//component/timer_component_sensor/proto:timer_component_sensor_proto",

        ],
    )

    apollo_package()

    cpplint()
   ```

   由于源文件中由于包含了`timer_component_sensor.pb.h`，需要在编译的依赖项中添加对应的依赖项。

### 6. 编译TimerComponent功能包

在终端中，执行以下指令，编译`component_fusion_message`功能包。

```bash
buildtool build -p  component/component_fusion_message/
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f489afb.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5187f81.png)

编译成功，没有报错，若编译报错，请按照报错提示，修改代码后，重新编译，直到编译成功。

### 7. 修改Component的dag文件

在`component_fusion_message.dag`文件中，将第一个 readers 相中的 channel 值改为 component 中融合的主 channel， 在主 channel readers下面添加需要融合的其它 readers 配置，并配置正确的 channel。

```bash
module_config {
  module_library : "component/component_fusion_message/libcomponent_fusion_message_component.so"
  components {
    class_name : "ComponentFusionMessage"
    config {
      name : "component_fusion_message"
      config_file_path: "component/component_fusion_message/conf/component_fusion_message.pb.txt"
      flag_file_path: "component/component_fusion_message/conf/component_fusion_message.conf"
      readers {
        channel: "/sensor/first"
      }
      readers {
        channel: "/sensor/second"
      }
    }
  }
}
```

这里将 readers 中的 channel 值修改为 /sensor/first ，并在 readers 项下面添加新的 readers 项，并将其 channel 值配置为 /sensor/second。

### 8. 修改Component的launch文件

```bash
<cyber>
  <module>
    <name>component_fusion_message</name>
    <dag_conf>./component/component_fusion_message/dag/component_fusion_message.dag</dag_conf>
    <process_name>component_fusion_message</process_name>
  </module>
</cyber>
```

这里在 <dag_conf> 标签内的 dag 文件路径前加上“ ./ ” 。由于目前 cyber 与 apollo 绑定的比较紧密，编译完成后，系统会把编译产出及配置文件拷贝到 apollo 相应目录下，且执行文件时，系统会优先执行 apollo 目录下的文件，这样就会导致此处的配置无法起作用，这里加上“ ./ ”，就是告诉系统使用此处的dag文件来运行 component 。

### 9. 运行Component

1. 使用两个终端分别启动两个 TimerComponent，产生两路消息。

   产生 /sensor/first 消息

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3f455b3.png)

   产生 /sensor/second 消息

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4d1d207.png)

2. 新启一个终端，在其中启动`component_fusion_message`，进行两路消息融合处理。

   将 /sensor/first 与 /sensor/second 融合后，产生 /fusion/message 消息。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_81ebe14.png)

### 10. 查看Component运行结果

1. 使用 cyber_monitor 工具查看`component_fusion_message`运行结果。

   可以看到 /fusion/message 的帧率与 主 channel 消息 /sensor/first 的帧率一致。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_1774b35.png)

2. 使用 cyber_channel 工具打印 /fusion/message 消息。

   可以看到主 channel 的消息 msg_id 变化了，而另一个 channel 的消息却没的现象，这也验证了 component 是由主 channel 消息出发的。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ed1ff48.png)

### 11. 一个launch文件启动多个component

复制`component_fusion_message.launch`文件，并重命名为`component_fusion_message_all.launch`，将`timer_component_sensor.launch`文件和`timer_component_sensor_second.launch`文件中的 <module> 标签内容拷贝到`component_fusion_message_all.launch`中的 <cyber> 标签内。这样，就可以只启动此`component_fusion_message_all.launch`文件即可一次启动 3 个 component 了。

```bash
<cyber>
  <module>
    <name>timer_component_sensor</name>
    <dag_conf>./component/timer_component_sensor/dag/timer_component_sensor.dag</dag_conf>
    <process_name>timer_component_sensor</process_name>
  </module>

  <module>
    <name>timer_component_sensor</name>
    <dag_conf>./component/timer_component_sensor/dag/timer_component_sensor_second.dag</dag_conf>
    <process_name>timer_component_sensor</process_name>
  </module>

  <module>
    <name>component_fusion_message</name>
    <dag_conf>./component/component_fusion_message/dag/component_fusion_message.dag</dag_conf>
    <process_name>component_fusion_message</process_name>
  </module>
</cyber>
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_fb11746.png)

好的，本次实验到此结束。
