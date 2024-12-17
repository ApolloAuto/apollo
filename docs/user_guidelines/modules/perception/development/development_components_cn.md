# 组件开发

组件开发指的是开发一个独立的组件（component），用于实现一个独立的功能。例如在点云感知中增加语义分割模型，可以开发一个独立的组件来实现。

## 创建文件夹

创建文件夹，以组件的名称命名。如下示例，pointcloud_ground_detection、pointcloud_map_based_roi、pointcloud_preprocess 每个文件夹代表一个 component。

```bash
├── perception
    ├── pointcloud_ground_detection  // 组件文件夹
    ├── pointcloud_map_based_roi
    ├── pointcloud_preprocess
```

## 组件代码

首先，创建组件的必要代码，如下所示。

```bash
├── component_name
    ├── component_name.cc  // 组件定义，以及入口
    ├── component_name.h
    └── BUILD
```

然后，根据组件的输入、输出数据，定义组件类接口。

> 注意：接收数据类型如果是自定义的类、结构体，则新建 component 与上下游 component 在一个进程内启动，才能正常收发消息。接收的消息如果是 proto 定义的数据类型，则上下游组件在不同进程启动，也能够正常接受消息。

最后，实现 Init 方法，用于初始化加载参数。实现Proc函数，实现组件功能。

```bash
class ComponentName
    : public cyber::Component<LidarFrameMessage> {
 public:
  ComponentName() = default;
  virtual ~ComponentName() = default;
  /**
   * @brief Init component
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of component
   *
   * @param lidar frame message
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<LidarFrameMessage>& message) override;
 };
```

## 配置文件

在 proto 文件夹中定义 component 发送消息通道的名称，组件需要完成的功能的配置。代码结构如下：

```bash
├── component_name
    ├── conf               // 组件配置文件
    ├── proto              // proto定义
    ├── component_name.cc
    ├── component_name.h
    └── BUILD
```

通常配置定义示例如下。component 的配置文件放到 conf 中。

```bash
message LidarDetectionComponentConfig {
  optional string output_channel_name = 1;  // 输出通道名称
  optional PluginParam plugin_param = 2;    // 功能配置，例如地面点分割、预处理等
}
// PluginParam 定义，这里的PluginParam是具体算法或功能的配置。注意与“插件”区分。
// message PluginParam {
//   optional string name = 1;  // 功能名称
//   optional string config_path = 2;  // 功能配置的路径
//   optional string config_file = 3;  // 功能配置的文件，包含具体参数
// }
```

## 功能定义

在组件中声明该功能的基类，用于调用该功能。基类定义在 interface 中，功能代码定义在 function 文件夹中。

```bash
├── component_name
    ├── conf
    ├── proto
    ├── data               // 功能配置参数
    ├── function           // 组件的实际功能定义
        ├── function.h
        ├── function.cc
    ├── interface          // 功能的接口
        ├── base_function.h
        ├── base_function.cc
    ├── component_name.cc
    ├── component_name.h
    └── BUILD
```

## 组件使用

组件的代码结构、功能定义好，就是使用组件。通过 dag 文件来配置组件的启动项，通过 launch 启动组件。

```bash
├── component_name
    ├── conf
    ├── dag     // dag文件
    ├── launch  // launch启动文件
    ├── proto
    ├── data
    ├── function
    ├── interface
    ├── component_name.cc
    ├── component_name.h
    └── BUILD
```

dag文件定义如下:

```bash
module_config {
  module_library : "modules/perception/component_name/libcomponent_name.so"
  components {
    class_name : "ComponentClassName"
    config {
      name : "ComponentName"
      # 在conf文件中定义输出消息的类型和名称，定义具体功能的配置路径（即plugin_param）
      config_file_path : "/apollo/modules/perception/component_name/conf/component_name.pb.txt"
      flag_file_path: "/apollo/modules/perception/data/flag/perception_common.flag"
      readers {
        channel: "/perception/reader_name"  # 上游组件的输出通道名称
      }
    }
  }
}
```

launch 文件通常需要启动的 dag。如下所示：

```bash
<cyber>
    <desc>cyber modules list config</desc>
    <version>1.0.0</version>
    <module>
        <name>component_name</name>
        <dag_conf>/apollo/modules/perception/component_name/dag/component_name.dag</dag_conf>
        <!-- if not set, use default process -->
        <process_name>component_name</process_name>
        <version>1.0.0</version>
    </module>
</cyber>
```

在调试单个组件功能的时候会启动该组件的 launch，如果要启动这个感知，就需要在 launch 中定义所有需要启动的dag，每个dag的上下游 channel 要一一对应。

启动 launch 的命令：

```bash
cyber_launch launch_file_name
```
