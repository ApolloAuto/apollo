## Summary

The new version of cyber has improved the framework to support communication between cyber and ros. With the following functional highlights:

**Preset conversion of commonly used messages:**

Cyber supports the conversion of some commonly used sensors' messages to ros, including point cloud, IMU and other message formats. Users just need to align the channels on both ends in the configuration, and start it up to use it.

**Plug-in message conversion framework to improve development efficiency:：**

If users still want to implement message communication other than the above, cyber also provides a plug-in based message conversion framework, users do not need to care about the communication API between cyber and ros, they just need to implement the message conversion function, configure the corresponding channel, and then they can receive the messages sent by ros in Apollo modules, which further improves the development efficiency.

## Key technologies

### Leveling the communications barrier

ros uses the rclcpp communication interface, while cyberRT has its own set of communication interfaces, which are incompatible with each other due to the huge difference in the underlying version of the messaging middleware/application layer interface, and would be quite costly to migrate and adapt.

Therefore, we leveled out the communication middleware version, and developed ros-bridge to adapt the communication interface of ros 2, realizing the path from cyber to ros_bridge to ros, and from ros to ros_bridge to cyber.：

![cyber_ros_bridge_1](./images/cyber_ros_bridge_1.png)]

### Plug-in message conversion framework for lower development costs

In addition to dependency leveling and interface adaptation, due to the different definitions of Apollo's messages and ros messages, users can't escape from the message conversion logic between ros bridge and Apollo. This part of message conversion code is strongly coupled with cyber communication API and ros rclcpp API, which is costly to learn, hard to use, and difficult to maintain. Therefore, based on the cyber plug-in mechanism in Apollo 9.0, we decouple the message conversion logic from the internal operation logic of ros_bridge:

![cyber_ros_bridge_2](./images/cyber_ros_bridge_2.png)

## How to use

### 1. Before using：Installnation of Apollo

To use ros-bridge of cyber, simply deploy the latest version of Apollo, Installnation same as the previous Apollo Installnation process.

#### Source code of Apollo Installnation

clone Apollo latest code of github

```bash
git clone https://github.com/ApolloAuto/apollo.git
```

use script to start and enter a docker container

```bash
bash docker/scripts/dev_start.sh
```

compiling Apollo

```bash
bash apollo.sh build
```

setup ros env

```bash
buildtool rosenv -t # -t means use tsu proxy
```

> If you need to implement non-ros official message communication, you need to use the -p parameter to specify the source code path of the non-standard message, usually specify the src path in the user's existing ros project.
> If you need to use specify ros distribution, use -c parameter, such as buildtool rosenv -c humble -t. Currently support iron and humble in ubuntu 22.04, galactic and foxy in ubuntu 20.04.

source ros setup.sh

```bash
source /opt/ros/iron/setup.sh    # if using ubuntu 22.04 and default ros distribution
source /opt/ros/galactic/setup.sh # if using ubuntu 20.04 and default ros distribution
```

> Before building ros_bridge, please ensure source setup.sh of ros. Otherwise the compilation will fail.

compiling ros bridge

```bash
buildtool build -p cyber
```

#### Packages of apollo Installnation

clone latest Apollo workspace of github

```bash
git clone https://github.com/ApolloAuto/application-core.git
```

install aem and start and enter a docker container

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://apollo-pkg-beta.cdn.bcebos.com/neo/beta/key/deb.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/apolloauto.gpg
sudo chmod a+r /etc/apt/keyrings/apolloauto.gpg
echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/apolloauto.gpg] https://apollo-pkg-beta.cdn.bcebos.com/apollo/core"\
    $(. /etc/os-release && echo "$VERSION_CODENAME") "main" | \
    sudo tee /etc/apt/sources.list.d/apolloauto.list
sudo apt-get update
sudo apt install apollo-neo-env-manager-dev --reinstall

aem start
```

install Apollo packages

```bash
buildtool build
```

install cyber source code

```bash
buildtool install cyber
```

setup ros env

```bash
buildtool rosenv -t # -t means use tsu proxy
```

> If you need to implement non-ros official message communication, you need to use the -p parameter to specify the source code path of the non-standard message, usually specify the src path in the user's existing ros project.
> If you need to use specify ros distribution, use -c parameter, such as buildtool rosenv -c humble -t. Currently support iron and humble in ubuntu 22.04, galactic and foxy in ubuntu 20.04.

source ros setup.sh

```bash
source /opt/ros/iron/setup.sh    # if using ubuntu 22.04 and default ros distribution
source /opt/ros/galactic/setup.sh # if using ubuntu 20.04 and default ros distribution
```

> Before building ros_bridge, please ensure source setup.sh of ros. Otherwise the compilation will fail.

compiling ros bridge

```bash
buildtool build -p cyber
```

### 2. Using the ros_bridge program in cyber by scenario

> Cautions:
>
> 1.  The ros package can be run in an apollo environment or in a non-apollo environment on the same device：
>
>     1. (Recommended) If ros_bridge is not in the same environment as the ros program and needs to be configured with DOMAIN_ID, configure the following environment variables in the shell terminal in the ros project as well as in the terminal where ros_bridge is running to enable ros to communicate across environments:
>
>        export ROS_DOMAIN_ID=0
>
>     2. If ros_bridge and ros program are in Apollo environment, you can directly copy the source code of ros program to ros_ws path in Apollo project directory, then use rosdep, colcon to install the dependency and compile

#### 2.1. build-in message communication

Assuming you already have a ROS project in your device, if your need is simply to communicate between the lidar driver, gnss driver, or localization algorithms in the ros and Apollo modules, the ros_bridge program already has built-in logic for converting these basic messages, so all you need to do is to modify the channels for the two parties in the appropriate configuration files:

##### lidar message

configured file：

```text
/apollo/cyber/ros_bridge/converters/common_plugins/pointcloud_msg_converter/conf/default.pb.txt
```

```text
name: "lidar"
apollo_channel_name_0: "/apollo/lidar/pointcloud2"
ros_topic_name_0: "/ros/lidar/pointcloud2"
```

where apollo_channel_name_0 refers to the channel on which apollo reads lidar messages, and ros_topic_name_0 refers to the topic on which the ros program sends lidar messages. after you have set it up, command:

```bash
ros_bridge
```

Start the ros_bridge program, ros_bridge will read the lidar messages from “/ros/lidar/pointcloud2”, transform them with certain rules, and send them to the “/apollo/lidar/pointcloud2” channel for use by downstream Apollo modules.

##### gnss message

While the lidar message conversion is a one-to-one relationship, the gnss part is more complex and involves many-to-many to accomplish the conversion of gnss messages of ros to apollo

configured file：

```text
/apollo/cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/conf/heading_msg_fusion.pb.txt      # Fusion ros nav_sat_fix, odometry, output Apollo Heading message configured file
/apollo/cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/conf/imu_msg_converter.pb.txt       # Converts ros to imu, outputs Apollo imu message configured file
/apollo/cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/conf/nav_msg_converter.pb.txt       # Convert ros of nav_sat_fix, output Apollo gnss best pose message configured file
/apollo/cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/conf/odometry_msg_converter.pb.txt  # If the gnss driver for ros doesn't output odometry itself, this plugin fuses imu and nav_sat_fix based on Kalman filtering to output ros odometry./apollo/cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/conf/odometry_parser.pb.txt         # Converts ros to odometry, outputs Apollo odometry's message conversion plugin
```

The channel configuration of the configuration file is similar to that of the lidar message translation plugin.

##### localization message

The message type of the positioning message plugin ros odometry is converted to apollo's localization_estimate and transform, where the positioning output must be in the world coordinate system or correspond to the coordinates of an apollo map.

#### 2.2. Self-developed message conversion plugin

In addition to the above messages, if you want to develop the conversion of other messages, you can implement your own message conversion plug-in, the specific logic can refer to the lidar message conversion plugin：

```text
.
├── BUILD
├── conf
│   └── default.pb.txt    # Configured file
├── lidar_pointcloud.cc   # Converter plugin implementation file
├── lidar_pointcloud.h    # Converter plugin header file
└── plugins.xml           # Converter destruction file
```

`lidar_pointcloud.h`

```cpp
#pragma once

#include <algorithm>
#include <memory>
#include <cstring>
#include <string>

// include cyber headers
#include "cyber/cyber.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "cyber/ros_bridge/converter_base/convert_interface.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

// include ros message header
#include "sensor_msgs/point_cloud2_iterator.hpp"

// define input output
using InputMsg = sensor_msgs::msg::PointCloud2;
using OutputMsg = apollo::drivers::PointCloud;

using InputMsgPtr = std::shared_ptr<InputMsg>;
using OutputMsgPtr = std::shared_ptr<OutputMsg>;


namespace apollo {
namespace cyber {

...

// Inherit apollo::cyber::RosApolloMessageConverter base class
class LidarPointcloud :
  public apollo::cyber::RosApolloMessageConverter<
    InputTypes<InputMsgPtr>, OutputTypes<OutputMsgPtr>> {
// apollo::cyber::RosApolloMessageConverter<InputTypes<InputMsgPtr>, OutputTypes<OutputMsgPtr>> refers to 1-to-1 message conversion plugin, currently apollo supports up to 4-to-4 message conversion plugins, for example:
// apollo::cyber::RosApolloMessageConverter<InputTypes<InputMsgPtr1, InputMsgPtr2>, OutputTypes<OutputMsgPtr>> is a message conversion plugin that converts two messages in ROS into one Apollo message.
// apollo::cyber::ApolloRosMessageConverter<InputTypes<InputMsgPtr1, InputMsgPtr2>, OutputTypes<OutputMsgPtr>> is a message conversion plugin that converts two messages in Apollo into one message in ROS.
 public:
  LidarPointcloud() {}
  ~LidarPointcloud() {}

  ...

  // define ConvertMsg function
  virtual bool ConvertMsg(
    InputTypes<InputMsgPtr>&, OutputTypes<OutputMsgPtr>&);

    return value;
  }

  ...

};

// Register the plugin using the macro provided by cyber, the first parameter is the plugin class name, the second parameter is fixed value, 'apollo::cyber::MessageConverter'
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
  apollo::cyber::LidarPointcloud, apollo::cyber::MessageConverter)

}  // namespace cyber
}  // namespace apollo
```

`lidar_pointcloud.cc`

```cpp
#include "cyber/ros_bridge/converters/common_plugins/pointcloud_msg_converter/lidar_pointcloud.h" // NOLINT

namespace apollo {
namespace cyber {

// Implementing Message convert Logic
bool LidarPointcloud::ConvertMsg(
  InputTypes<InputMsgPtr>& in, OutputTypes<OutputMsgPtr>& out) {
  auto in_msg = std::get<0>(in.values);
  auto out_msg = std::get<0>(out.values);
  ...

  for (size_t i = 0; i < cloud_msg.height * cloud_msg.width; ++i) {
    auto pointcloud = out_msg -> add_point();
    pointcloud -> set_x(x);
    pointcloud -> set_y(y);
    pointcloud -> set_z(z);
    pointcloud -> set_intensity(static_cast<uint32_t>(std::round(intensity)));
    pointcloud -> set_timestamp(static_cast<uint64_t>(timestamp * 10e9));
  }

  return true;
}

}  // namespace cyber
}  // namespace apollo
```

`BUILD`

```python
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

apollo_plugin(
    name = "liblidar_pointcloud_plugin.so",
    srcs = ["lidar_pointcloud.cc"],
    hdrs = ["lidar_pointcloud.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_proto",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@ros" # declare ros as dependency
    ],
)

apollo_package()

cpplint()
```

`plugins.xml`

```xml
<library path="cyber/ros_bridge/converters/common_plugins/pointcloud_msg_converter/liblidar_pointcloud_plugin.so"> // Specify the relative path to the plugin's dynamic library
    <class type="apollo::cyber::LidarPointcloud" base_class="apollo::cyber::MessageConverter"></class> // Specify the plugin class name, just use apollo::cyber::MessageConverter for the base class name.
</library>
```

`default.pb.txt`

```text
name: "lidar"
apollo_channel_name_0: "/apollo/lidar/pointcloud2" # The channel that corresponds to the first apollo message.
ros_topic_name_0: "/ros/lidar/pointcloud2"         # The channel that corresponds to the first ros message.
```

After completing development, you will also need to specify for ros_bridge to load this new plugin:

```bash
vim /apollo/cyber/ros_bridge/conf/ros_bridge_conf.pb.txt
```

Assuming the new plugin class name: SamplePlugin

```text
converter {
  type: "LidarPointcloud"
}

converter {
  type: "HeadingMsgFusion"
}

converter {
  type: "ImuMsgConverter"
}

converter {
  type: "NavMsgConverter"
}

converter {
  type: "OdometryMsgConverter"
}

# Just add the following
converter {
  type: "SamplePlugin"
}
```

Once you've made the changes, build it with buildtool build -p cyber and then start ros_bridge as normal.
