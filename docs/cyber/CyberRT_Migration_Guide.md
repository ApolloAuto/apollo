# Migration guide from Apollo ROS

This article describes the essential changes for projects to migrate from Apollo ROS (Apollo 3.0 and before) to Apollo Cyber RT (Apollo 3.5 and after). We will be using the very first ROS project talker/listener as example to demonstrate step by step migration instruction.

## Build system

ROS use `CMake` as its build system but Cyber RT use `bazel`. In a ROS project, CmakeLists.txt and package.xml are required for defining build configs like build target, dependency, message files and so on. As for a Cyber RT component, a single bazel BUILD file covers. Some key build config mappings are listed below.

Cmake

``` cmake
project(pb_msgs_example)
add_proto_files(
  DIRECTORY proto
  FILES chatter.proto
)
## Declare a C++ executable
add_executable(pb_talker src/talker.cpp)
target_link_libraries(pb_talker ${catkin_LIBRARIES}pb_msgs_example_proto)
add_executable(pb_listener src/listener.cpp)
target_link_libraries(pb_listener ${catkin_LIBRARIES}  pb_msgs_example_proto)
```

Bazel

```python
cc_binary(
  name = "talker",
  srcs = ["talker.cc"],
  deps = [
    "//cyber",
    "//cyber/examples/proto:examples_cc_proto",
    ],
  )
cc_binary(
  name = "listener",
  srcs = ["listener.cc"],
  deps = [
    "//cyber",
    "//cyber/examples/proto:examples_cc_proto",
    ],
  )
```

We can find the mapping easily from the 2 file snippets. For example, `pb_talker` and `src/talker.cpp` in cmake `add_executable` setting map to `name = "talker"` and `srcs = ["talker.cc"]` in BUILD file `cc_binary`.

### Proto

Apollo ROS has customized to support proto message formate that a separate section `add_proto_files` and projectName_proto(`pb_msgs_example_proto`) in `target_link_libraries` are required to send message in proto formate. For config proto message in Cyber RT, it's as simple as adding the target proto file path concantenated with name of `cc_proto_library` in `deps` setting. The `cc_proto_library` is set up in BUILD file under proto folder.

```python
cc_proto_library(
  name = "examples_cc_proto",
  deps = [
    ":examples_proto",
  ],
)
proto_library(
  name = "examples_proto",
  srcs = [
    "examples.proto",
  ],
)
```

The package definition has also changed in Cyber RT. In Apollo ROS a fixed package `package pb_msgs;` is used for proto files, but in Cyber RT, the proto file path `package apollo.cyber.examples.proto;` is used instead.

## Folder structure

As shown below, Cyber RT remove the src folder and pull all source code in the same folder as BUILD file. BUILD file plays the same role as CMakeLists.txt plus package.xml. Both Cyber RT and Apollo ROS talker/listener example have a proto folder for message proto files but Cyber RT requires a separate BUILD file for proto folder to set up the proto library.

### Apollo ROS

- CMakeLists.txt
- package.xml
- proto
  - chatter.proto
- src
  - listener.cpp
  - talker.cpp

### Cyber RT

- BUILD
- listener.cc
- talker.cc
- proto
  - BUILD
  - examples.proto (with chatter message)

## Update source code

### Listener

Cyber RT

```cpp
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"

void MessageCallback(
    const std::shared_ptr<apollo::cyber::examples::proto::Chatter>& msg) {
  AINFO << "Received message seq-> " << msg->seq();
  AINFO << "msgcontent->" << msg->content();
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("listener");
  // create listener
  auto listener =
      listener_node->CreateReader<apollo::cyber::examples::proto::Chatter>(
          "channel/chatter", MessageCallback);
  apollo::cyber::WaitForShutdown();
  return 0;
}
```

ROS

```cpp
#include "ros/ros.h"
#include "chatter.pb.h"

void MessageCallback(const boost::shared_ptr<pb_msgs::Chatter>& msg) {
  ROS_INFO_STREAM("Time: " << msg->stamp().sec() << "." << msg->stamp().nsec());
  ROS_INFO("I heard pb Chatter message: [%s]", msg->content().c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber pb_sub = n.subscribe("chatter", 1000, MessageCallback);
  ros::spin();
  return 0;
}
```

You can see easily from the two listener code above that Cyber RT provides very similar API to for developers to migrate from ROS.

- `ros::init(argc, argv, "listener");` --> `apollo::cyber::Init(argv[0]);`
- `ros::NodeHandle n;` --> `auto listener_node = apollo::cyber::CreateNode("listener");`
- `ros::Subscriber pb_sub = n.subscribe("chatter", 1000, MessageCallback);` --> `auto listener =
      listener_node->CreateReader("channel/chatter", MessageCallback);`
- `ros::spin();` --> `apollo::cyber::WaitForShutdown();`

Note: for Cyber RT, a listener node has to use `node->CreateReader<messageType>(channelName, callback)` to read data from channel.

### Talker

Cyber RT

```cpp
#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"

using apollo::cyber::examples::proto::Chatter;

int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create talker node
  auto talker_node = apollo::cyber::CreateNode("talker");
  // create talker
  auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
  Rate rate(1.0);
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_lidar_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    msg->set_content("Hello, apollo!");
    talker->Write(msg);
    AINFO << "talker sent a message!";
    rate.Sleep();
  }
  return 0;
}
```

ROS

```cpp
#include "ros/ros.h"
#include "chatter.pb.h"

#include <sstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<pb_msgs::Chatter>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    pb_msgs::Chatter msg;
    ros::Time now = ros::Time::now();
    msg.mutable_stamp()->set_sec(now.sec);
    msg.mutable_stamp()->set_nsec(now.nsec);
    std::stringstream ss;
    ss << "Hello world " << count;
    msg.set_content(ss.str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
```

Most of the mappings are illustrated in listener code above, the rest are listed here.

- `ros::Publisher chatter_pub = n.advertise<pb_msgs::Chatter>("chatter", 1000);` --> `auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");`

- `chatter_pub.publish(msg);` --> ` talker->Write(msg);`

## Tools mapping

ROS | Cyber RT | Note
:------------- | :------------- | :--------------
rosbag    |   cyber_recorder |   data file
scripts/diagnostics.sh | cyber_monitor | channel debug
offline_lidar_visualizer_tool   | cyber_visualizer |point cloud visualizer

## ROS bag data migration

The data file changed from ROS bag to Cyber record in Cyber RT. Cyber RT has a data migration tool `rosbag_to_record` for users to easily migrate data files before Apollo 3.0 (ROS) to Cyber RT like the sample usage below.

```bash
rosbag_to_record demo_3.0.bag demo_3.5.record
```
