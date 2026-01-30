## 实验简介

构建一个简单的通信流程，发送方通过 Cyber 的 Node 创建 Writer，接受方通过 Cyber 的 Node 创建 Reader，发送方实时发送车辆的速度，接受方将接受到的车辆速度进行打印。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9baa590.png)

## 体验地址

https://apollo.baidu.com/community/course/37

## 实验目的

通过 Cyber 的发布-订阅机制，实现一个节点发布消息，另一个节点订阅并接收该消息，了解 cyber 的基础通信知识。

## 前置条件

本实验需要您对 C++ 代码有一定的了解。

## 实践流程

### 1. 编写代码

1. 创建代码结构目录。
   
   ```bash
   buildtool create --template component communication
   touch /apollo_workspace/communication/listener.cc
   touch /apollo_workspace/communication/talker.cc
   ```
   
   检验目录结构是否正确。
   
   输入指令：
   
   ```bash
   tree communication/
   ```
   
   目录结构：
   
   ```bash
   communication
    ├── BUILD
    ├── communication.cc
    ├── communication.h
    ├── conf
    │   ├── communication.conf
    │   └── communication.pb.txt
    ├── cyberfile.xml
    ├── dag
    │   └── communication.dag
    ├── launch
    │   └── communication.launch
    ├── listener.cc <必须有>
    ├── proto <必须有>
    │   ├── BUILD <必须有>
    │   └── communication.proto <必须有>
    └── talker.cc <必须有>
   ```

2. 定义此次通信消息的数据结构，编写`proto/communication.proto`文件，内容如下：
   
   ```bash
   syntax = "proto2";
   
   package apollo.communication.proto;
   
    //定义一个车的消息，车的型号，车主，车的车牌号,已跑公里数,车速
    message Car{
        optional string plate = 1;
        optional string type = 2;
        optional string owner = 3;
        optional uint64 kilometers = 4;
        optional uint64 speed = 5;
    };
   ```

3. 编写发送方 talker 代码，talker.cc 代码如下：
   
   ```bash
   #include "communication/proto/communication.pb.h"
    #include "cyber/cyber.h"
    #include "cyber/time/rate.h"
   
    //car数据定义的引用，可以看出其定义来源于一个proto
    using apollo::communication::proto::Car;
   
    int main(int argc, char *argv[]) {
      // 初始化一个cyber框架
      apollo::cyber::Init(argv[0]);
      // 创建talker节点
      auto talker_node = apollo::cyber::CreateNode("talker");
      // 从节点创建一个Topic,来实现对车速的查看
      auto talker = talker_node->CreateWriter<Car>("car_speed");
      AINFO << "I'll start telling you the current speed of the car.";
   
      //设置初始速度为0，然后速度每秒增加5km/h
      uint64_t speed = 0;
      while (apollo::cyber::OK()) {
          auto msg = std::make_shared<Car>();
          msg->set_speed(speed);
          //假设车速持续增加
          speed += 5;
          talker->Write(msg);
          sleep(1);
      }
      return 0;
    }
   ```

4. 编写接受方 listener 代码，listener.cc 代码如下：
   
   ```bash
   #include "communication/proto/communication.pb.h"
    #include "cyber/cyber.h"
   
    using apollo::communication::proto::Car;
   
    //接收到消息后的响应函数
    void message_callback(
            const std::shared_ptr<Car>& msg) {
        AINFO << "now speed is: " << msg->speed();
    }
   
    int main(int argc, char* argv[]) {
        //初始化cyber框架
        apollo::cyber::Init(argv[0]);
        //创建监听节点
        auto listener_node = apollo::cyber::CreateNode("listener");
        //创建监听响应进行消息读取
        auto listener = listener_node->CreateReader<Car>(
                "car_speed", message_callback);
        apollo::cyber::WaitForShutdown();
        return 0;
    }
   ```

5. 修改 BUILD 文件，将新写的代码加入到编译中，`communication/BUILD`文件修改如下：
   
   ```bash
       load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_binary", "apollo_package", "apollo_component")
    load("//tools:cpplint.bzl", "cpplint")
   
    package(default_visibility = ["//visibility:public"])
   
    apollo_cc_binary(
        name = "talker",
        srcs = ["talker.cc"],
        deps = [
            "//cyber",
            "//communication/proto:communication_proto",
        ],
        linkstatic = True,
    )
    apollo_cc_binary(
        name = "listener",
        srcs = ["listener.cc"],
        deps = [
            "//cyber",
            "//communication/proto:communication_proto",
        ],
        linkstatic = True,
    )
   
    apollo_package()
   
    cpplint()
   ```

### 2. 编译程序

```bash
//回到 /apollo_workspace目录下编译
cd /apollo_workspace
buildtool build -p communication
```

编译成功后显示如下信息：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ff7ffc5.png)

### 3. 运行通信程序

#### 1. 运行 talker 程序

打开新的终端，执行以下命令。

```bash
// 设置将输出结果到控制台
export GLOG_alsologtostderr=1
// 编译产生的可执行文件在 /opt/apollo/neo/bin/
cd /opt/apollo/neo/bin/
// 执行talker
./talker
```

#### 2. 运行 listener 程序

再次打开一个新的终端，执行以下命令。

```bash
// 设置将输出结果到控制台
export GLOG_alsologtostderr=1
// 编译产生的可执行文件在 /opt/apollo/neo/bin/
cd /opt/apollo/neo/bin/
// 执行listener
./listener
```

### 4. 运行结果

talker 会一直给 listener 发送实时速度信息。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_151fd6c.png)

listener 收到后会将速度输出到控制台。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a5150d7.png)
