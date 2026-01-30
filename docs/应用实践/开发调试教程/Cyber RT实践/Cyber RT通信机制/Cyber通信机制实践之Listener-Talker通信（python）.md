## 实验简介

构建一个简单的通信流程，发送方通过 Cyber 的 Node 创建 Writer，接受方通过 Cyber 的 Node 创建 Reader，发送方实时发送车辆的速度，接受方将接受到的车辆速度进行打印。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9d42b48.png)

## 体验地址

https://apollo.baidu.com/community/course/38

## 实验目的

通过 Cyber 的发布-订阅机制，实现一个节点发布消息，另一个节点订阅并接收该消息，了解 cyber 的基础通信知识。

## 前置条件

本实验需要您对 python 代码有一定的了解。

## 实践流程

### 1. 编写代码

1. 创建代码结构目录。
   
   ```bash
    buildtool create --template component communication
    touch /apollo_workspace/communication/listener.py
    touch /apollo_workspace/communication/talker.py
   ```
   
   检验目录结构是否正确：
   
   ```bash
   communication
    ├── BUILD <必须有>
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
    ├── listener.py <必须有>
    ├── proto <必须有>
    │   ├── BUILD <必须有>
    │   └── communication.proto <必须有>
    └── talker.py <必须有>
   ```

2. 定义此次通信消息的数据结构，编写proto/communication.proto文件，内容如下：
   
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

3. 编写发送方 talker 代码，talker.py 代码如下：
   
   ```bash
   import time
    from cyber.python.cyber_py3 import cyber
    from communication.proto.communication_pb2 import Car
   
    def main():
        # 初始化 Cyber
        cyber.init()
   
        # 创建 talker 节点
        talker_node = cyber.Node("talker")
   
        # 从节点创建一个 Topic，用于发送车速数据
        talker = talker_node.create_writer("/car_speed", Car)
        print("I'll start telling you the current speed of the car.")
   
        # 设置初始速度为0，然后速度每秒增加5km/h
        speed = 0
        while not cyber.is_shutdown():
            msg = Car()
            msg.speed = speed
            # 假设车速持续增加
            speed += 5
            talker.write(msg)
            time.sleep(1)
   
    if __name__ == '__main__':
        main()
   ```

4. 编写接受方 listener 代码，listener.py 代码如下：
   
   ```bash
   import time
   from cyber.python.cyber_py3 import cyber
   from communication.proto.communication_pb2 import Car
   
   # 接收到消息后的回调函数
   def message_callback(msg):
       print("now speed is:", msg.speed)
   
   def main():
       # 初始化 Cyber
       cyber.init()
   
       # 创建监听节点
       listener_node = cyber.Node("listener")
   
       # 创建消息监听器并指定回调函数
       listener = listener_node.create_reader("/car_speed", Car, message_callback)
   
       while not cyber.is_shutdown():
           time.sleep(0.2)
   
   if __name__ == '__main__':
       main()
   ```

5. 将`communication/BUILD`修改为以下内容：
   
   ```bash
    load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_binary", "apollo_package", "apollo_component")
    load("//tools:cpplint.bzl", "cpplint")
   
    package(default_visibility = ["//visibility:public"])
   
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

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_1b9204c.png)

### 3. 运行通信程序

#### 1. 运行talker程序

打开新的终端，执行以下命令。

```bash
//设置将输出结果到控制台
export GLOG_alsologtostderr=1
//执行talker
python communication/talker.py
```

#### 2. 运行listener程序

再次打开一个新的终端，执行以下命令：

```bash
//设置将输出结果到控制台
export GLOG_alsologtostderr=1
//执行listener
python communication/listener.py
```

### 4. 运行结果

talker 会一直给 listener 发送实时速度信息。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dc88dab.png)

listener 收到后会将速度输出到控制台：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6ac4c0f.png)
