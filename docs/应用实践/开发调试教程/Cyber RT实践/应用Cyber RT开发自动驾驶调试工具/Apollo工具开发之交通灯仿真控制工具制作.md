## 实验内容

本实验旨在使用 Apollo Cyber RT 开发交通灯仿真控制工具，通过该工具控制红绿灯状态。

## 体验地址

https://apollo.baidu.com/community/course/39

## 实验目的

通过本实验，学习如何使用 Apollo Cyber RT 开发交通灯仿真控制工具，并利用该工具控制红绿灯状态。

- 建立一个 cyber 工程，
- 熟悉 cyber 工具制作。

## 实践步骤

### 步骤一：启动仿真环境

1. 在终端中，执行 DreamView+ 启动指令，执行成功后，点击菜单栏 dreamview+ 按钮，进入 dreamview+ 界面。

   ```bash
   aem bootstrap start --plus
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_372016d.png)

   当出现如下，即表示 dreamview+ 启动成功了。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_bf50bff.png)

2. 左侧导航栏打开 **Mode Settings** 面板，模式选择 **PnC** Mode，操作选择 **Sim_Control**，进入仿真模式。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3f6490d.png)

3. 选择地图。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c93b265.png)

4. 从左侧 **Modules** 模块种启动 Planning 模块。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_badef52.png)

### 步骤二：编辑代码实现任务功能

#### 前置条件

在`/apollo_workspace`目录创建一个名为`cyber_trafficlight.py`的文件。

```bash
touch cyber_trafficlight.py
```

1. 导入必要的库和模块。

   首先，导入所需的 Python 库和 Apollo Cyber RT 模块。这些库和模块将帮助我们初始化 Cyber 框架并使用 Cyber RT 工具。

   ```bash
   import argparse
    from cyber.python.cyber_py3 import cyber
    from cyber.python.cyber_py3 import cyber_time
    import modules.common_msgs.perception_msgs.traffic_light_detection_pb2 as traffic_light_detection_pb2
    import threading
    import time
   ```

2. 定义相关功能函数。

   在这一步中，我们将定义几个功能函数，用于实现脚本的不同功能。

   1）添加交通灯信息的函数。

   ```bash
   def add_traffic_light(color, traffic_light_pb):
    light = traffic_light_pb.traffic_light.add()
    light.color = color
    light.id = "红绿灯ID"  # 需要自行设置，这是交通灯的唯一标识
    light.tracking_time = 10.0  # 设置跟踪时间，单位秒
   ```

   2）添加消息头信息的函数。

   ```bash
   seq_num = 0

   def add_header(msg):
    global seq_num
    msg.header.sequence_num = seq_num
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    msg.header.module_name = "manual_traffic_light"
    seq_num += 1  # 递增序列号
   ```

   3）发布交通灯消息的函数。

   ```bash
   def publish_traffic_light(writer, traffic_light_msg):
    while not cyber.is_shutdown():
        add_header(traffic_light_msg)  # 添加消息头信息
        writer.write(traffic_light_msg)  # 发布交通灯消息
        time.sleep(0.1)  # 控制消息发布速率
   ```

3. 初始化 Cyber 框架和相关对象。

   在主函数中，我们需要初始化 Cyber 框架和相关对象，以便使用 Cyber RT 工具。

   ```bash
   if __name__ == '__main__':
    # 初始化交通灯检测消息对象
    traffic_light_msg = traffic_light_detection_pb2.TrafficLightDetection()

    # 初始化Cyber框架
    cyber.init()
    node = cyber.Node("traffic_light_command")

    # 创建消息写入器
    writer = node.create_writer(
        "/apollo/perception/traffic_light", traffic_light_detection_pb2.TrafficLightDetection)

    # 创建发布交通灯消息的线程
    thread = threading.Thread(target=publish_traffic_light, args=(writer, traffic_light_msg))
    thread.start()
   ```

4. 进入用户输入循环。

   在一个无限循环中，等待用户的输入。用户可以输入不同的命令，执行相应的操作。

   ```bash
   while not cyber.is_shutdown():
        user_input = input(
            "1: 设置交通灯为红色 2: 设置交通灯为绿色 3: 退出\n")
        traffic_light_msg.ClearField('traffic_light')  # 清空交通灯信息
        if user_input == '1':
            # 用户输入1，设置交通灯为红色
            add_traffic_light(
                traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
        elif user_input == "2":
            # 用户输入2，设置交通灯为绿色
            add_traffic_light(
                traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)
        elif user_input == "3":
            # 用户输入3，退出循环
            break

    # 关闭Cyber框架
    cyber.shutdown()
   ```

   总体：

   ```bash
   from cyber.python.cyber_py3 import cyber
   from cyber.python.cyber_py3 import cyber_time
   import modules.common_msgs.perception_msgs.traffic_light_detection_pb2 as traffic_light_detection_pb2
   import threading
   import time

   def add_forward_light(color, traffic_light_pb):
       light = traffic_light_pb.traffic_light.add()
       light.color = color
       light.id = "红绿灯ID" #需要自行设置
       light.tracking_time = 10.0

   seq_num = 0

   def add_header(msg):
       global seq_num
       msg.header.sequence_num = seq_num
       msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
       msg.header.module_name = "manual_traffic_light"
       seq_num = seq_num + 1

   def pub_func(writer):
       while not cyber.is_shutdown():
           global traffic_light_msg
           add_header(traffic_light_msg)
           writer.write(traffic_light_msg)
           time.sleep(0.1)

   traffic_light_msg = None

   if __name__ == '__main__':
       traffic_light_msg = traffic_light_detection_pb2.TrafficLightDetection()
       cyber.init()
       node = cyber.Node("traffic_light_command")
       writer = node.create_writer(
           "/apollo/perception/traffic_light", traffic_light_detection_pb2.TrafficLightDetection)
       thread = threading.Thread(target=pub_func, args=(writer,))
       thread.start()
       while not cyber.is_shutdown():
           m = input(
               "1: forward red 2: forward green\n")
           traffic_light_msg.ClearField('traffic_light')
           print(m)
           if m == '1':
               add_forward_light(
                   traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
           elif m == "2":
               add_forward_light(
                   traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)

       cyber.shutdown()
   ```

### 步骤三：路径设置工具的使用

1. 打开 Dreamview+ routing editing。

   在地图上选择起点、终点并保存。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_43361d3.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_55e489f.png)

   点击播放按钮，车辆启动：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_1b1a0ee.png)

2. 查看红绿灯 ID。

   回到终端，打开 cyber_monitor

   ```bash
   cyber_monitor
   ```

   找到 planning 的 channel。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a1fa25f.png)

   通过键盘的 PgUp 键进行翻页找到车辆停止原因。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7165b76.png)

   > 注意：TL\_后面的数字就是红绿灯的ID。

3. 修改红绿灯 ID。

   在工具中添加交通灯 ID：

   ```bash
    def add_traffic_light(color, traffic_light_pb):
        light = traffic_light_pb.traffic_light.add()
        light.color = color
        #light.id = "红绿灯ID" #修改前
        light.id = "451089196 #修改后
        light.tracking_time = 10.0
   ```

4. 运行交通灯仿真控制工具。

   ```bash
   python cyber_trafficlight.py
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_02a8989.png)

   > 注意：数字 1 是控制交通灯为红灯；数字 2 是控制交通灯为绿灯。

数字 1：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_92bee2e.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_dc4fad6.png)

数字 2：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f562f17.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_21fdb15.png)
