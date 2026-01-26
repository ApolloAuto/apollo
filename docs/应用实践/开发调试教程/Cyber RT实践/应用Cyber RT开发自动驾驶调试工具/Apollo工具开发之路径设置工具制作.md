## 实验内容

本实验旨在使用 Apollo Cyber RT 开发路径设置工具，通过该工具创建并发送路由请求消息。

## 体验地址

https://apollo.baidu.com/community/course/40

## 实验目的

通过本实验，学习如何使用 Apollo Cyber RT 开发路径设置工具，并利用该工具创建并发送路由请求，以使小车运动起来。

- 建立一个 cyber 工程，

- 熟悉cyber工具制作。

## 实践步骤

### 步骤一：启动仿真环境

1. 在终端中，执行 DreamView+ 启动指令，执行成功后，点击菜单栏 dreamview+ 按钮，进入 dreamview+ 界面。

   ```bash
   aem bootstrap start --plus
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9b5696d.png)

   当出现如下，即表示 dreamview+ 启动成功了。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_204edc0.png)

2. 左侧导航栏打开 **Mode Settings** 面板，模式选择 **PnC** Mode，操作选择 **Sim_Control**，进入仿真模式。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c8bc6e9.png)

3. 选择地图。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_984ecea.png)

4. 从左侧 **Modules** 模块种启动 Planning 模块。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c4c0ba6.png)

### 步骤二：编辑代码实现任务功能

#### 前置条件：

在`/apollo_workspace`目录创建一个名为`cyber_routing.py`的文件：

1. 导入必要的库和模块。

   首先，导入所需的 Python 库和 Apollo Cyber RT 模块。这些库和模块将帮助我们初始化 Cyber 框架并创建路由请求消息。

   ```bash
   import argparseimport time
   from cyber.python.cyber_py3 import cyber
   from cyber.python.cyber_py3 import cyber_time
   from modules.common_msgs.external_command_msgs import lane_follow_command_pb2
   from modules.common_msgs.external_command_msgs import command_status_pb2
   ```

2. 初始化 Cyber 框架。

   在主函数中，我们需要初始化Cyber框架，以便与Cyber节点进行通信。这是实现与其他模块交互的第一步。

   ```bash
   def main():
    # 初始化Cyber
    cyber.init()
    node = cyber.Node("mock_routing_requester")
    sequence_num = 1
   ```

3. 创建路由请求消息。

   在此步骤中，我们将创建一个路由请求消息（LaneFollowCommand），并设置一些必要的属性，如时间戳、模块名称和序列号。

   ```bash
    routing_request = lane_follow_command_pb2.LaneFollowCommand()
    routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
    routing_request.header.module_name = 'cyber_routing'
    routing_request.header.sequence_num = sequence_num
    routing_request.is_start_pose_set = True
   ```

4. 输入起点和终点坐标。

   设置从命令行获取用户输入的起点和终点坐标信息，包括起始航向、起始 X 坐标、起始 Y 坐标、终点航向、终点 X 坐标和终点 Y 坐标。

   ```bash
    start_heading = float(input("start_heading : "))
    start_x = float(input("start_x : "))
    start_y = float(input("start_y : "))
    end_heading = float(input("end_heading: "))
    end_x = float(input("end_x : "))
    end_y = float(input("end_y: "))
   ```

5. 添加路径信息。

   在这一步中，输入的起点和终点坐标信息添加到路由请求消息中，包括起点和终点的坐标信息。

   ```bash
    waypoint = routing_request.way_point.add()
    waypoint.heading = start_heading
    waypoint.x = start_x
    waypoint.y = start_y

    waypoint1 = routing_request.end_pose
    waypoint1.heading = end_heading
    waypoint1.x = end_x
    waypoint1.y = end_y
   ```

6. 创建 Cyber Writer 并发送路由请求。

   最后，创建一个 Cyber Writer，并使用它来将路由请求消息发送到"/apollo/external_command/lane_follow"。

   ```bash
   writer = node.create_client('/apollo/external_command/lane_follow', lane_follow_command_pb2.LaneFollowCommand, command_status_pb2.CommandStatus)
    time.sleep(2.0)
    print("routing", routing_request)
    writer.send_request(routing_request)
   ```

   cyber 工具代码：

   ```bash
   from cyber.python.cyber_py3 import cyber
    from cyber.python.cyber_py3 import cyber_time
    from modules.common_msgs.external_command_msgs import lane_follow_command_pb2
    from modules.common_msgs.external_command_msgs import command_status_pb2

    def main():
        """
        主函数
        """
        # 初始化Cyber
        cyber.init()
        node = cyber.Node("mock_routing_requester")
        sequence_num = 1

        routing_request = lane_follow_command_pb2.LaneFollowCommand()

        routing_request.header.timestamp_sec = cyber_time.Time.now().to_sec()
        routing_request.header.module_name = '111'
        routing_request.header.sequence_num = sequence_num

        routing_request.is_start_pose_set = True

        sequence_num = sequence_num + 1

        # 从命令行选择起点坐标
        start_heading = float(input("start_heading : "))
        start_x = float(input("start_x : "))
        start_y = float(input("start_y : "))

        # 从命令行选择终点坐标
        end_heading = float(input("end_heading: "))
        end_x = float(input("end_x : "))
        end_y = float(input("end_y: "))

        # 添加路径起点
        waypoint = routing_request.way_point.add()
        waypoint.heading = start_heading
        waypoint.x = start_x
        waypoint.y = start_y

        # 设置终点坐标
        waypoint1 = routing_request.end_pose
        waypoint1.heading = end_heading
        waypoint1.x = end_x
        waypoint1.y = end_y

        # 创建Cyber writer
        writer = node.create_client('/apollo/external_command/lane_follow', lane_follow_command_pb2.LaneFollowCommand, command_status_pb2.CommandStatus)
        time.sleep(2.0)
        print("routing", routing_request)
        writer.send_request(routing_request)

    if __name__ == '__main__':
        main()
   ```

### 步骤三：路径设置工具的使用

1. 运行路径设置工具。

   ```bash
   python cyber_routing.py
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_efc8ef9.png)

2. 打开 Dreamview+ routing editing。

   在地图上找任意两点作为起点、终点：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cccb399.png)

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5d65e9e.png)

3. 在工具中输入起点坐标和终点坐标以及朝向。

   ```bash
   start_heading : 0.03
    start_x : 752517.91
    start_y : 2563962.54
    end_heading: 0.00
    end_x : 752754.52
    end_y: 2563960.03
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_d1a0429.png)

   【车辆运行】

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3919a2f.png)

【车辆运行】
