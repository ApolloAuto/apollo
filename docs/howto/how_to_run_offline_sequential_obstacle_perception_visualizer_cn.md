# 如何运行融合障碍可视化工具

Apollo创建了LiDAR障碍物可视化工具，这是一种离线可视化工具，用于显示基于LiDAR的障碍物感知结果（请参看 [如何离线运行Perception Visulizer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer_cn.md)）。但是，该工具缺乏基于雷达的障碍物感知结果和基于其两个传感器的融合结果的可视化能力。

Apollo开发了第二个可视化工具，即融合障碍可视化工具，以补充LiDAR障碍物可视化工具。融合障碍可视化工具显示了这些模块的障碍感知结果：

-  基于LiDAR的算法模块 
-  基于雷达的算法模块
-  融合算法模块，用于调试和测试完整的障碍物感知算法

所有可视化都基于LiDAR数据可视化。来自LiDAR的源数据（用于对场景中的任何对象进行整形的一组3D点）在描绘整个场景的视觉特征方面优于雷达。获取更多关于融合障碍物可视化工具的信息，可参考[Demo视频](http://apollo.auto/platform/perception.html)。

通常，您可以按照三个步骤在Docker中构建和运行融合障碍物可视化工具：

1. 准备源数据。
2. 构建融合障碍物可视化工具。
3. 运行工具。

下面对这三个步骤进行详细阐述。

## 准备源数据


运行融合障碍物可视化工具之前，需要做如下准备：

##### 基于LiDAR的障碍物感知

- 点云数据(PCD)文件
- 车姿

##### 基于雷达的障碍物感知

- 基于雷达获取的障碍物数据的protobuf格式
- 车姿
- 车速

为了便于数据提取，Apollo提供了一个名为`export_sensor_data`的工具来从ROS包中导出数据。

### 步骤

1. 用下面的命令构建数据导出工具：


```
cd /apollo
bazel build //modules/perception/tool/export_sensor_data:export_sensor_data
```

2. 用下面的命令运行数据导出工具：

```
/apollo/bazel-bin/modules/perception/tool/export_sensor_data/export_sensor_data
```

3. 运行ROS bag.   

​       ROS bag的默认目录是`/apollo/data/bag`。 
​       下面的例子展示了文件名为`example.bag`的ROS bag.

​      使用下面的命令：

```
cd /apollo/data/bag
rosbag play --clock example.bag --rate=0.1
```

为确保在执行ROS消息回调时不丢失任何帧数据，建议您降低播放速率，在上例中设置为`0.1`。

播放包时，所有数据文件都会逐帧转储到导出目录中，使用时间戳作为文件名。

默认的LiDAR数据导出目录是`/apollo/data/lidar`。

雷达的数据导出目录是`/apollo/data/radar`。

这些目录通过flag `lidar_path`和`radar_path` 在文件`/apollo/modules/perception/tool/export_sensor_data/conf/export_sensor_data.flag`中定义。

在`lidar_path`指定的目录中，会生成两种不同后缀的文件：*`.pcd`* 和 *`.pose`*。

在`radar_path`指定的目录中，会生成三种不同后缀的文件：*`.radar`*, *`.pose`*, and *`.velocity`*。

## 构建融合障碍物可视化工具

Apollo使用Bazel构建融合障碍物可视化工具。

1. 用下面的命令构建融合障碍物可视化工具：

```
cd /apollo
bazel build -c opt //modules/perception/tool/offline_visualizer_tool:offline_sequential_obstacle_perception_test
```

`-c opt`选项用来优化被构建程序的性能，这种优化对离线模拟和感知模块的实时可视化至关重要。

2. (可选)如果你想在GPU模式下运行感知模块，请使用下面的命令：

```
bazel build -c opt --cxxopt=-DUSE_GPU //modules/perception/tool/offline_visualizer_tool:offline_sequential_obstacle_perception_test
```

## 运行工具

在运行融合障碍物可视化工具之前，您可以在配置文件中设置源数据目录和算法模块设置： `/apollo/modules/perception/tool/offline_visualizer_tool/conf/offline_sequential_obstacle_perception_test.flag`。

默认源数据的目录`lidar_path`和`radar_path`分别对应的是`/apollo/data/lidar`和`/apollo/data/radar`。

visualization-enabling布尔标志为`true`，默认情况下，要显示的障碍物结果类型为`fused`（基于LiDAR和RADAR传感器的融合障碍物结果）。 您可以将`fused`更改为`lidar`或`radar`，以显示基于单传感器的障碍物感知产生的纯障碍物结果。

用下面的命令运行融合障碍物可视化工具：

```
/apollo/bazel-bin/modules/perception/tool/offline_visualizer_tool/offline_sequential_obstacle_perception_test
```

您可以看到如下的结果：

- 一个弹出窗口，逐帧显示点云的感知结果 
- 原点云以灰色显示
- 已检测到的边界框（带有指示标题的红色箭头）：
  -  车辆 (绿色)
  -  行人 (粉色)
  -  自行车 (蓝色)
  -  无法识别的元素 (紫色) 
