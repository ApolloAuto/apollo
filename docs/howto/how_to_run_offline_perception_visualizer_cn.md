# 如何运行脱机感知可视化

DreamView工具可以模拟感知模块并展现模拟效果，但它缺乏可视化点云和ROI区域的能力，这有助于调试和测试感知算法。为此，我们提供了一个基于OpenGL和PCL库的离线可视化工具，以显示点云的障碍感知结果。详细信息请参考 [官网](http://apollo.auto/platform/perception.html) 关于离线可视化工具的介绍。

我们将介绍在docker中构建和运行离线可视化工具的详细步骤，如下所示:
### 1. 准备PCD和Pose数据
在运行可视化工具之前，我们需要准备必要的PCD和Pose数据，这些数据可以从带有记录点云数据的ROS包中提取。为了便于数据提取，我们提供了一个ROS启动文件来转储PCD帧文件和一个python脚本来为每个帧生成Pose文件。

#### 1.1 启动PCD导出器
 更多详细信息请参考 [Velodyne驱动文档](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne)
```
roslaunch velodyne export_pcd_offline.launch
```

#### 1.2 播放 ROS bag
默认的ROS bag目录在`/apollo/data/bag`。假设ROS bag的文件名是`example.bag`.
```
cd /apollo/data/bag
rosbag play --clock example.bag
```
播放包时，PCD文件将被转储到导出目录（defalut：`/ apollo / data / pcd`）。提取的PCD文件根据它们的帧号命名，帧号对应于从点云ROS主题播放记录时消息的顺序（例如，`/ apollo / sensor / velodyne64 / compensator / PointCloud2`）。此外，在导出目录中还有另外两个文件（`stamp.txt`和`pose.txt`）。它们将用于为每个帧生成Pose文件。

#### 1.3 生成Pose文件
Python脚本 `gen_pose_file.py` 用来根据文件`pose.txt`生成Pose文件。
```
cd /apollo/modules/perception/tool
python gen_pose_file.py /apollo/data/pcd
```
与生成Pose文件名称对应的帧编号作为PCD文件名。也就是帧的PCD和Pose文件的名称相同但具有不同的扩展名（即分别为.pcd和.pose）。

### 2. 构建离线感知可视化工具
使用Bazel构建离线可视化工具
```
cd /apollo
bazel build -c opt //modules/perception/tool/offline_visualizer_tool:offline_lidar_visualizer_tool
```
选项`-c opt`用于构建具有优化性能的程序，这对于实时感知模块的离线模拟和可视化非常重要。
如果您想使用GPU运行感知模块，请使用以下命令：
```
bazel build -c opt --cxxopt=-DUSE_GPU //modules/perception/tool/offline_visualizer_tool:offline_lidar_visualizer_tool
```

### 3. 使用离线感知模拟运行可视化工具
在运行可视化工具之前，您可以在配置文件`/apollo/modules/perception/tool/offline_visualizer_tool/conf/offline_lidar_perception_test.flag`中设置数据目录和算法模块设置。每个算法模块的详细参数设置可以根据相应的配置文件`/apollo/modules/perception/tool/offline_visualizer_tool/conf/config_manager.config`进行设置。然后，您可以通过以下命令运行具有离线感知模拟的可视化工具：
```
/apollo/bazel-bin/modules/perception/tool/offline_visualizer_tool/offline_lidar_visualizer_tool
```
现在，可以看到一个弹出窗口，逐帧显示点云的感知结果。障碍物用紫色矩形边界框显示。ROI区域的点云可视化有三种模式：
* 显示灰色的所有点云；
* 仅用绿色显示ROI区域的点云；
* 显示绿色ROI区域的点云和灰色的其他区域的点云。 您可以按键盘上的`S`键依次切换模式。
