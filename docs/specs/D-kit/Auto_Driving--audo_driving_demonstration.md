# 封闭园区自动驾驶搭建--自动驾驶演示
## 前提条件
 1. 正确完成了感知硬件集成、感知设备的标定、定位地图的制作、高精地图的制作以及感知适配和规划适配等前序工作。
 2. 确保在道路平整、车少人少等相对安全的情况下实验。
 3. 确保至少两人操作，一人操作工控机，一人操作遥控器，做好随时接管准备

## 启动流程
1. 进入docker环境，用gpu编译项目，编译项目，启动Dreamview 

```
    cd apollo
    bash docker/scripts/dev_start.sh
    bash docker/scripts/dev_into.sh
    bash apollo.sh build_gpu
    bash scripts/bootstrap.sh
```
2. 选择车辆相应配置（ch），选择相应高精地图，在module Controller标签页启动gps、localization、Radar、canbus、control模块
3. 用以下命令启动激光雷达

```
    bash scripts/velodyne_16.sh
```

4. 用以下命令查看topic

```
    rostopic echo /apollo/localization/pose 
    #如果有正常输出就查看下一条topic
    rostopic echo /apollo/tf_static #确保里面child_frame_id是velodyne16
    rostopic echo /apollo/sensor/velodyne16/compensator/PointCloud2 #确保有点云输出
```

5. 在module Controller标签页启动perception模块，并在dreamview上查看障碍物信息，并确定有障碍物输出，如下图所示
![图片](../images/D-kits_Audo_driving_Adaptation/perception_verification.png)
6. 在module Controller标签页启动planning、prediction、routing模块确保这些模块能够正常启动。
7. 在routing editor标签中点击Add Point of Interest按钮添加一个point 然后选择Send Routing Request按钮发送添加的routing点，从dreamview中查看会出现一个蓝色的线 以及一个红色的stop标志。
如下图所示
![图片](../images/D-kits_Audo_driving_Adaptation/planning_add_routing.png)
8. 在dreamview上查看planning轨迹线，只有当轨迹线出现的时候才能进行下一步操作。如上图中蓝色轨迹线所示。
9. 在附近没有人员和车的情况下，遥控器下放权限并在task标签页中点击start auto使车进入自动驾驶状态，在车自动驾驶这个过程中做好随时用遥控器接管，确保安全的准备。