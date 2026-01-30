## 前提条件

1. 完成 [基于激光雷达的封闭园区自动驾驶搭建--感知设备标定](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知设备标定.md).

2. 完成 [开通云服务账号向导](https://apollo.baidu.com/Apollo-Homepage-Document/Vehicle_Nameplate/三、开通Fuel云服务/3.%20开通Fuel研发云服务账号).

## 步骤一：虚拟车道线数据包录制

将想要自动驾驶的区域提前按要求录制数据包，作为后面生成虚拟车道线的输入数据。

> 注意：目前限制数据包大小不要超过 5G。

需要的 channel 信息如下所示：

| 序号 | topic                                      |
| ---- | ------------------------------------------ |
| 1    | /apollo/localization/pose                  |
| 2    | /apollo/sensor/gnss/odometry               |
| 3    | /apollo/sensor/gnss/ins_stat               |
| 4    | /apollo/sensor/40p/compensator/PointCloud2 |

为获取上述 `channel` ，需要启动 `GPS` 、 `Localization` 、 `Velodyne` 三个模块。

- `GPS`、`Localization` 模块的启动请参考定位模块配置文档。

- `Velodyne` 模块的启动请参考感知传感器集成文档。

为了节省数据空间，建议您在 docker 环境使用命令行录制数据包，不要使用 `Recorder` 模块录制，录制命令如下：

```
 cyber_recorder record -c /apollo/localization/pose /apollo/sensor/gnss/odometry /apollo/sensor/gnss/ins_stat /apollo/sensor/40p/compensator/PointCloud2
```

> 注意：在录制数据前，应首先确保 `modules/localization/conf/localization.conf` 文件中的 `enable_lidar_localization` 设置为 `false` ，待地图制作完成后如果需要 msf 定位，再修改为 true。

## 步骤二：使用虚拟车道线云服务生成虚拟车道线

### 1. 上传数据至 BOS

> 注意：必须使用在 Dreamland 中提交用户账号信息的 Bos bucket，确认 `Bos容器名` 、 `Bos服务区域` 、 `Bos访问秘钥` 、 `Bos访问私钥` 正确。

在 BOS bucket 中新建目录 `virtual_lane` ，作为后续云标定服务读取数据的 `Input Data Path` ，把前面录制的数据包以及 `lidar_GNSS` 外参文件拷贝至该目录。目录结构如图所示：

![virtual_lane_tree1](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/154fdb28aa1ae132e64c80cd3aef9942.png)

### 2. 提交虚拟车道线生成任务

1. 点击左侧菜单栏的 **Apollo Fuel** 里的 **任务** 菜单项，进入任务管理页面。

2. 点击 **新建任务**，在弹出的下拉框中选择虚拟车道线服务。如下图所示：

   ![virtual_lane_fuel](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/e7092b0aa3b3c66b99bf6398fa48c61a.png)

   - 填写输入数据路径（在本例中为 `virtual_lane` ），

   - 填写输出数据路径（地图生成路径此例中为 `result` ），

   - 填写区域编号（根据当地实际情况填写，本例中所在地方为北京应填 `50` ），

   - 填写雷达类型（是配置的 `/apollo/sensor/lidar16/compensator/PointCloud2` 这个 channel 的雷达类型，此例中是 `lidar16` ），

   - 填写车道宽度（车道线的宽度，此例中为 `3.3` ），

   - 填写额外 ROI 拓展（车道线的边界到真实道路边缘的距离，此例中为 `0.5` ）。

3. 点击 **提交任务**。

> 注意：bos 需要有写权限，并且传入的区域编号应和录制数据包时 `localiztion.conf` 配置文件中的 `local_utm_zone_id` 配置项的值一致才行。

### 3. 获取虚拟车道线并添加到 Apollo 中

任务完成后，将在注册的邮箱中收到一封邮件：

![virtual_lane_email](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/5fd17fb449b843c97b7ac485bfa8f559.png)

如果邮件内容为上图所示则表示虚拟车道线成功生成，查看 bos 上的 Output Data Path（此例中为`result`），此 path 的目录结构如下图所示：

![virtual_lane_tree2](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/ae0b2fcb700303179005ff3aa51f3352.png)

将上图中的 `2020-01-16-08-08-42` 整个文件夹拷贝到 `/apollo/modules/map/data/` 下，重命名为您的地图名字（例如：gongyuan），重启 DreamView 即可在地图下拉框中看到刚添加的地图。

## NEXT

现在，您已经完成虚拟车道线制作，根据您使用的是基于 Lidar 的感知方案还是基于 Camera 的感知方案，接下来可以开始 [基于激光雷达的封闭园区自动驾驶搭建--感知适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知适配.md) 或 [基于摄像头的封闭园区自动驾驶搭建--感知适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知适配.md)。

## 常见问题

### 在 BOS 中没有生成 local_map 文件夹

确认 `/apollo/modules/localization/conf/localization.conf` 文件中 `--local_utm_zone_id` 选项和提交虚拟车道线云服务的时传入的 `zone_id` 一致。

### 在 BOS 中生成的地图有问题

在虚拟车道线数据包录制过程中，车只能沿着预定道路走一遍且不能闭环。

### 在 BOS 没有生成任何地图数据

确认提交账号时给的 BOS 信息正确。
