该用户手册旨在帮助用户完成激光雷达的标定（Lidar-GNSS）。

## 前提条件

1. 完成 [循迹搭建--车辆循迹演示](../应用实践/车辆集成教程/循迹搭建/车辆循迹演示.md)。

2. 完成 [基于激光雷达的封闭园区自动驾驶搭建--感知设备集成](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知设备集成.md)。

3. 完成 [开通云服务账号向导](https://apollo.baidu.com/Apollo-Homepage-Document/Vehicle_Nameplate/三、开通Fuel云服务/3.%20开通Fuel研发云服务账号)。

## 步骤一：修改配置文件

| 修改文件名称                                                            | 修改内容                                 |
| ----------------------------------------------------------------------- | ---------------------------------------- |
| `modules/calibration/data/车型目录/localization_conf/localization.conf` | `enable_lidar_localization`设置为`false` |

> 注意：上表中 `车型目录` 需要您根据自身车型确认。

![sensor_calibration_modify_localization_file](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/627bfcd21272f783859586cab253755e.png)

> 注意：所有传感器标定完成后，如果用户要使用 msf 定位，则需要再改为 `true`。

## 步骤二：Lidar-GNSS 标定初始外参测量

各传感器坐标系的定义及初始化外参文件的配置。

### IMU、Lidar 的坐标系定义

![1.jpeg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_dd81f51.jpeg)

### Lidar 坐标系原点在传感器的位置

Lidar 坐标系原点的位置如下图所示。

![lidar_calibration_coordinate_picture](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/34765121c8579d0383f025651559843c.png)

### IMU 坐标系原点在传感器的位置

IMU 坐标系原点如下图所示：

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_d2ff22c.png)

### 手动测量 Lidar-GNSS 的初始化外参

这里默认用户按照感知设备集成文档的要求正确安装了传感器，即传感器坐标系的定义与上文中的定义相同，且安装误差满足文档中的要求。

- rotation：在传感器的安装满足上述安装要求的情况下，用户无需测量该值，可直接使用如下的默认值即可。

  ```txt
    rotation:
      w: 0.7071
      x: 0.0
      y: 0.0
      z: 0.7071
  ```

- translation：用户需要手动测量以 IMU 坐标系为基坐标系，以激光雷达坐标系为目标坐标系的位移变换，一个 IMU-Lidar 位移变换的示例如下：

  ```txt
    translation:
      x: 0.0
      y: 0.38
      z: 1.33
  ```

## 步骤三：标定场地选择

标定场地会直接影响到标定效果。对标定场地有如下要求：

- 标定场地中心 8 米范围内需要有轮廓清晰的静态参照物，如电线杆、建筑物、车辆，避免过多动态障碍物。如果静态障碍物距离较远，会严重影响标定效果。

- 确保路面平坦。

- 能确保 GNSS 信号良好，不要有过多的干扰。

## 步骤四：使用 Fuel-Client 采集数据

### 1. 选择正确的模式、车型

1. 选择 `Lidar-IMU Sensor Calibration` 模式。

2. 根据实际情况选择正确的车型。

   ![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_0caa8d8.png)

### 2. 启动 Fuel Client，并启动相应模块

1. 在 DreamView 的 `Tasks` 标签下，打开`Sim Control` 。

2. 打开 `Fuel Client` 。

> 注意：`Fuel Client` 打开后务必关闭 `Sim Control` 。

    ![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_bd85dab.png)

3. 在 DreamView 的 `Module Controllers` 标签下，启动 `GPS` 、 `Lidar` 、 `Localization` 模块，等待左侧状显示模块中的 `GPS` 、 `Lidar` 、 `RTK` 均为绿色时，代表模块启动成功。

> 注意：Localization 启动后，需要等待 1~2 分钟才能正常输出数据。

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_15d0247.png)

### 3. 启动 Recorder 模块并开始采集

- 当左侧左侧状显示模块中的 `GPS` 、 `Lidar` 、 `RTK` 均为绿色时，打开 `Recorder` 模块，并开始采集数据，需要控制车辆以 8 字形轨迹缓慢行驶，并使转弯半径尽量小 ，包含 2~3 圈完整的 8 字轨迹数据。

![lidar_calibration_turn_around](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/9a1bdc4dfdb96585db3a9c4cd9411965.png)

- 数据采集完成后，关闭 `Recorder` 模块停止数据录制。

## 步骤五：使用 Fuel Client 进行数据预处理

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_af63894.png)

1. 点击右上角的 `Configuration` 进入预处理界面。

2. 填写测量的初始化外参。

3. 点击 `Preprocess` 进行预处理。

   等待预处理完成，提示 `Data extraction is completed successfully!` 代表预处理完成。

> 注意：对于使用 3 激光雷达的用户，需要填入 3 个雷达的初始化外参，其中红色的 `lidar16_back` 参数为 `main_sensor` （主激光雷达）的外参。

![1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/1_44c34ae.png)

4. 保存生成的预处理文件。切换到 `apollo/output/sensor_calibration/lidar_to_gnss/extracted_data/` 目录，保存该目录下的 `lidar_to_gnss-xxx` 文件（ `xxx` 为数据包录制的时间），这里和下文以 `lidar_to_gnss-2021-01-12-14-10` 为例。

## 步骤六：使用标定云服务生成外参文件

### 1. 上传预处理后的数据至 BOS

> 注意：必须使用开通过权限的 bucket，确认 `Bucket名称` 、 `所属地域` 和提交商务注册时的 Bucket 名称和所属区域保持一致。

将 `lidar_to_gnss-2021-01-12-14-10` 目录上传到 BOS 的根目录下，作为后续云标定服务读取数据的 `输入数据路径`。

### 2. 提交云标定任务

1. 打开 Apollo 云服务页面。

2. 点击 **Apollo Fuel** > **任务** 。

3. 点击 **新建任务** 。

![fuel_new_task](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/78f28add5ffcca20b38b547db16292b1.png)

4. 在下拉框中选择 **感知标定** 。

   - 根据实际情况输入 **输入数据路径** ，这里以 `lidar_to_gnss-2021-01-12-14-10` 为例。
   - 指定 **输出数据路径**。

5. 点击 **提交任务**（Submit Job）按钮提交。

   ![sensor_calibration_fuel](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/fd6b2d735012fe35b66f50ea4447576d.jpg)

### 3. 获取标定结果验证及标定外参文件

云标定任务完成后，将在注册的邮箱中收到一封标定结果邮件。如果标定任务成功，将包含标定外参文件。

**Lidar-GNSS标定结果验证** ：

- BOS 中用户指定的 **输出文件路径** 下包含了后缀名为 `.pcd` 的点云文件，使用点云查看工具检查 pcd 文件，如果点云文件中周围障碍物清晰、锐利，边缘整齐表示标定结果准确，否则请重新标定。

![lidar_calibration_point_cloud](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/dcfdc92e79a693d42045638154d8f26e.png)

- 在 Ubuntu 系统下，可以使用 `pcl-tools` 工具查看点云文件：

```bash
  sudo apt-get update
  sudo apt-get install pcl-tools
  pcl_viewer xxx.pcd
```

**Lidar-GNSS标定外参文件**：

确认得到的外参文件合理后，将获取的外参文件的 `rotation` 、 `translation` 的值替换掉 `modules/calibration/data/车型目录/lidar_params/velodyne16_novatel_extrinsics.yaml` 中对应的 `rotation` 、 `translation` 值。注意不要修改 `frame_id` 、不要直接替换文件。

## NEXT

现在，您已经完成激光雷达感知设备标定，接下来可以开始 [封闭园区自动驾驶搭建--虚拟车道线制作](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/虚拟车道线制作.md)。

## 常见问题

### 进行 Sensor Calibration 任务后，邮件显示任务失败

建议检查一下输入路径是否正确。

### 标定结果效果较差

- 标定时，确保 GNSS 信号状态良好，周围有轮廓清晰的静态障碍物。

- 保证传感器的安装精度，安装误差超过要求精度时，标定结果不容易收敛。

- 标定时，不要距离有效静态障碍物太远。
