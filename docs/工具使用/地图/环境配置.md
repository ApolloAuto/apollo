首先联系 apollo 团队，为自己的 apollo 账号添加地图工具权限。

## 步骤一：环境部署

地图工具支持在 x86_64 和 aarch64 两种架构上运行，部署方式：

```bash
# 下载apollo开发工程
git clone https://github.com/ApolloAuto/application-core
# 进入工程目录
cd application-core/
# 拉取最新的配置
git pull
# 环境配置：会识别主机系统是x86_64还是aarch64修改对应的.env 和 .workspace.json配置
bash setup.sh
# 启动容器
aem start_gpu
# 进入容器
aem enter
# 安装
buildtool build

# application-core里面profiles目录提供了基于1个16线lidar和2个camera的样例配置，您可以复制sample目录根据自己的车型修改里面的配置
# 选择您的profile，以使用sample profile为例：
aem profile use sample
# 注意：profile use命令可能会有一些warning提示，通常可以忽略。
```

## 步骤二：配置文件

### 模式配置文件说明

模式配置文件包括地图采集模式配置和地图编辑模式配置文件。

#### 地图采集模式配置文件

路径：`/apollo/modules/dreamview_plus/conf/hmi_modes/map_collect.pb.txt`。

> 说明：里面主要是启动 lidar、gps、localization、transform 几个模块的配置，通常使用默认配置即可，如果需要修改注意 dag、conf 配置文件路径要正确。

#### 地图编辑模式配置文件

路径：`/apollo/modules/dreamview_plus/conf/hmi_modes/map_editor.pb.txt`。

> 说明：里面配置不需要更改。

### 参数配置说明

#### 地图采集参数配置

路径：`/apollo/modules/private_tools/tile_map_images_creator/conf/image_creator_conf.pb.txt`。

> 说明：以下红色配置是您需要修改或者注意的配置，其他通常用默认值即可。
> 
> * reader_conf ：
>   * point_cloud_channel ：绘制底图的点云 channel，通常写 compensator 的点云 channel 名称，
> * point_cloud_processing_conf：
>   * intensity_converter_conf（重点关注这个配置项）：sigmoid 变化器的配置项。包含 k 和 x0 两项，一般只需要关注 x0 项，将 x0 调大一般会让底图变暗，反之则变亮；根据目前一些的测试结果，在一些颜色较深的油漆路面x0 调整为 11.0~15.0 比较好；通常在颜色较浅的水泥路面x0 调整为 18.0~20.0 比较好。您可以先使用默认值。
>   * filter_conf：点云过滤条件，包括过滤的点云高度以及点云范围，通常不需要修改。
> * matrix_generator_conf。包含精度和初始生成的瓦片最大层级，精度指最大层级的瓦片每个图像的每个像素包含的物理距离，通常不需要修改。地图通常分为 0~4 层共五层，每张瓦片分辨率都是 1024\*1024。第四层瓦片每个像素点的物理距离为精度的大小，也就是 0.03125\*0.3125m^2，第四层瓦片每张大小表示 32\*32m^2 的物理范围。接着上层瓦片的表示物理范围的长宽都分别为下层瓦片的两倍，如第三层级表示 64\*64 的物理范围，第二层级表示 128\*128 的范围，依次类推。
> * sample_distance，目的是过滤距离太近的点云帧，限定用到的两帧点云之间的最小距离间隔，如果所有点云都需要用于渲染，则设为 0.0。

#### 地图编辑质检参数配置

路径：

* 质检项配置文件：`/apollo/modules/private_tools/map_tool/conf/map_validator_checker_conf.pb.txt`
* 质检登记配置文件：`modules/private_tools/map_tool/conf/map_validator_errcode_conf.pb.txt`

> 说明：
> 
> * 质检项配置：通常不需要修改，包括车道宽度检查、曲率检查、绘制的元素是否和车道关联、停车位大小检查等；
> * 质检等级配置：Error 类型的会拦截地图发布，Warning 只是提出警告，不拦截地图发布。

#### SLAM 建图相关配置

如果您使用 slam 模式建图，需要关注以下配置。

**laser_multiscan_registration_conf**

路径：`/apollo/modules/loam_velodyne/conf/laser_multiscan_registration_conf.pb.txt`

> 说明：主要修改 channel 配置和 imu 配置。
> 
> * pointCloudTopic ：改成使用 slam 建图的点云 channel（注意需要是 lidar 原始点云，不要写融合或者运动补偿的点云），
> * IMU Settings：包括 IMU 类型（6 轴或 9 轴）、IMU 频率、IMU 噪声、偏置等设置，默认提供的是星宇网达 M2 的配置信息。

**laser_odometry_conf**

路径：`/apollo/modules/loam_velodyne/conf/laser_odometry_conf.pb.txt`

> 说明：主要修改 channel 配置。
> 
> * pointCloudTopic：和上述点云 channel 一致，
> * lidarFrame：和 lidar 输出的 fram id 一致，
> * Sensor Settings：激光雷达参数的配置，包括雷达类型（一般均可设置 velodyne）、雷达线束、水平分辨率等。

**localization_conf**

路径：`/apollo/modules/loam_velodyne/conf/localization_conf.pb.txt`

> 说明：RTK参数配置。
> 
> * pointCloudTopic：和上述点云 channel 一致，
> * GPS Settings：RTK 协方差的阈值，通常不需要修改，
> * CPU Params、Surrounding map、 Loop closure 等是建图的一些配置，通常不需要修改。

## 步骤三：启动服务

1. 在 docker 环境下输入以下命令启动 Dreamview+：
   
   ```bash
   aem bootstrap restart --plus
   ```
   
   在浏览器端通过访问机器的 8888 端口打开 Dreamview+ 页面。

2. 页面左下角有登录信息说明登录成功。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5341e20.png)

3. 模式选择中选择 **Map Collect** 完成底图采集。参见地图采集相关文档。

4. 模式选择中选择 **Map Editor** 完成地图编辑。参见地图编辑相关文档。

## 步骤四：结果目录说明

结果都在工程的 data 目录下保存：

* record 数据:  每次采集会把 record 数据保存在data/records 目录下以`map_record`开头文件夹内。
* 底图：`data/base_map`，可以直接拷贝到其他机器上使用。
* 地图：`data/map_data`，可以直接拷贝到其他机器上使用。
* 编辑地图：`data/editor_map`，可以直接拷贝到其他机器上使用。
