## 概述

地图采集指将工具部署到车端后，随着车辆沿道路的移动，由传感器采集周围的环境和道路信息，完成点云底图生成，采集的原始地图数据，包含 RTK、激光雷达数据。地图采集工具依赖 lidar 驱动、GNSS 定位，请提前准备好。

## 操作步骤

### 步骤一：环境检测

1. 进入 Dreamview+ 界面，选择地图采集模式 **Map Collect** ，并点击 **Enter this mode** 进入地图采集模式。
   
   > 注意：地图采集只允许使用一个浏览器页面访问，建议使用 chrome 浏览器。

2. 打开 GPS 模块、Lidar 模块、定位模块、Transform 模块，并等待 GPS 模块，Lidar 模块、定位模块状态显示正常（绿色）。

3. 选择采集算法，目前提供了基于 RTK 定位点云融合的普通算法以及基于SLAM定位点云融合的 SLAM 算法，此处以 **普通算法** 为例。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_0b09997.png)

### 步骤二：数据采集

1. 点击 **开始采集** 按钮。

2. 移动车辆在需要采集的路线进行采集。
   
   > 注意：根据您的 lidar 配置，通常 32 线及以上的 lidar 每条道路走两次（一个来回）即可，一个 16 线的 lidar 建议每条路走 2~3 次。您可以先行驶一小段距离生成底图先查看下，根据效果判断是否要调整参数以及需要走的次数。

3. 采集路线行驶完毕后，点击 **结束采集** 按钮。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3c5bbf5.png)
   
   > 注意：slam 模式下，需要回环检测校准位置信息，需要计算一段时间。

4. 在结束采集后，系统会自动生成底图。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_b970989.png)

### 步骤三：文件导出

底图生成后，系统会自动预览并保存底图至默认文件夹。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7a529f0.png)

## 底图离线生成工具

为避免底图生成时出现异常要重新采集，我们提供了离线底图生成工具。

首先编辑底图生成配置：

```bash
vim /apollo/modules/private_tools/tile_map_images_creator/conf/image_creator_conf.pb.txt
```

修改`input_output_conf`的配置，以采集目录为`data/records/map_record_20240401103023`为例，修改为：

```bash
input_output_conf {
    input_dir: "/apollo_workspace/data/records/map_record_20240401103023"
    bin_output_dir: "/apollo/data/base_map/20240401103023/map_bin"
    images_output_dir: "/apollo/data/base_map/20240401103023/map_images"
    use_LRU_cache: true
    LRU_cache_size: 20
}
```

执行离线底图生成工具前需要启动transform模块，请提前在dreamview页面打开transform模块，或者手工执行：

```bash
mainboard  -d /apollo/modules/transform/dag/static_transform.dag &
```

然后执行离线底图生成工具：

```bash
tile_map_images_creator -c /apollo/modules/private_tools/tile_map_images_creator/conf/image_creator_conf.pb.txt
```

底图会生成在`/apollo/data/base_map/20240401103023`目录，打开地图编辑模式可以预览底图。
