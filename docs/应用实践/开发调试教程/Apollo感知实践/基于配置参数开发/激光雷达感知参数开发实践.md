激光雷达和相机是目前自动驾驶中最重要的 2 个传感器，由于激光雷达测距准确，因此检测出的障碍物的精度普遍会比相机高。

激光雷达（Laser Radar，简称 LiDAR）是一种通过激光束来探测目标位置、反射强度等特征量的传感器。它由激光发射机、光学接收机、转台和信息处理系统等组成。激光器将电脉冲变成光脉冲发射出去，光接收机再把从目标反射回来的光脉冲还原成电脉冲，送到显示器。

激光雷达相比传统雷达，具有高分辨率、高精度、抗有源干扰能力强等优点。它的工作原理是利用激光的波长和脉冲宽度，通过测量光的传播时间来精确测量距离。激光雷达的发射系统口径很小，可接收区域窄，有意发射的激光干扰信号进入接收机的概率极低。

根据扫描系统的差异，激光雷达可以分为机械式、混合固态（转镜式、棱镜式、MEMS振镜）、固态（FLASH、OPA）等。根据测距方法的差异，就可以分为 TOF 激光雷达、FMCW 激光雷达。

![lidarintro.gif](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/lidarintro_8e9ce55.gif)

## 激光雷达感知整体框架

Apollo 激光雷达的感知由以下几个组件构成，它们一起工作共同实现了激光雷达感知功能。

![激光雷达感知整体框架.jpeg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E6%84%9F%E7%9F%A5%E6%95%B4%E4%BD%93%E6%A1%86%E6%9E%B6_ae57744.jpeg)

在介绍各个模块之前，我们首先需要知道激光雷达感知模块的输入和输出，以及它的工作原理。

激光雷达感知模块的输入：

- 激光雷达点云。由激光雷达传感器生成，通过扫描周围环境得到一系列的点和反射强度值。订阅消息的通道`/apollo/sensor/lidar16/compensator/PointCloud2`，

- 坐标转换。检测到的障碍物在激光雷达坐标系，需要转换到世界坐标系下，因此需要激光雷达到世界坐标的转换关系。订阅消息的通道`/tf和/tf_static`。

激光雷达感知模块的输出：

- 障碍物信息。通过点云检测到的障碍物信息，包括目标的类别，大小，速度以及追踪 ID 等，用于后续的规划模块进行路径规划。发布消息的通道为`/apollo/perception/obstacles`。

  ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_90de77a.png)

接下来我们详细介绍这几个模块。

### 点云预处理（pointcloud_preprocess）

点云预处理是激光雷达感知的第一个组件，它的主要功能如下：

- 过滤无效点，即过滤坐标为 nan 的点

- 过滤高度超过 z_threshold 的点。比如过滤高度大于 6 米的点。

- 通过自车的点，考虑到激光雷达安装位置，有可能扫描到自己，因此过滤自身的点，避免误当作障碍物。

### 高精度地图过滤（pointcloud_map_based_roi）

过滤 ROI 之外的点云。

感兴趣区域 (ROI) 指定可行驶区域，包括从高精地图检索到的路面和路口。 HDMap ROI 过滤器处理 ROI 外部的 lidar 点，去除背景物体，例如道路周围的建筑物和树木。剩下的就是 ROI 中的点云以供后续处理。

### 地面检测（pointcloud_ground_detection）

查找地面，查找地面的目的主要是为了后续做点云分割。

![地面检测.gif](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/%E5%9C%B0%E9%9D%A2%E6%A3%80%E6%B5%8B_4d2a947.gif)

### 3D 目标检测（lidar_detection）

通过深度学习算法，对上述处理过的点云进行目标检测，最后得到当前场景下的行人、车辆和非机动车辆的位置和速度信息等。

目前 Apollo 一共支持以下 4 种激光雷达目标检测模型。

- **center_point_detection** ：是一种基于点云的 anchor-free 的三维目标检测算法，该模型在计算效率和准确度上有一定的优势。其核心思想是通过预测物体的中心点来进行目标检测和位置回归，而不需要预先产生大量候选框（anchor）。

- **cnn_segmentation** ：自研模型。

- **mask_pillars_detection** ：自研模型，基于 point_pillars 做了一些改进。

- **point_pillars_detection** ：是一种用于自动驾驶的3D目标检测模型，它基于点云数据实现。该模型通过将点云数据转化为柱形稀疏表示，然后结合 2D CNN 和 3D SSD 来进行目标检测。

目前支持的在线推理框架有：

- pytorch

- paddle

- onnx

- TensorRT

已经开放训练代码的模型是 center_point 激光雷达检测模型，后续会讲到如何进行模型训练和部署。

### 检测结果过滤（lidar_detection_filter）

根据对象属性、车道线、ROI 等过滤模型检测出的前景和背景障碍物。

### 目标追踪（lidar_tracking）

对检测到的障碍物进行追踪，从而获取到目标的速度和追踪 ID，用于后续进行目标轨迹预测。

### 消息转发（msg_adapter）

msg 适配器模块用于在感知模块内部转发消息。

内部消息的通道名称通常有一个内部字段“inner”。 这些消息仅在线程之间传递，而不是在进程之间传递，因此您无法使用 cyber_monitor 来查看它们。因为这些消息一般都比较大，比如图像、点云数据，为了避免复制、序列化和反序列化，我们采用直接传递对象的方法，所以这些消息只能在线程内传递。

为了方便调试，可以使用 msg 适配器模块转发消息，这样就可以通过 cyber_monitor 查看。而且单纯的激光雷达感知不需要进行传感器融合，因此需要通过 msg_adapter 模块进行消息转发输出，如果有多传感器融合模块，则需要启动激光雷达、相机和毫米波雷达模块加上融合模块才能输出最终的感知结果。

## 激光雷达感知参数介绍

接下来我们会介绍第一种感知开发模式：修改激光雷达感知模块参数。

### 参数目录结构

首先介绍各个组件的配置目录，以激光雷达检测模块为例

```bash
├── lidar_detection
    ├── conf            // 组件配置文件
    ├── dag             // 组件dag文件
    ├── data            // 功能配置文件
    ├── detector
    │   ├── center_point_detection
    │   ├── cnn_segmentation
    │   ├── mask_pillars_detection
    │   └── point_pillars_detection
    ├── interface
    ├── proto
    ├── lidar_detection_component.cc
    ├── lidar_detection_component.h
    ├── cyberfile.xml   // 包配置文件
    ├── README.md
    └── BUILD
```

公共配置目录，多个组件的公共配置放到`modules/perception/data`路径下。

```bash
modules/perception/data
├── BUILD
├── conf    // 感知公共配置，例如一些单例
├── cyberfile.xml
├── flag    // 感知模块所有的gflags命令行传入，用于dag
├── models  // 模型存放路径
└── params  // 传感器内外参
```

### 全局配置

gflag配置：`modules/perception/common/perception_gflags.h`

| 参数名                       | 默认值                                                                                                                | 含义                                 |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| obs_sensor_intrinsic_path    | /apollo/modules/perception/data/params                                                                                | 传感器内参路径                       |
| obs_sensor_meta_file         | sensor_meta.pb.txt                                                                                                    | 传感器meta文件名                     |
| enable_base_object_pool      | enable_base_object_pool                                                                                               | 开启对象池                           |
| config_manager_path          | ./                                                                                                                    | 配置管理器路径                       |
| work_root                    | /apollo/modules/perception                                                                                            | 工作目录                             |
| onnx_obstacle_detector_model | /apollo/modules/perception/camera/lib/obstacle/detector/yolov4/model/yolov4_1_3_416_416.onnx                          | 目标检测模型                         |
| onnx_test_input_path         | /apollo/modules/perception/inference/onnx/testdata/dog.jpg                                                            | 目标检测测试路径                     |
| onnx_test_input_name_file    | /apollo/modules/perception/inference/onnx/testdata/coco.names                                                         | 目标检测测试文件                     |
| onnx_prediction_image_path   | /apollo/modules/perception/inference/onnx/testdata/prediction.jpg                                                     | 目标检测预测图像路径                 |
| num_classes                  | 80                                                                                                                    | 目标类型数量                         |
| torch_detector_model         | /apollo/modules/perception/camera/lib/obstacle/detector/yolov4/model/yolov4.pt                                        | torch检测模型路径                    |
| lidar_sensor_name            | velodyne128                                                                                                           | lidar传感器名称                      |
| use_trt                      | false                                                                                                                 | 是否使用tensorrt                     |
| trt_precision                | 1                                                                                                                     | tensorrt的精度，0: 32float，1: kInt8 |
| trt_use_static               | true                                                                                                                  | 是否从磁盘路径加载tensorrt图优化     |
| use_calibration              | true                                                                                                                  | 是否使用校正表                       |
| use_dynamicshape             | true                                                                                                                  | 是否使用动态形状                     |
| collect_shape_info           | true                                                                                                                  | 是否采集动态形状信息                 |
| dynamic_shape_file           | /apollo/modules/perception/lidar_detection/data/center_point_paddle/pillar_20_0625/collect_shape_info_3lidar_20.pbtxt | 动态文件路径                         |
| object_template_file         | object_template.pb.txt                                                                                                | 对象模版配置文件                     |
| hdmap_sample_step            | 5                                                                                                                     | 高精度地图采样率                     |
| scene_manager_file           | scene_manager.conf                                                                                                    | 场景管理器配置文件                   |
| roi_service_file             | roi_service.conf                                                                                                      | ROI服务配置文件                      |
| ground_service_file          | ground_service.conf                                                                                                   | 地面检测服务配置文件                 |

`modules/perception/common/onboard/common_flags/common_flags.h`：

| 参数名                     | 默认值 | 含义                                |
| -------------------------- | ------ | ----------------------------------- |
| obs_enable_hdmap_input     | true   | 为 roi 过滤器启用 hdmap 输入        |
| obs_enable_visualization   | false  | 是否发送可视化消息                  |
| obs_screen_output_dir      | ./     | 输出目录，用于保存可视化屏幕截图    |
| obs_benchmark_mode         | false  | 是否开启 benchmark 模式，默认 false |
| obs_save_fusion_supplement | false  | 是否保存融合补充数据，默认false     |
| start_visualizer           | false  |                                     |

### 点云预处理配置（pointcloud_preprocess）

`pointcloud_preprocessor`配置：

`modules/perception/pointcloud_preprocess/data/pointcloud_preprocessor.pb.txt`

| 参数类型 | 参数名                   | 默认值     | 含义                                        |
| -------- | ------------------------ | ---------- | ------------------------------------------- |
| bool     | filter_naninf_points     | true       | 是否过滤nan点                               |
| bool     | filter_nearby_box_points | true       | 是否过滤自车周围的点（大小由以下4个值确定） |
| float    | box_forward_x            | 1.0（米）  | 自车imu到右边界距离                         |
| float    | box_backward_x           | -1.0（米） | 自车imu到左边界距离                         |
| float    | box_forward_y            | 2.0（米）  | 自车imu到前边界距离                         |
| float    | box_backward_y           | -1.5（米） | 自车imu到后边界距离                         |
| bool     | filter_high_z_points     | true       | 是否过滤超过高度阈值的点                    |
| float    | z_threshold              | 2.0（米）  | 高度过滤阈值                                |

### 高精度地图过滤（pointcloud_map_based_roi）

`hdmap_roi_filter`配置：

`modules/perception/pointcloud_map_based_roi/data/hdmap_roi_filter.pb.txt`

| 参数类型 | 参数名          | 默认值 | 含义                                       |
| -------- | --------------- | ------ | ------------------------------------------ |
| float    | range           | 120.0  | 基于LiDAR传感器点的2D网格ROI LUT的图层范围 |
| float    | cell_size       | 0.25   | 用于量化2D网格的单元格的大小。             |
| float    | extend_dist     | 0      | 外扩ROI边界的距离。                        |
| bool     | no_edge_table   | false  | 是否有edge table                           |
| bool     | set_roi_service | true   | 是否启用ROI Service                        |

`map_manager`配置：

`modules/perception/pointcloud_map_based_roi/data/map_manager.pb.txt`

| 参数类型 | 参数名              | 默认值      | 含义                            |
| -------- | ------------------- | ----------- | ------------------------------- |
| bool     | update_pose         | false       | 是否更新lidar到世界坐标系的位姿 |
| float    | roi_search_distance | 120.0（米） | ROI搜索的距离范围               |

### 地面检测（pointcloud_ground_detection）

`spatio_temporal_ground_detector`配置文件：
`modules/perception/pointcloud_ground_detection/data/spatio_temporal_ground_detector.pb.txt`

| 参数类型 | 参数名             | 默认值 | 含义                           |
| -------- | ------------------ | ------ | ------------------------------ |
| uint32   | grid_size          | 16     | 网格size                       |
| float    | ground_thres       | 0.25   | 地面高度阈值，低于则认为是地面 |
| float    | roi_rad_x          | 120.0  | x方向roi radius                |
| float    | roi_rad_y          | 120.0  | y方向roi radius                |
| float    | roi_rad_z          | 100.0  | z反向roi radius                |
| uint32   | nr_smooth_iter     | 5      | smooth迭代次数                 |
| bool     | use_roi            | true   | 是否用roi内的点云              |
| bool     | use_ground_service | true   | 是否用ground service           |

### 3D目标检测（lidar_detection）

此模块配置文件路径：
`modules/perception/lidar_detection/conf`，
`modules/perception/lidar_detection/data`，

ModelInfo 配置，文件路径：`modules/perception/common/proto/model_info.proto`。

| 参数类型 | 参数名                 | 默认值 | 含义                          |
| -------- | ---------------------- | ------ | ----------------------------- |
| string   | name                   | /      | 模型名称，同models/下文件夹名 |
| string   | framework              | /      | 模型推理框架                  |
| string   | ModelFile.proto_file   | /      | 模型网络结构                  |
| string   | ModelFile.weight_file  | /      | 模型权重文件                  |
| string   | ModelFile.anchors_file | /      | anchor size                   |
| string   | ModelBlob.inputs       | /      | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs      | /      | 模型输出数据名称及维度        |

PointCloudPreProcess：

| 参数类型 | 参数名                       | 默认值     | 含义                                             |
| -------- | ---------------------------- | ---------- | ------------------------------------------------ |
| int32    | gpu_id                       | 0          | GPU的id                                          |
| double   | normalizing_factor           | 255        | 强度归一化的缩放因子                             |
| int32    | num_point_feature            | 4          | 每个点的特征数量                                 |
| bool     | enable_ground_removal        | false      | 是否过滤掉地面点                                 |
| double   | ground_removal_height        | -1.5       | 过滤掉z值小于阈值的点                            |
| bool     | enable_downsample_beams      | false      | 是否根据beam id对点云进行过滤                    |
| int32    | downsample_beams_factor      | 4          | 保留beam id为downsample_beams_factor的倍数的点云 |
| bool     | enable_downsample_pointcloud | false      | 是否根据voxel过滤点云                            |
| double   | downsample_voxel_size_x      | 0.01       | 过滤时voxel的x方向长度                           |
| double   | downsample_voxel_size_y      | 0.01       | 过滤时voxel的y方向长度                           |
| double   | downsample_voxel_size_z      | 0.01       | 过滤时voxel的z方向长度                           |
| bool     | enable_fuse_frames           | false      | 是否融合多帧点云                                 |
| int32    | num_fuse_frames              | 5          | 融合点云的帧数                                   |
| double   | fuse_time_interval           | 0.5        | 融合点云的时间间隔                               |
| bool     | enable_shuffle_points        | false      | 是否打乱点云索引                                 |
| int32    | max_num_points               | 2147483647 | 允许的最大点云数量                               |
| bool     | reproduce_result_mode        | false      | 是否开启复现结果模式                             |
| bool     | enable_roi_outside_removal   | false      | 是否在输入模型之前将roi外的点云进行过滤          |

PointCloudPostProcess：

| 参数类型 | 参数名                 | 默认值 | 含义                             |
| -------- | ---------------------- | ------ | -------------------------------- |
| float    | score_threshold        | 0.5    | 置信度阈值                       |
| float    | nms_overlap_threshold  | 0.5    | NMS的iou阈值                     |
| int32    | num_output_box_feature | 7      | 输出障碍物的属性个数             |
| float    | bottom_enlarge_height  | 0.25   | 获取目标真实点云时向上扩充的范围 |
| float    | top_enlarge_height     | 0.25   | 获取目标真实点云时向下扩充的范围 |
| float    | width_enlarge_value    | 0      | 获取目标真实点云时宽度扩充的范围 |
| float    | length_enlarge_value   | 0      | 获取目标真实点云时长度扩充的范围 |

cnnseg 配置
`modules/perception/lidar_detection/detector/cnn_segmentation/proto/model_param.proto`：

<table><thead><tr><th>参数类型</th><th></th><th>参数名</th><th>默认值</th><th>含义</th></tr></thead><tbody><tr><td>ModelInfo</td><td></td><td>info</td><td>/</td><td>模型基本配置</td></tr><tr><td rowspan="7">FeatureParam</td><td>float</td><td>point_cloud_range</td><td>90</td><td>点云范围</td></tr><tr><td>uint32</td><td>width</td><td>864</td><td>BEV宽度</td></tr><tr><td>uint32</td><td>height</td><td>864</td><td>BEV高度</td></tr><tr><td>float</td><td>min_height</td><td>-5.0</td><td>点云z值最小值</td></tr><tr><td>float</td><td>max_height</td><td>5.0</td><td>点云z值最大值</td></tr><tr><td>bool</td><td>use_intensity_feature</td><td>true</td><td>是否使用强度特征</td></tr><tr><td>bool</td><td>use_constant_feature</td><td>false</td><td>是否使用常数特征</td></tr><tr><td>bool</td><td></td><td>do_classification</td><td>true</td><td>是否预测分类信息</td></tr><tr><td>bool</td><td></td><td>do_heading</td><td>true</td><td>是否预测朝向角信息</td></tr><tr><td>PointCloudPreProcess</td><td></td><td>preprocess</td><td>/</td><td>预处理信息</td></tr><tr><td>SppEngineConfig</td><td>float</td><td>height_gap</td><td>0.5</td><td>高度差距</td></tr><tr><td>bool</td><td></td><td>remove_ground_points</td><td>true</td><td>是否过滤掉障碍物中的地面点</td></tr><tr><td>float</td><td></td><td>objectness_thresh</td><td>0.5</td><td>objectness的阈值</td></tr><tr><td>float</td><td></td><td>confidence_thresh</td><td>0.1</td><td>confidence的阈值</td></tr><tr><td>float</td><td></td><td>height_thresh</td><td>0.5</td><td>高度阈值</td></tr><tr><td>uint32</td><td></td><td>min_pts_num</td><td>3</td><td>目标最少点数</td></tr><tr><td>float</td><td></td><td>confidence_range</td><td>85</td><td>置信度范围</td></tr><tr><td>bool</td><td></td><td>fill_recall_with_segmentor</td><td>true</td><td>是否使用背景分割</td></tr></tbody></table>

centerpoint配置：

`modules/perception/lidar_detection/data/center_point_param.pb.txt`

| 参数类型              | 参数名            | 默认值 | 含义                        |
| --------------------- | ----------------- | ------ | --------------------------- |
| ModelInfo             | info              | /      | 模型通用配置                |
| PointCloudPreProcess  | preprocess        | /      | 预处理                      |
| PointCloudPostProcess | postprocess       | /      | 后处理                      |
| int32                 | point2box_max_num | 5      | 每个点最多可以属于多少个box |
| float                 | quantize          | 0.2    |                             |

### 检测结果过滤（lidar_detection_filter）

background_filter 配置：
`modules/perception/lidar_detection_filter/data/background_filter.pb.txt`

| 参数类型 | 参数名                      | 默认值 | 含义                   |
| -------- | --------------------------- | ------ | ---------------------- |
| float    | outside_roi_filter_distance | 1.0    | 在车道线之外的距离阈值 |

roi_boundary_filter配置：

`modules/perception/lidar_detection_filter/data/roi_boundary_filter.pb.txt`

| 参数类型 | 参数名                         | 默认值 | 含义                                                          |
| -------- | ------------------------------ | ------ | ------------------------------------------------------------- |
| float    | distance_to_boundary_threshold | 2.5    | 障碍物在车道线外时，距离车道线边界的阈值， 大于该阈值则被删除 |
| float    | confidence_threshold           | 0.0    | 目标置信度阈值                                                |
| float    | cross_roi_threshold            | 0.6    | 目标是否在roi交界处的阈值                                     |
| float    | inside_threshold               | -1.0   | 障碍物在车道线内时，距离车道线边界的阈值，                    |

### 目标追踪（lidar_tracking）

**fused_classifer**

fused_classifier 配置文件：

`modules/perception/lidar_tracking/data/fused_classifier/fused_classifier.pb.txt`

| 参数类型 | 参数名                 | 默认值                 | 含义                |
| -------- | ---------------------- | ---------------------- | ------------------- |
| float    | temporal_window        | 20.0                   | 时间窗口            |
| bool     | enable_temporal_fusion | true                   | 是否开启时间融合    |
| string   | one_shot_fusion_method | CCRFOneShotTypeFusion  | one shot fusion方法 |
| string   | sequence_fusion_method | CCRFSequenceTypeFusion | sequence fusion方法 |
| bool     | use_tracked_objects    | true                   | 是否使用跟踪的目标  |

ccrf_type_fusion配置文件：

| 参数类型 | 参数名                         | 默认值 | 含义              |
| -------- | ------------------------------ | ------ | ----------------- |
| string   | classifiers_property_file_path | null   | 分类属性文件路径  |
| string   | transition_property_file_path  | null   | 转移属性文件路径  |
| float    | transition_matrix_alpha        | 1.8    | 转移矩阵alpha参数 |

**tracker**

mlf_engine 配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_engine.pb.txt`

| 参数类型 | 参数名                  | 默认值 | 含义                      |
| -------- | ----------------------- | ------ | ------------------------- |
| string   | main_sensor             | /      | 主传感器                  |
| bool     | use_histogram_for_match | true   | 是否用histogram用来做匹配 |
| uint32   | histogram_bin_size      | 10     | histogram的大小           |
| bool     | output_predict_objects  | false  | 是否输出预测目标          |
| double   | reserved_invisible_time | 0.2    | 保留不可见时间            |
| bool     | use_frame_timestamp     | false  | 是否用frame时间戳         |

`mlf_track_object_matcher` 配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_track_object_matcher.conf`

| 参数类型 | 参数名                    | 默认值                       | 含义             |
| -------- | ------------------------- | ---------------------------- | ---------------- |
| string   | foreground_mathcer_method | MultiHmBipartiteGraphMatcher | 前景目标匹配方法 |
| string   | background_matcher_method | GnnBipartiteGraphMatcher     | 背景目标匹配方法 |
| float    | bound_value               | 100.0                        | 边界值           |
| float    | max_match_distance        | 4.0                          | 最大匹配距离     |

`mlf_track_object_distance` 配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_track_object_distance.conf`

| 参数类型 | 参数名                     | 默认值 | 含义                 |
| -------- | -------------------------- | ------ | -------------------- |
| string   | sensor_name_pair           | null   | 传感器名称pair       |
| float    | location_dist_weight       | 0      | 位置距离权重         |
| float    | direction_dist_weight      | 0      | 方向距离权重         |
| float    | bbox_size_dist_weight      | 0      | bbox大小距离权重     |
| float    | point_num_dist_weight      | 0      | 目标点云数量距离权重 |
| float    | histogram_dist_weight      | 0      | 统计直方图距离权重   |
| float    | centroid_shift_dist_weight | 0      | 中心点偏移距离权重   |
| float    | bbox_iou_dist_weight       | 0      | bbox iou距离权重     |
| float    | semantic_map_dist_weight   | 0      | 语义图距离权重       |

`mlf_tracker`配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_tracker.conf`。

| 参数类型 | 参数名      | 默认值 | 含义       |
| -------- | ----------- | ------ | ---------- |
| string   | filter_name | /      | filter名称 |

`mlf_motion_filter`配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_motion_filter.conf`。
| 参数类型 | 参数名 | 默认值 | 含义 |
|---|---|---|---|
| bool | use_adaptive | true | 是否用自适应方法 |
| bool | use_breakdown | true | 是否使用breakdown |
| bool | use_convergence_boostup | true | 是否用收敛启动 |
| double | init_velocity_variance | 5.0 | 速度方差初始参数 |
| double | init_acceleration_variance | 10.0 | 加速度方差初始参数 |
| double | measured_velocity_variance | 0.4 | 速度方差测量参数 |
| double | predict_variance_per_sqrsec | 1.0 | 预测方差参数 |
| uint32 | boostup_history_size_minimum | 3 | 历史最小值初始化参数 |
| uint32 | boostup_history_size_maximum | 6 | 历史最大值初始化参数 |
| double | converged_confidence_minimum | 0.5 | 最小收敛置信度参数 |
| double | noise_maximum | 0.1 | 最大噪声参数 |
| double | trust_orientation_range | 40 | 方向范围信任参数 |

`mlf_motion_refiner`配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_motion_refiner.conf`。

| 参数类型 | 参数名                         | 默认值 | 含义           |
| -------- | ------------------------------ | ------ | -------------- |
| double   | claping_acceleration_threshold | 10.0   | 加速度截断阈值 |
| double   | claping_speed_threshold        | 1.0    | 速度截断阈值   |

`mlf_shape_filter`配置文件：
`modules/perception/lidar_tracking/data/tracking/mlf_shape_filter.conf`。

| 参数类型 | 参数名                         | 默认值 | 含义             |
| -------- | ------------------------------ | ------ | ---------------- |
| double   | bottom_points_ignore_threshold | 0.1    | 底部点云忽略阈值 |
| double   | top_points_ignore_threshold    | 1.6    | 头部点云忽略阈值 |

### 消息转发（msg_adapter）

`modules/perception/msg_adapter/common/msg_adapter_gflags.h`

| 参数名                               | 默认值                       | 含义                               |
| ------------------------------------ | ---------------------------- | ---------------------------------- |
| cameraframe_to_obstacles_in          | /fake_topic                  | cameraframe输入                    |
| cameraframe_to_obstacles_out         | /apollo/perception/obstacles | cameraframe转obstacles输出         |
| sensorframe_message_to_obstacles_in  | /fake_topic                  | sensorframe_message输入            |
| sensorframe_message_to_obstacles_out | /apollo/perception/obstacles | sensorframe_message转obstacles输出 |
| lidarframe_to_obstacles_in           | /fake_topic                  | lidarframe输入                     |
| lidarframe_to_obstacles_out          | /apollo/perception/obstacles |                                    |

## 激光雷达感知参数开发模式

### 1. 进入Docker环境

```bash
# 进入容器
aem enter

# 下载安装依赖包： 会拉取安装core目录下的cyberfile.xml里面所有的依赖包
buildtool build --gpu
```

### 2. 启动Dreamview

```bash
aem bootstrap start --plus
```

> 注意：plus 参数指的是启动 dreamview+，如需启动老版本 dreamview，去掉 --plus 参数即可。

### 3. 查看感知模型

通过 amodel 命令查看当前系统中已经安装的感知模型。

```bash
amodel list
```

### 4. 修改参数

1. 修改模型输出置信度

   通过调整模型输出置信度，学习如何调整模型的检测结果输出。模型的置信度越高，检测的结果越准确，但召回率会变低，模型的置信度变低，召回率变高，准确率变低，因此要平衡准确率和召回率，来达到一个比较好的效果。

   首先检查`/apollo/modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt`中的配置是否为 CenterPointDetection：

   ```bash
   plugin_param {
      name: "CenterPointDetection"
      config_path: "perception/lidar_detection/data"
      config_file: "center_point_param.pb.txt"
    }
   ```

   然后在`/apollo/modules/perception/lidar_detection/data/center_point_param.pb.txt`修改`score_threshold`。

   ```bash
       postprocess {
     score_threshold: 0.25      // 检测结果置信度
     num_output_box_feature: 7
     bottom_enlarge_height: 0.1  // 0.25 -> 0.1
     top_enlarge_height: 0.25
     width_enlarge_value: 0
     length_enlarge_value: 0
   }
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ba00a57.png)

2. 修改 ROI 范围

   修改 ROI 参数可以通过规则过滤不需要关注的障碍物，避免对自动驾驶汽车的干扰。

   找到`/apollo/modules/perception/lidar_detection_filter/data/roi_boundary_filter.pb.txt`文件，修改参数

   ```bash
   distance_to_boundary_threshold: 0.0
   confidence_threshold: 0.0
   cross_roi_threshold: 0.6
   inside_threshold: -1.0
   ```

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_66d8cfb.png)

### 5. 启动lidar感知程序

选择相应车型配置

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_974600a.png)

启动 transform 模块

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9631794.png)

启动 lidar 感知模块

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_c73820a.png)

下载并播放感知包，在dreamview+左下角点击resource manger，下载sensorrgb数据包，下载完成后选择sensorrgb数据包点击播放。

![sensor数据包1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/sensor%E6%95%B0%E6%8D%AE%E5%8C%851_0106d45.png)

![sensor数据包2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/sensor%E6%95%B0%E6%8D%AE%E5%8C%852_c46da07.png)

dreamview+ 查看 lidar 检测结果：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cb8f0b8.png)
