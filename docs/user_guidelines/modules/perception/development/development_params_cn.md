# 插件开发

## 参数目录结构

首先介绍各个组件的配置目录，以激光雷达检测模块为例：

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

## 全局配置

全局配置在 gflag 文件中定义，包括`modules/perception/common/perception_gflags.h`。

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

modules/perception/common/onboard/common_flags/common_flags.h

| 参数名                     | 默认值 | 含义                                |
| -------------------------- | ------ | ----------------------------------- |
| obs_enable_hdmap_input     | true   | 为 roi 过滤器启用 hdmap 输入        |
| obs_enable_visualization   | false  | 是否发送可视化消息                  |
| obs_screen_output_dir      | ./     | 输出目录，用于保存可视化屏幕截图    |
| obs_benchmark_mode         | false  | 是否开启 benchmark 模式，默认 false |
| obs_save_fusion_supplement | false  | 是否保存融合补充数据，默认false     |
| start_visualizer           | false  |                                     |

### 修改配置

全局配置修改在`/apollo/modules/perception/data/flag/perception_common.flag`中，例如我们要修改 obs_enable_hdmap_input 的值为 false，可以在文件中添加：

```bash
--obs_enable_hdmap_input=false
```

## 组件配置

各个组件的配置在对应组件的 conf 和 data 目录，直接修改对应文件的值即可，以 3D 目标检测模块（lidar_detection）为例。

lidar_detection 模块配置文件路径：

- modules/perception/lidar_detection/conf
- modules/perception/lidar_detection/data

ModelInfo 配置，文件路径：modules/perception/common/proto/model_info.proto：

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

PointCloudPostProcess

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

modules/perception/lidar_detection/detector/cnn_segmentation/proto/model_param.proto

<table>
<thead>
  <tr>
    <th colspan="2">参数类型</th>
    <th>参数名</th>
    <th>默认值</th>
    <th>含义</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td colspan="2">ModelInfo</td>
    <td>info</td>
    <td>/</td>
    <td>模型基本配置</td>
  </tr>
  <tr>
    <td rowspan="7">FeatureParam</td>
    <td>float</td>
    <td>point_cloud_range</td>
    <td>90</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td>uint32</td>
    <td>width</td>
    <td>864</td>
    <td>BEV宽度</td>
  </tr>
  <tr>
    <td>uint32</td>
    <td>height</td>
    <td>864</td>
    <td>BEV高度</td>
  </tr>
  <tr>
    <td>float</td>
    <td>min_height</td>
    <td>-5.0</td>
    <td>点云z值最小值</td>
  </tr>
  <tr>
    <td>float</td>
    <td>max_height</td>
    <td>5.0</td>
    <td>点云z值最大值</td>
  </tr>
  <tr>
    <td>bool</td>
    <td>use_intensity_feature</td>
    <td>true</td>
    <td>是否使用强度特征</td>
  </tr>
  <tr>
    <td>bool</td>
    <td>use_constant_feature</td>
    <td>false</td>
    <td>是否使用常数特征</td>
  </tr>
  <tr>
    <td colspan="2">bool</td>
    <td>do_classification</td>
    <td>true</td>
    <td>是否预测分类信息</td>
  </tr>
  <tr>
    <td colspan="2">bool</td>
    <td>do_heading</td>
    <td>true</td>
    <td>是否预测朝向角信息</td>
  </tr>
  <tr>
    <td colspan="2">PointCloudPreProcess</td>
    <td>preprocess</td>
    <td>/</td>
    <td>预处理信息</td>
  </tr>
  <tr>
    <td>SppEngineConfig</td>
    <td>float</td>
    <td>height_gap</td>
    <td>0.5</td>
    <td>高度差距</td>
  </tr>
  <tr>
    <td colspan="2">bool</td>
    <td>remove_ground_points</td>
    <td>true</td>
    <td>是否过滤掉障碍物中的地面点</td>
  </tr>
  <tr>
    <td colspan="2">float</td>
    <td>objectness_thresh</td>
    <td>0.5</td>
    <td>objectness的阈值</td>
  </tr>
  <tr>
    <td colspan="2">float</td>
    <td>confidence_thresh</td>
    <td>0.1</td>
    <td>confidence的阈值</td>
  </tr>
  <tr>
    <td colspan="2">float</td>
    <td>height_thresh</td>
    <td>0.5</td>
    <td>高度阈值</td>
  </tr>
  <tr>
    <td colspan="2">uint32</td>
    <td>min_pts_num</td>
    <td>3</td>
    <td>目标最少点数</td>
  </tr>
  <tr>
    <td colspan="2">float</td>
    <td>confidence_range</td>
    <td>85</td>
    <td>置信度范围</td>
  </tr>
  <tr>
    <td colspan="2">bool</td>
    <td>fill_recall_with_segmentor</td>
    <td>true</td>
    <td>是否使用背景分割</td>
  </tr>
</tbody>
</table>

centerpoint 配置在 modules/perception/lidar_detection/data/center_point_param.pb.txt 中，通过修改其中的值来选择不同配置。

| 参数类型              | 参数名            | 默认值 | 含义                        |
| --------------------- | ----------------- | ------ | --------------------------- |
| ModelInfo             | info              | /      | 模型通用配置                |
| PointCloudPreProcess  | preprocess        | /      | 预处理                      |
| PointCloudPostProcess | postprocess       | /      | 后处理                      |
| int32                 | point2box_max_num | 5      | 每个点最多可以属于多少个box |
| float                 | quantize          | 0.2    |                             |

## 激光雷达感知参数开发模式

### 进入Docker环境

```bash
# 进入容器
aem enter

# 下载安装依赖包： 会拉取安装 core 目录下的 cyberfile.xml 里面所有的依赖包
buildtool build --gpu
```

### 启动Dreamview+

```bash
aem bootstrap start --plus
```

> 注意：plus 参数指的是启动 dreamview+，如需启动老版本 dreamview，去掉 --plus 参数即可。

### 修改参数

#### 修改模型输出置信度

通过调整模型输出置信度，学习如何调整模型的检测结果输出。模型的置信度越高，检测的结果越准确，但召回率会变低，模型的置信度变低，召回率变高，准确率变低，因此要平衡准确率和召回率，来达到一个比较好的效果。

首先检查`/apollo/modules/perception/lidar_detection/conf/lidar_detection_config.pb.txt`中的配置是否为 CenterPointDetection。

```bash
plugin_param {
  name: "CenterPointDetection"
  config_path: "perception/lidar_detection/data"
  config_file: "center_point_param.pb.txt"
}
```

然后在`/apollo/modules/perception/lidar_detection/data/center_point_param.pb.txt`修改 score_threshold:

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

![修改模型输出.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E4%BF%AE%E6%94%B9%E6%A8%A1%E5%9E%8B%E8%BE%93%E5%87%BA_9a82260.png)

#### 修改 ROI 范围

修改 ROI 参数可以通过规则过滤不需要关注的障碍物，避免对自动驾驶汽车的干扰。

找到`/apollo/modules/perception/lidar_detection_filter/data/roi_boundary_filter.pb.txt`文件，修改参数：

```bash
distance_to_boundary_threshold: 0.0
confidence_threshold: 0.0
cross_roi_threshold: 0.6
inside_threshold: -1.0
```

![修改ROI.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E4%BF%AE%E6%94%B9ROI_108597c.png)

### 启动 lidar 感知程序

选择相应车型配置：

![启动lidar感知程序.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%90%AF%E5%8A%A8lidar%E6%84%9F%E7%9F%A5%E7%A8%8B%E5%BA%8F_bff5aaf.png)

启动transform 模块：

![启动transform模块1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%90%AF%E5%8A%A8transform%E6%A8%A1%E5%9D%971_a46d7a4.png)

启动 lidar 感知模块：

![启动lidar 感知模块.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%90%AF%E5%8A%A8lidar%20%E6%84%9F%E7%9F%A5%E6%A8%A1%E5%9D%97_60b8392.png)

下载并播放感知包，在 Dreamview+ 左下角点击 Resource Manager，下载 sensor_rgb 数据包，下载完成后选择 sensor_rgb 数据包点击播放。

![lidar.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/lidar_0106d45.png)

![lidar 2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/lidar%202_c46da07.png)

dreamview+ 查看 lidar 检测结果：

![查看lidar检测效果.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%9F%A5%E7%9C%8Blidar%E6%A3%80%E6%B5%8B%E6%95%88%E6%9E%9C_add2416.png)
