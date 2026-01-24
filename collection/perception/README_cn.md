# 感知模块介绍

## 相机模块概述

相机模块包括多种纯视觉感知算法，包括环视bev算法、多阶段2d检测及3d姿态回归算法、单阶段算法。BEV检测模块使用了PETR模型，可以完成对多个摄像机数据的推理获得障碍物目标。多阶段检测模块是基于YOLO开发的多任务模型，可以同时输出数十个维度信息，如“2d”、“3d”和车辆转向信息。相机单阶段检测算法包含两个模型Caddn和SMOKE，可以同时输出目标的2d和3d信息，上述模块可以将结果传输到camera_tracking组件实现目标追踪。

### Channel+消息格式

| 模块                          | 输入通道                                                                                                                                                                                                                                                                                   | 输出通道                              | 消息格式                                        |
| ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------- | ----------------------------------------------- |
| camera_detection_bev          | /apollo/sensor/camera/CAM_FRONT/image <br/> /apollo/sensor/camera/CAM_FRONT_RIGHT/image <br/> /apollo/sensor/camera/CAM_FRONT_LEFT/image <br/> /apollo/sensor/camera/CAM_BACK/image <br/> /apollo/sensor/camera/CAM_BACK_LEFT/image <br/> /apollo/sensor/camera/CAM_BACK_RIGHT/image <br/> | /apollo/perception/obstacles          | apollo::perception::PerceptionObstacles         |
| camera_detection_multi_stage  | /apollo/sensor/camera/front_6mm/image                                                                                                                                                                                                                                                      | /perception/inner/Detection           | apollo::perception::onboard::CameraFrame        |
| camera_detection_single_stage | /apollo/sensor/camera/front_6mm/image                                                                                                                                                                                                                                                      | /perception/inner/Detection           | apollo::perception::onboard::CameraFrame        |
| camera_location_estimation    | /perception/inner/Detection                                                                                                                                                                                                                                                                | /perception/inner/location_estimation | apollo::perception::onboard::CameraFrame        |
| camera_location_refinement    | /perception/inner/location_estimation                                                                                                                                                                                                                                                      | /perception/inner/location_refinement | apollo::perception::onboard::CameraFrame        |
| camera_tracking               | /perception/inner/location_refinement                                                                                                                                                                                                                                                      | /perception/inner/PrefusedObjects     | apollo::perception::onboard::SensorFrameMessage |

### 参数

#### camera_detection_multi_stage 模块

此模块配置文件位置：

- modules/perception/camera_detection_multi_stage/conf

- modules/perception/camera_detection_multi_stage/data

算法基本配置： modules/perception/camera_detection_multi_stage/proto/camera_detection_multi_stage.proto

| 参数类型 | 参数名                                | 默认值                                       | 含义                         |
| -------- | ------------------------------------- | -------------------------------------------- | ---------------------------- |
| string   | camera_name                           | front_6mm                                    | 对应tf中摄像头名称，frame id |
| string   | PluginParam.name                      | YoloObstacleDetector                         | 检测算法                     |
| string   | PluginParam.config_path               | perception/camera_detection_multi_stage/data | 配置文件路径                 |
| string   | PluginParam.config_file               | yolo.pb.txt                                  | 配置文件名称                 |
| string   | channel.input_camera_channel_name     | /apollo/sensor/camera/front_6mm/image        | 输入消息通道                 |
| string   | channel.output_obstacles_channel_name | /perception/inner/Detection                  | 输出消息通道                 |
| int32    | gpu_id                                | 0                                            | gpu id                       |
| int32    | frame_capacity                        | 20                                           | 帧缓存容量                   |
| int32    | image_channel_num                     | 3                                            | 图片通道数两                 |
| bool     | enable_undistortion                   | false                                        | 不失真使能                   |
| bool     | enable_visualization                  | false                                        | 可视化使能                   |
| double   | default_camera_pitch                  | 0                                            | 默认摄像头俯仰角度           |
| double   | default_camera_height                 | 1.5                                          | 默认摄像头高度               |
| double   | ts_diff                               | 0.1                                          | 时间戳偏差值                 |
| bool     | output_final_obstacles                | false                                        | 是否输出最终障碍物           |
| string   | Debug.output_viz_message_channel_name | /perception/inner/camera_viz_msg             | 输出可视化调试信息通道       |
| bool     | Debug.output_camera_debug_msg         | false                                        | 是否输出摄像头调试信息       |
| string   | Debug.camera_debug_channel_name       | /perception/camera_debug                     | 输出调试信息通道             |
| string   | Debug.visual_debug_folder             | /apollo/debug_output                         | 可视化调试保存路径           |
| string   | Debug.visual_camera                   | front_6mm                                    | 摄像头名称                   |
| bool     | Debug.write_visual_img                | false                                        | 可视化状态量                 |

模型配置： modules/perception/camera_detection_multi_stage/detector/yolo/proto/model_param.proto

| 参数类型         | 参数名                   | 默认值  | 含义                     |
| ---------------- | ------------------------ | ------- | ------------------------ |
| common.ModelInfo | info                     | /       | 设定模型参数配置         |
| common.Resize    | resize                   | /       | 设定高度、宽度等         |
| common.Normalize | normalize                | /       | 设定均值、方差           |
| float            | offset_ratio             | 0.28889 | 偏移率                   |
| float            | cropped_ratio            | 0.71111 | 裁剪率                   |
| int32            | resized_width            | 1440    | resize 宽度              |
| int32            | aligned_pixel            | 32      | 对齐像素                 |
| float            | confidence_threshold     | 0.4     | 目标置信度               |
| float            | light_vis_conf_threshold | 0       | 车灯可视置信度           |
| float            | light_swt_conf_threshold | 0       | 车灯闪烁置信度           |
| int32            | ori_cycle                | 2       | 调整观测角范围           |
| bool             | per_cls_reg              | false   |                          |
| bool             | with_box3d               | true    | 是否计算3dbbox           |
| bool             | with_frbox               | false   | 是否计算前后bbox         |
| bool             | with_lights              | true    | 是否计算车灯             |
| bool             | with_ratios              | false   | 是否计算                 |
| int32            | num_areas                | 0       | 车身周围目标区域划分数量 |
| float            | border_ratio             | 0.01    | 边界比例阈值             |
| NMSParam         | nms_param                | /       | 设定 nms 参数            |
| float            | min_dims.min_2d_height   | 10      | 最小2d目标像素高度       |
| float            | min_dims.min_3d_height   | 0.1     | 最小3d目标高度           |
| float            | min_dims.min_3d_width    | 0       | 最小3d目标宽度           |
| float            | min_dims.min_3d_length   | 0       | 最小3d目标长度           |

ModelInfo配置：

| 参数类型 | 参数名                 | 默认值           | 含义                          |
| -------- | ---------------------- | ---------------- | ----------------------------- |
| string   | name                   | 3d-r4-half_caffe | 模型名称，同models/下文件夹名 |
| string   | framework              | TensorRT         | 模型推理框架                  |
| string   | ModelFile.proto_file   | deploy.pt        | 模型网络结构                  |
| string   | ModelFile.weight_file  | deploy.model     | 模型权重文件                  |
| string   | ModelFile.anchors_file | nchors.txt       | yolo2d框anchor size           |
| string   | ModelBlob.inputs       | 1440x800         | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs      | /                | 模型输出数据名称及维度        |

#### camera_detection_single_stage 模块

此模块配置位置：

- modules/perception/camera_detection_single_stage/conf

- modules/perception/camera_detection_single_stage/data

算法基本配置： modules/perception/camera_detection_single_stage/proto/camera_detection_single_stage.proto

| 参数类型 | 参数名                                | 默认值                                        | 含义                      |
| -------- | ------------------------------------- | --------------------------------------------- | ------------------------- |
| string   | camera_name                           | front_6mm                                     | 对应tf中摄像头名称，frame |
| float    | timestamp_offset                      | 0                                             | 时间偏移量                |
| string   | PluginParam.name                      | SmokeObstacleDetector                         | 检测算法                  |
| string   | PluginParam.config_path               | perception/camera_detection_single_stage/data | 配置文件路径              |
| string   | PluginParam.config_file               | smoke_config.pb.txt                           | 配置文件名称              |
| int32    | gpu_id                                | 0                                             | gpu id                    |
| string   | channel.input_camera_channel_name     | /apollo/sensor/camera/front_6mm/image         | 输入消息通道              |
| string   | channel.output_obstacles_channel_name | /perception/inner/Detection                   | 输出消息通道              |

模型配置： modules/perception/camera_detection_single_stage/detector/smoke/proto/model_param.proto

| 参数类型         | 参数名                 | 默认值   | 含义             |
| ---------------- | ---------------------- | -------- | ---------------- |
| common.ModelInfo | info                   | /        | 设定模型参数配置 |
| common.Resize    | resize                 | /        | 设定高度、宽度等 |
| common.Normalize | normalize              | /        | 设定均值、方差   |
| float            | offset_ratio           | x,y = 0  | 偏移率           |
| float            | confidence_threshold   | 0.5      | 目标置信度       |
| float            | min_dims.min_2d_height | 0.015625 | 最小2d目标高度   |
| float            | min_dims.min_3d_height | 0.1      | 最小3d目标高度   |
| float            | min_dims.min_3d_width  | 0        | 最小3d目标宽度   |
| float            | min_dims.min_3d_length | 0        | 最小3d目标长度   |
| int32            | ori_cycle              | 1        | 调整观测角范围   |
| float            | border_ratio           | 0.01     | 边界比例阈值     |

#### camera_detection_bev 模块

算法基本配置： modules/perception/camera_detection_bev/proto/camera_detection_bev.proto

| 参数类型 | 参数名                                | 默认值                                | 含义                         |
| -------- | ------------------------------------- | ------------------------------------- | ---------------------------- |
| string   | camera_name                           | front_6mm                             | 对应tf中摄像头名称，frame id |
| float    | timestamp_offset                      | 0                                     | 时间偏移量                   |
| string   | PluginParam.name                      | BEVObstacleDetector                   | 检测算法                     |
| string   | PluginParam.config_path               | perception/camera_detection_bev/data  | 配置文件路径                 |
| string   | PluginParam.config_file               | detr.pb.txt                           | 配置文件名称                 |
| int32    | gpu_id                                | 0                                     | gpu id                       |
| string   | channel.input_camera_channel_name     | /apollo/sensor/camera/front_6mm/image | 输入消息通道                 |
| string   | channel.output_obstacles_channel_name | /perception/inner/Detection           | 输出消息通道                 |

模型配置： modules/perception/camera_detection_bev/detector/detr/proto/model_param.proto

| 参数类型                       | 参数名          | 默认值 | 含义             |
| ------------------------------ | --------------- | ------ | ---------------- |
| common.ModelInfo               | info            | /      | 设定模型参数配置 |
| common.Resize                  | resize          | /      | 设定高度、宽度等 |
| common.Normalize               | normalize       | /      | 设定均值、方差   |
| float                          | score_threshold | 0      | 目标得分置信度   |
| camera_location_estimation模块 |

组件配置文件： modules/perception/camera_location_estimation/proto/camera_location_estimation.proto

| 参数类型 | 参数名                        | 默认值                                     | 含义             |
| -------- | ----------------------------- | ------------------------------------------ | ---------------- |
| string   | camera_name                   | front_6mm                                  | camera 名字      |
| string   | PluginParam.name              | MultiCueTransformer                        | 算法名称         |
| string   | PluginParam.config_path       | perception/camera_location_estimation/data | 算法配置文件路径 |
| string   | PluginParam.config_file       | config.pb.txt                              | 算法配置文件名称 |
| double   | timestamp_offset              | 0                                          | 时间戳偏移量     |
| string   | input_camera_channel_name     | /                                          | 数据输入通道     |
| string   | output_obstacles_channel_name | /                                          | 数据输出通道     |

算法配置文件： modules/perception/camera_location_estimation/transformer/multicue/proto/multicue.proto

| 参数类型 | 参数名            | 默认值 | 含义             |
| -------- | ----------------- | ------ | ---------------- |
| float    | min_dimension_val | 0.2    | 最小维度         |
| bool     | check_dimension   | true   | 是否检查维度信息 |

#### camera_location_refinement 模块

此模块配置文件位置：

- modules/perception/camera_location_refinement/conf

- modules/perception/camera_location_refinement/data

组件配置文件： modules/perception/camera_location_refinement/proto/camera_location_refinement.proto

| 参数类型 | 参数名                               | 默认值                                     | 含义               |
| -------- | ------------------------------------ | ------------------------------------------ | ------------------ |
| string   | camera_name                          | front_6mm                                  | camera 名字        |
| double   | timestamp_offset                     | 0                                          | 时间戳偏移量       |
| string   | input_camera_channel_name            | /                                          | 数据输入通道       |
| string   | output_obstacles_channel_name        | /                                          | 数据输出通道       |
| int32    | lane_calibration_working_sensor_name | front_6mm                                  | 车道校准传感器名称 |
| string   | PostprocessorParam.name              | MultiCueTransformer                        | 算法名称           |
| string   | PostprocessorParam.config_path       | perception/camera_location_estimation/data | 算法配置文件路径   |
| string   | PostprocessorParam.config_file       | config.pb.txt                              | 算法配置文件名称   |
| string   | CalibrationServiceParam.name         | MultiCueTransformer                        | 算法名称           |
| string   | CalibrationServiceParam.config_path  | perception/camera_location_estimation/data | 算法配置文件路径   |
| string   | CalibrationServiceParam.config_file  | config.pb.txt                              | 算法配置文件名称   |

算法配置文件： modules/perception/camera_location_refinement/location_refiner/proto/location_refiner.proto

| 参数类型 | 参数名             | 默认值 | 含义                               |
| -------- | ------------------ | ------ | ---------------------------------- |
| float    | min_dist_to_camera | 30     | 到camera最小距离，判断是否在roi内  |
| float    | roi_h2bottom_scale | 0.5    | 依据内参计算目标高度调整数值时增益 |

#### camera tracking 模块

组件配置文件：/apollo/modules/perception/camera_tracking/conf/camera_tracking_config.pb.txt

| 参数类型 | 参数名                  | 默认值                            | 含义                         |
| -------- | ----------------------- | --------------------------------- | ---------------------------- |
| string   | output_channel_name     | /perception/inner/PrefusedObjects | camera tracking 模块输出通道 |
| string   | PluginParam.name        | OMTObstacleTracker                | 跟踪算法名称                 |
| string   | PluginParam.config_path | perception/camera_tracking/data   | 跟踪算法配置文件路径         |
| string   | PluginParam.config_file | omt.pb.txt                        | 跟踪算法配置文件名称         |
| int32    | image_width             | 1920                              | 图像宽度                     |
| int32    | image_height            | 1080                              | 图像高度                     |
| int32    | gpu_id                  | 0                                 | gpu id                       |

omt tracker算法配置文件： /apollo/modules/perception/camera_tracking/data/omt.pb.txt

| 参数类型    | 参数名                        | 默认值                          | 含义                                 |
| ----------- | ----------------------------- | ------------------------------- | ------------------------------------ |
| int32       | img_capability                | 14                              | 缓存的图像数量                       |
| int32       | reserve_age                   | 5                               | 目标丢失次数阈值                     |
| float       | weight_same_camera.appearance | 0.45                            | sa不为0，计算分数appearance权重      |
| float       | weight_same_camera.motion     | 0.4                             | sa不为0，计算分数motion权重          |
| float       | weight_same_camera.shape      | 0.15                            | sa不为0，计算分数shape权重           |
| float       | weight_same_camera.overlap    | 0.05                            | sa不为0，计算分数overlap权重         |
| float       | weight_diff_camera.motion     | 0.5                             | sa为0，计算分数motion权重            |
| float       | weight_diff_camera.shape      | 0.15                            | sa为0，计算分数shape权重             |
| float       | weight_diff_camera.overlap    | 0.35                            | sa为0，计算分数overlap权重           |
| string      | type_change_cost_file         | type_change_cost                | cost文件名                           |
| float       | border                        | 20                              | 图像边界阈值                         |
| float       | target_thresh                 | 0.6                             | 目标分数阈值，如果低于该分数，则过滤 |
| float       | abnormal_movement             | 0.4                             | 不正常移动阈值，用于判断是否误匹配   |
| float       | min_init_height_ratio         | 17                              | 目标高度阈值，用于判断是否创建新目标 |
| float       | target_combine_iou_threshold  | 0.6                             | iou阈值，用于判断是否合并两个目标    |
| TargetParam | target_param                  |                                 | 目标初始化参数                       |
| string      | plugin_param.name             | TrackingFeatureExtractor        | 特征提取器                           |
| string      | plugin_param.config_path      | perception/camera_tracking/data | 特征提取器配置文件路径               |
| string      | plugin_param.config_file      | tracking_feature.pb.txt         | 特征提取起配置文件名称               |
| int32       | feature_input_width           | 960                             | 特征提取器输入特征宽度               |
| int32       | feature_input_height          | 640                             | 特征提取器输入特征高度               |

特征提取算法配置文件： /apollo/modules/perception/camera_tracking/data/tracking_feature.pb.txt

| 参数类型    | 参数名                                | 默认值       | 含义                        |
| ----------- | ------------------------------------- | ------------ | --------------------------- |
| string      | feat_blob_name                        | conv4_3      | 特征层                      |
| int32       | feat_blob_shape                       | 1/64/160/240 | 特征尺寸                    |
| FeatureType | extractor.feat_type                   | ROIPooling   | 特征类型                    |
| int32       | extractor.roi_pooling_param.pooled_h  | 3            | ROIPooling高度              |
| int32       | extractor.roi_pooling_param.pooled_w  | 3            | ROIPooling宽度              |
| bool        | extractor.roi_pooling_param.use_floor | true         | 在ROIPooling中使用floor取整 |

### 支持的模型

| 模型             | 类型             | 模型下载地址                                                                 |
| ---------------- | ---------------- | ---------------------------------------------------------------------------- |
| PETR_V1_paddle   | BEV_detection    | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/petrv1.zip           |
| yolox3d_onnx     | camera_detection | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/yolox3d_onnx.zip     |
| caddn_paddle     | camera_detection | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/caddn_paddle.zip     |
| 3d-r4-half_caffe | camera_detection | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/3d-r4-half_caffe.zip |
| smoke_torch      | camera_detection | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/smoke_torch.zip      |

### 包列表

| 分类       | 包名                                                                                                   |
| ---------- | ------------------------------------------------------------------------------------------------------ |
| perception | [perception-camera-detection-bev](modules/perception/camera_detection_bev/README.md)                   |
| perception | [perception-camera-detection-multi-stage](modules/perception/camera_detection_multi_stage/README.md)   |
| perception | [perception-camera-detection-single-stage](modules/perception/camera_detection_single_stage/README.md) |
| perception | [perception-camera-location-estimation](modules/perception/camera_location_estimation/README.md)       |
| perception | [perception-camera-location-refinement](modules/perception/camera_location_refinement/README.md))      |
| perception | [perception-camera-tracking](modules/perception/camera_tracking/README.md)                             |

## 红绿灯模块概述

红绿灯模块主要分为四个部分：预处理、检测、识别和追踪。预处理阶段用于查询定位和地图信号灯信息，选择检测红绿灯的摄像头根据信号光在像平面上的投影，保存相机选择结果输出给检测阶段使用。检测阶段是传统的卷积神经网络检测任务，接收带有ROI的图像信息作为输入数据并顺序输出边界框。识别阶段根据输出的边界框识别交通灯的颜色。 这个任务是使用传统的卷积神经网络完成。 识别模块的输入是带有ROI的图像信息和一组边界框信息作为输入数据，输出是一个四维向量。分别表示每个边界框为黑、红、黄、绿的概率，当且仅当概率足够大时，概率最高的类别将被识别为信号灯的状态。追踪阶段是对红绿灯的检测结果进行一定的修正。

### Channel+消息格式

| 模块                          | 输入通道                                                                           | 输出通道                         | 消息格式                                          |
| ----------------------------- | ---------------------------------------------------------------------------------- | -------------------------------- | ------------------------------------------------- |
| traffic_light_region_proposal | /apollo/sensor/camera/front_6mm/image <br/> /apollo/sensor/camera/front_12mm/image | /perception/inner/Detection      | apollo::perception::onboard::TrafficDetectMessage |
| traffic_light_detection       | /perception/inner/Detection                                                        | /perception/inner/Retection      | apollo::perception::onboard::TrafficDetectMessage |
| traffic_light_recognition     | /perception/inner/Retection                                                        | /perception/inner/Tracking       | apollo::perception::onboard::TrafficDetectMessage |
| traffic_light_tracking        | /perception/inner/Tracking                                                         | /apollo/perception/traffic_light | apollo::perception::TrafficLightDetection         |

### 参数

#### traffic_light_region_proposal 模块

配置文件位置： modules/perception/traffic_light_region_proposal/conf

组件基本配置： modules/perception/traffic_light_region_proposal/proto/trafficlights_proposal_component.proto

| 参数类型 | 参数名                            | 默认值                                                                             | 含义                           |
| -------- | --------------------------------- | ---------------------------------------------------------------------------------- | ------------------------------ |
| string   | camera_names                      | front_6mm,front_12mm                                                               | 检测使用相机名称               |
| string   | channel.input_camera_channel_name | /apollo/sensor/camera/front_6mm/image <br/> /apollo/sensor/camera/front_12mm/image | 输入消息通道                   |
| double   | tl_image_timestamp_offset         | default = 0.0                                                                      | 红绿灯预处理图像时间偏移       |
| string   | PluginParam.name                  | TLPreprocessor                                                                     | 预处理算法插件名称             |
| string   | proposal_output_channel_name      | /perception/inner/Detection                                                        | 预处理模块处理消息输出通道名称 |
| string   | max_process_image_fps             | 8                                                                                  | 最大处理图像fps参数            |
| double   | query_tf_interval_seconds         | 0.3                                                                                | 查询tf间隔时长                 |
| double   | valid_hdmap_interval              | 1.5                                                                                | 未能获取信号信息的合法间隔时长 |
| int32    | gpu_id                            | 0                                                                                  | gpu id                         |

#### traffic_light_detection 模块

组件配置文件： modules/perception/traffic_light_detection/conf/traffic_light_detection_config.pb.txt

| 参数类型 | 参数名                        | 默认值                                  | 含义             |
| -------- | ----------------------------- | --------------------------------------- | ---------------- |
| string   | PluginParam.name              | TrafficLightDetection                   | 检测算法名称     |
| string   | PluginParam.config_path       | perception/traffic_light_detection/data | 配置文件路径     |
| string   | PluginParam.config_file       | detection_caffe.pb.txt                  | 配置文件名称     |
| string   | detection_output_channel_name | /perception/inner/Retection             | 检测结果输出通道 |
| int32    | gpu_id                        | 0                                       | gpu id           |

检测模型配置： modules/perception/traffic_light_detection/detection/proto/model_param.proto

模型配置文件位置：modules/perception/traffic_light_detection/data/detection_caffe.pb.txt

| 参数类型 | 参数名                | 默认值                       | 含义                          |
| -------- | --------------------- | ---------------------------- | ----------------------------- |
| string   | name                  | tl_detection_caffe           | 模型名称，同models/下文件夹名 |
| string   | framework             | TensorRT                     | 模型推理框架                  |
| string   | ModelFile.proto_file  | deploy.prototxt              | 模型网络结构                  |
| string   | ModelFile.weight_file | baidu_iter_140000.caffemodel | 模型权重文件                  |
| string   | ModelBlob.inputs      | img、im_info                 | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs     | bboxes                       | 模型输出数据名称及维度        |

#### traffic_light_recognition 模块

此模块配置文件位置：

- modules/perception/traffic_light_recognition/conf

- modules/perception/traffic_light_recognition/data

组件配置文件： modules/perception/traffic_light_recognition/proto/traffic_light_recognition_component.proto

| 参数类型 | 参数名                          | 默认值                                    | 含义             |
| -------- | ------------------------------- | ----------------------------------------- | ---------------- |
| string   | PluginParam.name                | TrafficLightRecognition                   | 检测算法名称     |
| string   | PluginParam.config_path         | perception/traffic_light_recognition/data | 配置文件路径     |
| string   | PluginParam.config_file         | recognition.pb.txt                        | 配置文件名称     |
| string   | recognition_output_channel_name | /perception/inner/Tracking                | 检测结果输出通道 |
| int32    | gpu_id                          | 0                                         | gpu id           |

识别模型配置： modules/perception/traffic_light_recognition/recognition/proto/model_param.proto

vertical_model ClassifyParam：

| 参数类型 | 参数名                 | 默认值                       | 含义                          |
| -------- | ---------------------- | ---------------------------- | ----------------------------- |
| string   | name                   | vertical_caffe               | 模型名称，同models/下文件夹名 |
| string   | framework              | TensorRT                     | 模型推理框架                  |
| string   | ModelFile.proto_file   | deploy.prototxt              | 模型网络结构                  |
| string   | ModelFile.weight_file  | baidu_iter_250000.caffemodel | 模型权重文件                  |
| string   | ModelBlob.inputs       | data_orgshap：[1,96,32,3]    | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs      | prob                         | 模型输出数据名称及维度        |
| float    | classify_threshold     | 0.5                          | 分类模型阈值                  |
| int32    | classify_resize_width  | 32                           | 分类调整宽度                  |
| int32    | classify_resize_height | 96                           | 分类调整高度                  |
| float    | scale                  | 0.01                         | 规模参数                      |
| float    | mean_b                 | 69.06                        | 平均b值                       |
| float    | mean_g                 | 66.58                        | 平均g值                       |
| float    | mean_r                 | 66.56                        | 平均r值                       |
| bool     | is_bgr                 | true                         | 判断是BGR还是RGB              |

quadrate_model ClassifyParam：

| 参数类型 | 参数名                 | 默认值                       | 含义                          |
| -------- | ---------------------- | ---------------------------- | ----------------------------- |
| string   | name                   | quadrate_caffe               | 模型名称，同models/下文件夹名 |
| string   | framework              | TensorRT                     | 模型推理框架                  |
| string   | ModelFile.proto_file   | deploy.prototxt              | 模型网络结构                  |
| string   | ModelFile.weight_file  | baidu_iter_200000.caffemodel | 模型权重文件                  |
| string   | ModelBlob.inputs       | data_org shap：[1,64,64,3]   | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs      | prob                         | 模型输出数据名称及维度        |
| float    | classify_threshold     | 0.5                          | 分类模型阈值                  |
| int32    | classify_resize_width  | 64                           | 分类调整宽度                  |
| int32    | classify_resize_height | 64                           | 分类调整高度                  |
| float    | scale                  | 0.01                         | 规模参数                      |
| float    | mean_b                 | 69.06                        | 平均b值                       |
| float    | mean_g                 | 66.58                        | 平均g值                       |
| float    | mean_r                 | 66.56                        | 平均r值                       |
| bool     | is_bgr                 | true                         | 判断是BGR还是RGB              |

horizontal_model ClassifyParam：

| 参数类型 | 参数名                 | 默认值                       | 含义                          |
| -------- | ---------------------- | ---------------------------- | ----------------------------- |
| string   | name                   | horizontal_caffe             | 模型名称，同models/下文件夹名 |
| string   | framework              | TensorRT                     | 模型推理框架                  |
| string   | ModelFile.proto_file   | deploy.prototxt              | 模型网络结构                  |
| string   | ModelFile.weight_file  | baidu_iter_200000.caffemodel | 模型权重文件                  |
| string   | ModelBlob.inputs       | data_org shap：[1,32,96,3]   | 模型输入数据名称及维度        |
| int32    | ModelBlob.outputs      | prob                         | 模型输出数据名称及维度        |
| float    | classify_threshold     | 0.5                          | 分类模型阈值                  |
| int32    | classify_resize_width  | 96                           | 分类调整宽度                  |
| int32    | classify_resize_height | 32                           | 分类调整高度                  |
| float    | scale                  | 0.01                         | 规模参数                      |
| float    | mean_b                 | 69.06                        | 平均b值                       |
| float    | mean_g                 | 66.58                        | 平均g值                       |
| float    | mean_r                 | 66.56                        | 平均r值                       |
| bool     | is_bgr                 | true                         | 判断是BGR还是RGB              |

#### traffic_light_tracking 模块

此模块配置文件位置：

- modules/perception/traffic_light_tracking/conf

- modules/perception/traffic_light_tracking/data

组件配置文件： modules/perception/traffic_light_tracking/proto/traffic_light_tracking_component.proto

| 参数类型 | 参数名                               | 默认值                                 | 含义                  |
| -------- | ------------------------------------ | -------------------------------------- | --------------------- |
| string   | PluginParam.name                     | SemanticReviser                        | 检测算法名称          |
| string   | PluginParam.config_path              | perception/traffic_light_tracking/data | 配置文件路径          |
| string   | PluginParam.config_file              | semantic.pb.txt                        | 配置文件名称          |
| string   | traffic_light_output_channel_name    | /apollo/perception/traffic_light       | 检测结果输出通道      |
| string   | v2x_trafficlights_input_channel_name | /apollo/v2x/traffic_light              | v2x红绿灯信号输入通道 |

tracking配置文件： modules/perception/traffic_light_tracking/tracker/proto/semantic.proto

参数类型 参数名 默认值 含义
float revise_time_second 1.5 修改时间阈值
float blink_threshold_second 0.55 红绿灯闪烁阈值时间
int hysteretic_threshold_count 1 延迟阈值时间计数统计

### 支持的模型

| 模型               | 类型                     | 模型下载地址                                                                   |
| ------------------ | ------------------------ | ------------------------------------------------------------------------------ |
| horizontal_torch   | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/horizontal_torch.zip   |
| quadrate_torch     | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/quadrate_torch.zip     |
| vertical_torch     | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/vertical_torch.zip     |
| horizontal_caffe   | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/horizontal_caffe.zip   |
| quadrate_caffe     | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/quadrate_caffe.zip     |
| vertical_caffe     | Trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/vertical_caffe.zip     |
| tl_detection_caffe | Trafficlight_detection   | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/tl_detection_caffe.zip |

### 包列表

| 分类       | 包名                                                                                                   |
| ---------- | ------------------------------------------------------------------------------------------------------ |
| perception | [perception-traffic-light-region-proposal](modules/perception/traffic_light_region_proposal/README.md) |
| perception | [perception-traffic-light-detection](modules/perception/traffic_light_detection/README.md)             |
| perception | [perception-traffic-light-recognition](modules/perception/traffic_light_recognition/README.md)         |
| perception | [perception-traffic-light-tracking](modules/perception/traffic_light_tracking/README.md)               |

## 毫米波模块概述

毫米波模块主要负责读取毫米波雷达的驱动输出信息，对毫米波硬件输出的目标级数据进行处理，包括目标跟踪、ROI过滤等，输出跟踪后的目标级障碍物，可以单独输出也可以输入到多传感器融合模块。

### Channel+消息格式

| 模块            | 输入通道                   | 输出通道                     | 消息格式                                |
| --------------- | -------------------------- | ---------------------------- | --------------------------------------- |
| radar_detection | /apollo/sensor/radar/front | /apollo/perception/obstacles | apollo::perception::PerceptionObstacles |

参数

#### radar_detection 模块

此模块配置文件位置：

- modules/perception/radar_detection/conf

- modules/perception/radar_detection/data

算法基本配置： modules/perception/radar_detection/proto

| 参数类型 | 参数名                         | 默认值                            | 含义                   |
| -------- | ------------------------------ | --------------------------------- | ---------------------- |
| double   | max_match_distance             | 2.5                               | 匹配最大距离阈值       |
| double   | bound_match_distance           | 10.0                              | 匹配边界阈值           |
| double   | delay_time                     | 0.07                              | 用于纠正目标的时间戳   |
| string   | radar_name                     | radar_front                       | 毫米波雷达名称         |
| string   | tf_child_frame_id              | radar_front                       | 毫米波雷达名称         |
| double   | radar_forward_distance         | 200.0                             | 毫米波雷达前向探测距离 |
| string   | preprocessor_param.name        | ContiArsPreprocessor              | 预处理插件名           |
| string   | preprocessor_param.config_path | perception/radar_detection/data   | 预处理插件配置路径     |
| string   | preprocessor_param.config_file | preprocessor.pb.txt               | 预处理插件配置文件名   |
| string   | perception_param.name          | RadarObstaclePerception           | 感知插件名             |
| string   | perception_param.config_path   | perception/radar_detection/data   | 感知插件配置路径       |
| string   | perception_param.config_path   | radar_obstacle_perception.pb.txt  | 感知插件配置文件名     |
| string   | odometry_channel_name          | /apollo/localization/pose         | 定位通道名             |
| string   | output_channel_name            | /perception/inner/PrefusedObjects | 输出通道名             |
| string   | detector_param.name            | ContiArsDetector                  | 目标检测插件名         |
| string   | roi_filter_param.name          | HdmapRadarRoiFilter               | ROI过滤插件名          |
| string   | tracker_param.name             | ContiArsTracker                   | 跟踪插件名             |
| string   | tracker_param.config_path      | perception/radar_detection/data   | 跟踪插件配置路径       |
| string   | tracker_param.config_file      | conti_ars_tracker.pb.txt          | 跟踪插件配置文件名     |
| double   | tracking_time_window           | 0.06                              | 跟踪时间窗口           |
| string   | matcher_param.name             | HMMatcher                         | 匹配器插件名           |
| string   | matcher_param.config_path      | perception/radar_detection/data   | 匹配器插件配置路径     |
| string   | matcher_param.config_file      | hm_matcher.pb.txt                 | 匹配器插件配置文件名   |
| string   | chosen_filter                  | AdaptiveKalmanFilter              | 过滤器名               |
| double   | tracked_times_threshold        | 3                                 | 跟踪时间阈值           |
| bool     | use_filter                     | false                             | 是否使用过滤器         |

### 包列表

| 分类       | 包名                                                                       |
| ---------- | -------------------------------------------------------------------------- |
| perception | [perception-radar-detection](modules/perception/radar_detection/README.md) |

## 4D毫米波模块概述

4D毫米波模块主要负责读取4D毫米波雷达的驱动输出信息，即4D毫米波雷达输出的点云数据。4D毫米波模块会对毫米波点云进行点云预处理，使用经典的点云3D目标检测模型PointPillars对目标进行检测，并通过ROI过滤和目标跟踪算法输出跟踪后的目标级障碍物，可以单独输出，也可以输入到多传感器融合模块。

### Channel+消息格式

| 模块              | 输入通道                          | 输出通道                     | 消息格式                                |
| ----------------- | --------------------------------- | ---------------------------- | --------------------------------------- |
| radar4d_detection | /apollo/sensor/oculii/PointCloud2 | /apollo/perception/obstacles | apollo::perception::PerceptionObstacles |

### 参数

#### radar4d_detection 模块

此模块配置文件位置：

- modules/perception/radar4d_detection/conf

- modules/perception/radar4d_detection/data

算法基本配置： modules/perception/radar4d_detection/proto

| 参数类型 | 参数名                                 | 默认值                                             | 含义                       |
| -------- | -------------------------------------- | -------------------------------------------------- | -------------------------- |
| double   | rcs_offset                             | -35                                                | 毫米波雷达反射强度的偏置   |
| string   | detector_param.name                    | Radar4dDetection                                   | 目标检测器插件名           |
| string   | detector_param.config_path             | perception/radar4d_detection/data                  | 目标检测器配置路径         |
| string   | detector_param.config_file             | point_pillars_param.pb.txt                         | 目标检测器配置文件名       |
| string   | roi_filter_param.name                  | HdmapRadarRoiFilter                                | ROI过滤器插件名            |
| string   | multi_target_tracker_param.name        | MrfEngine                                          | 多目标跟踪器插件名         |
| string   | multi_target_tracker_param.config_path | perception/radar4d_detection/data/tracking         | 多目标跟踪器配置路径       |
| string   | multi_target_tracker_param.config_file | mrf_engine.pb.txt                                  | 多目标跟踪器配置文件名     |
| string   | fusion_classifier_param.name           | FusedClassifier                                    | 目标分类器插件名           |
| string   | fusion_classifier_param.config_path    | perception/radar4d_detection/data/fused_classifier | 目标分类器配置路径         |
| string   | fusion_classifier_param.config_file    | fused_classifier.pb.txt                            | 目标分类器配置文件名       |
| bool     | enable_roi_filter                      | true                                               | 是否使用roi过滤器          |
| string   | radar_name                             | radar_front                                        | 4D毫米波名称               |
| string   | tf_child_frame_id                      | radar_front                                        | 4D毫米波名称               |
| double   | radar_forward_distance                 | 200.0                                              | 毫米波最大探测距离         |
| string   | preprocessor_param.name                | RadarPreprocessor                                  | 点云预处理插件名           |
| string   | preprocessor_param.config_path         | perception/radar4d_detection/data                  | 点云预处理配置路径         |
| string   | preprocessor_param.config_file         | preprocessor_config.pb.txt                         | 点云预处理配置文件名       |
| string   | perception_param.name                  | RadarObstaclePerception                            | 目标检测模块插件名         |
| string   | perception_param.config_path           | perception/radar4d_detection/data                  | 目标检测模块配置文件路径   |
| string   | perception_param.config_file           | radar_obstacle_perception_config.pb.txt            | 目标检测模块配置文件名     |
| string   | odometry_channel_name                  | /apollo/localization/pose                          | 定位模块通道名             |
| string   | output_channel_name                    | /perception/inner/PrefusedObjects                  | 4D毫米波模块输出通道名     |
| string   | info.weight_file.file                  | radar_libtorch_all.zip                             | 目标检测网络参数文件名     |
| float    | postprocess.score_threshold            | 0.1                                                | 置信度阈值                 |
| float    | postprocess.nms_overlap_threshold      | 0.01                                               | nms阈值                    |
| int32    | postprocess.num_output_box_feature     | 7                                                  | 网络输出的特征数量         |
| int32    | preprocess.num_point_feature           | 7                                                  | 网络输入的点特征数量       |
| bool     | preprocess.enable_fuse_frames          | true                                               | 是否使用多帧融合策略       |
| int32    | preprocess.num_fuse_frames             | 5                                                  | 融合帧数量                 |
| double   | preprocess.fuse_time_interval          | 0.5                                                | 融合帧最大时间差           |
| bool     | use_histogram_for_match                | false                                              | 是否使用直方图特征进行匹配 |
| bool     | output_predict_objects                 | false                                              | 是否输出预测的目标         |
| double   | reserved_invisible_time                | 2                                                  | 最大不可见时间             |

### 包列表

| 分类       | 包名                                                                           |
| ---------- | ------------------------------------------------------------------------------ |
| perception | [perception-radar4d-detection](modules/perception/radar4d_detection/README.md) |

## lidar模块概述

激光雷达检测用于 3D 目标检测，它的输入是激光雷达点云，输出为检测到的物体的类型和坐标。

### Channel+消息格式

| 模块                        | 输入通道                                          | 输出通道                                      | 消息格式                                                                                        |
| --------------------------- | ------------------------------------------------- | --------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| lidar_detection             | perception/lidar/pointcloud_ground_detection      | /perception/lidar/detection                   | apollo::perception::onboard::LidarFrameMessage                                                  |
| lidar_detection_filter      | /perception/lidar/detection                       | /perception/lidar/detection_filter            | apollo::perception::onboard::LidarFrameMessage                                                  |
| lidar_tracking              | /perception/lidar/detection_filter                | /perception/inner/PrefusedObjects             | apollo::perception::onboard::LidarFrameMessage, apollo::perception::onboard::SensorFrameMessage |
| pointcloud_ground_detection | /perception/lidar/pointcloud_map_based_roi        | /perception/lidar/pointcloud_ground_detection | apollo::perception::onboard::LidarFrameMessage                                                  |
| pointcloud_map_based_roi    | /perception/lidar/pointcloud_preprocess           | /perception/lidar/pointcloud_map_based_roi    | apollo::perception::onboard::LidarFrameMessage                                                  |
| pointcloud_preprocess       | /apollo/sensor/velodyne64/compensator/PointCloud2 | /perception/lidar/pointcloud_preprocess       | apollo::drivers::PointCloud, apollo::perception::onboard::LidarFrameMessage                     |

### 参数

#### lidar_detection模块

此模块配置文件路径：

- modules/perception/lidar_detection/conf
- modules/perception/lidar_detection/data

ModelInfo配置，文件路径：modules/perception/common/proto/model_info.proto

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

cnnseg配置

modules/perception/lidar_detection/detector/cnn_segmentation/proto/model_param.proto

| 参数类型             | 参数名                     | 默认值            | 含义                       |          |
| -------------------- | -------------------------- | ----------------- | -------------------------- | -------- |
| ModelInfo            | info                       | /                 | 模型基本配置               |          |
| FeatureParam         | float                      | point_cloud_range | 90                         | 点云范围 |
| uint32               | width                      | 864               | BEV宽度                    |          |
| uint32               | height                     | 864               | BEV高度                    |          |
| float                | min_height                 | -5.0              | 点云z值最小值              |          |
| float                | max_height                 | 5.0               | 点云z值最大值              |          |
| bool                 | use_intensity_feature      | true              | 是否使用强度特征           |          |
| bool                 | use_constant_feature       | false             | 是否使用常数特征           |          |
| bool                 | do_classification          | true              | 是否预测分类信息           |          |
| bool                 | do_heading                 | true              | 是否预测朝向角信息         |          |
| PointCloudPreProcess | preprocess                 | /                 | 预处理信息                 |          |
| SppEngineConfig      | float                      | height_gap        | 0.5                        | 高度差距 |
| bool                 | remove_ground_points       | true              | 是否过滤掉障碍物中的地面点 |          |
| float                | objectness_thresh          | 0.5               | objectness的阈值           |          |
| float                | confidence_thresh          | 0.1               | confidence的阈值           |          |
| float                | height_thresh              | 0.5               | 高度阈值                   |          |
| uint32               | min_pts_num                | 3                 | 目标最少点数               |          |
| float                | confidence_range           | 85                | 置信度范围                 |          |
| bool                 | fill_recall_with_segmentor | true              | 是否使用背景分割           |          |

centerpoint配置：

modules/perception/lidar_detection/data/center_point_param.pb.txt

| 参数类型              | 参数名            | 默认值 | 含义                        |
| --------------------- | ----------------- | ------ | --------------------------- |
| ModelInfo             | info              | /      | 模型通用配置                |
| PointCloudPreProcess  | preprocess        | /      | 预处理                      |
| PointCloudPostProcess | postprocess       | /      | 后处理                      |
| int32                 | point2box_max_num | 5      | 每个点最多可以属于多少个box |
| float                 | quantize          | 0.2    | 将尺寸量化为quantize的倍数  |

### 包列表

| 分类       | 包名                                                                       |
| ---------- | -------------------------------------------------------------------------- |
| perception | [perception-lidar-detection](modules/perception/lidar_detection/README.md) |

#### lidar_detection_filter模块

background_filter配置：

modules/perception/lidar_detection_filter/data/background_filter.pb.txt

| 参数类型 | 参数名                      | 默认值 | 含义                   |
| -------- | --------------------------- | ------ | ---------------------- |
| float    | outside_roi_filter_distance | 1.0    | 在车道线之外的距离阈值 |

roi_boundary_filter配置：

modules/perception/lidar_detection_filter/data/roi_boundary_filter.pb.txt

| 参数类型 | 参数名                         | 默认值 | 含义                                                         |
| -------- | ------------------------------ | ------ | ------------------------------------------------------------ |
| float    | distance_to_boundary_threshold | 2.5    | 障碍物在车道线外时，距离车道线边界的阈值，大于该阈值则被删除 |
| float    | confidence_threshold           | 0.0    | 目标置信度阈值                                               |
| float    | cross_roi_threshold            | 0.6    | 目标是否在roi交界处的阈值                                    |
| float    | inside_threshold               | -1.0   | 障碍物在车道线内时，距离车道线边界的阈值，小于该阈值则被删除 |

### 包列表

| 分类       | 包名                                                                                     |
| ---------- | ---------------------------------------------------------------------------------------- |
| perception | [perception-lidar-detection-filter](modules/perception/lidar_detection_filter/README.md) |

#### pointcloud_map_base_roi模块

hdmap_roi_filter配置：

modules/perception/pointcloud_map_based_roi/data/hdmap_roi_filter.pb.txt

| 参数类型 | 参数名          | 默认值 | 含义                                       |
| -------- | --------------- | ------ | ------------------------------------------ |
| float    | range           | 120.0  | 基于LiDAR传感器点的2D网格ROI LUT的图层范围 |
| float    | cell_size       | 0.25   | 用于量化2D网格的单元格的大小。             |
| float    | extend_dist     | 2.5    | 从多边形边界扩展ROI的距离。                |
| bool     | no_edge_table   | false  | 是否有edge table                           |
| bool     | set_roi_service | true   | 是否设置ROI Service                        |

map_manager配置：

modules/perception/pointcloud_map_based_roi/data/map_manager.pb.txt

| 参数类型 | 参数名              | 默认值 | 含义                            |
| -------- | ------------------- | ------ | ------------------------------- |
| bool     | update_pose         | false  | 是否更新lidar到世界坐标系的位姿 |
| float    | roi_search_distance | 120.0  | ROI搜索的范围                   |

### 包列表

| 分类       | 包名                                                                                         |
| ---------- | -------------------------------------------------------------------------------------------- |
| perception | [perception-pointcloud-map-based-roi](modules/perception/pointcloud_map_based_roi/README.md) |

#### pointcloud_preprocess模块

pointcloud_preprocessor配置：

modules/perception/pointcloud_preprocess/data/pointcloud_preprocessor.pb.txt

| 参数类型 | 参数名                   | 默认值 | 含义                    |
| -------- | ------------------------ | ------ | ----------------------- |
| bool     | filter_naninf_points     | true   | 是否过滤nan点           |
| bool     | filter_nearby_box_points | false  | 是否过滤nearby box的点  |
| float    | box_forward_x            | 0      | 主车imu到右侧宽度       |
| float    | box_backward_x           | 0      | 主车imu到左侧宽度       |
| float    | box_forward_y            | 0      | 主车imu到前方距离度     |
| float    | box_backward_y           | 0      | 主车imu到后方距离       |
| bool     | filter_high_z_points     | false  | 是否过滤高度方向的z坐标 |
| float    | z_threshold              | 5      | 高度过滤阈值            |

### 包列表

| 分类       | 包名                                                                                   |
| ---------- | -------------------------------------------------------------------------------------- |
| perception | [perception-pointcloud-preprocess](modules/perception/pointcloud_preprocess/README.md) |

#### pointcloud_ground_detection模块

spatio_temporal_ground_detector配置文件：

modules/perception/pointcloud_ground_detection/data/spatio_temporal_ground_detector.pb.txt

| 参数类型 | 参数名             | 默认值 | 含义                 |
| -------- | ------------------ | ------ | -------------------- |
| uint32   | grid_size          | 16     | 网格size             |
| float    | ground_thres       | 0.25   | 网格阈值             |
| float    | roi_rad_x          | 120.0  | x方向roi radius      |
| float    | roi_rad_y          | 120.0  | y方向roi radius      |
| float    | roi_rad_z          | 100.0  | z反向roi radius      |
| uint32   | nr_smooth_iter     | 5      | smooth迭代次数       |
| bool     | use_roi            | true   | 是否用roi内的点云    |
| bool     | use_ground_service | true   | 是否用ground service |

ground_service_detector配置文件：

modules/perception/pointcloud_ground_detection/data/ground_service_detector.pb.txt

| 参数类型 | 参数名           | 默认值 | 含义         |
| -------- | ---------------- | ------ | ------------ |
| double   | ground_threshold | 0.25   | 地面分割阈值 |

### 包列表

| 分类       | 包名                                                                                               |
| ---------- | -------------------------------------------------------------------------------------------------- |
| perception | [perception-pointcloud-ground-detection](modules/perception/pointcloud_ground_detection/README.md) |

#### lidar_tracking模块

##### fused_classifer

fused_classifier配置文件：

modules/perception/lidar_tracking/data/fused_classifier/fused_classifier.pb.txt

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

##### tracker

mlf_engine配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_engine.pb.txt

| 参数类型 | 参数名                  | 默认值 | 含义                      |
| -------- | ----------------------- | ------ | ------------------------- |
| string   | main_sensor             | /      | 主传感器                  |
| bool     | use_histogram_for_match | true   | 是否用histogram用来做匹配 |
| uint32   | histogram_bin_size      | 10     | histogram的大小           |
| bool     | output_predict_objects  | false  | 是否输出预测目标          |
| double   | reserved_invisible_time | 0.2    | 保留不可见时间            |
| bool     | use_frame_timestamp     | false  | 是否用frame时间戳         |

mlf_track_object_matcher配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_track_object_matcher.conf

| 参数类型 | 参数名                    | 默认值                       | 含义             |
| -------- | ------------------------- | ---------------------------- | ---------------- |
| string   | foreground_mathcer_method | MultiHmBipartiteGraphMatcher | 前景目标匹配方法 |
| string   | background_matcher_method | GnnBipartiteGraphMatcher     | 背景目标匹配方法 |
| float    | bound_value               | 100.0                        | 边界值           |
| float    | max_match_distance        | 4.0                          | 最大匹配距离     |

mlf_track_object_distance配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_track_object_distance.conf

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

mlf_tracker配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_tracker.conf

| 参数类型 | 参数名      | 默认值 | 含义       |
| -------- | ----------- | ------ | ---------- |
| string   | filter_name | /      | filter名称 |

mlf_motion_filter配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_motion_filter.conf

| 参数类型 | 参数名                       | 默认值 | 含义                 |
| -------- | ---------------------------- | ------ | -------------------- |
| bool     | use_adaptive                 | true   | 是否用自适应方法     |
| bool     | use_breakdown                | true   | 是否使用breakdown    |
| bool     | use_convergence_boostup      | true   | 是否用收敛启动       |
| double   | init_velocity_variance       | 5.0    | 速度方差初始参数     |
| double   | init_acceleration_variance   | 10.0   | 加速度方差初始参数   |
| double   | measured_velocity_variance   | 0.4    | 速度方差测量参数     |
| double   | predict_variance_per_sqrsec  | 1.0    | 预测方差参数         |
| uint32   | boostup_history_size_minimum | 3      | 历史最小值初始化参数 |
| uint32   | boostup_history_size_maximum | 6      | 历史最大值初始化参数 |
| double   | converged_confidence_minimum | 0.5    | 最小收敛置信度参数   |
| double   | noise_maximum                | 0.1    | 最大噪声参数         |
| double   | trust_orientation_range      | 40     | 方向范围信任参数     |

mlf_motion_refiner配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_motion_refiner.conf

| 参数类型 | 参数名                         | 默认值 | 含义           |
| -------- | ------------------------------ | ------ | -------------- |
| double   | claping_acceleration_threshold | 10.0   | 加速度截断阈值 |
| double   | claping_speed_threshold        | 1.0    | 速度截断阈值   |

mlf_shape_filter配置文件：

modules/perception/lidar_tracking/data/tracking/mlf_shape_filter.conf

| 参数类型 | 参数名                         | 默认值 | 含义             |
| -------- | ------------------------------ | ------ | ---------------- |
| double   | bottom_points_ignore_threshold | 0.1    | 底部点云忽略阈值 |
| double   | top_points_ignore_threshold    | 1.6    | 顶部点云忽略阈值 |

### 包列表

| 分类       | 包名                                                                     |
| ---------- | ------------------------------------------------------------------------ |
| perception | [perception-lidar-tracking](modules/perception/lidar_tracking/README.md) |

## 融合模块概述

融合模块是Apollo自动驾驶系统的重要组成部分，本质为多源数据下的多目标跟踪问题，将车端搭载的多种传感器的障碍物检测跟踪结果作为观测，再次进行跟踪和融合，并将最终的障碍物检测跟踪结果输出。融合模块弥补了单一传感器的FOV缺陷，有效结合了不同传感器的优点特性，提供了更高精度和信息更为丰富的障碍物感知结果。
融合模块大体分为以下几步：

- 对输入的各传感器跟踪的障碍物信息进行统一的结构转换；

- 对转化后的各传感器跟踪的障碍物信息与历史track信息进行匹配

  - 对于匹配上的历史track，根据输入的观测，进行各属性（如类别、中心点、尺寸、存在概率等）的融合更新

  - 对于没有匹配上的历史track，单纯进行预测，将各属性预测至当前时间下

  - 对于没有匹配上的传感器障碍物信息，根据特定条件新建track，加入到历史track中，并初始化各属性信息

- 根据各属性信息，分别判断所有的历史track是否满足预设的门限条件，选择输出或不输出

- 根据时间信息，删除超过一定时间的历史track

### Channel + 消息格式

|          | 通道名称                                     | 消息格式                               | 备注                                  |
| -------- | -------------------------------------------- | -------------------------------------- | ------------------------------------- |
| 输入通道 | /perception/inner/PrefusedObjects            | 感知common中定义的SensorFrameMessage类 | 每个传感器通过消息里的sensor_name区分 |
| 输出通道 | /apollo/perception/obstacles                 | PerceptionObstacles                    | 对外发布                              |
| 输出通道 | /perception/inner/visualization/FusedObjects | 感知common中定义的SensorFrameMessage类 | 用于后续的可视化，暂时没用            |

### 参数介绍

融合模块算法配置文件为 multi_sensor_fusion/conf/multi_sensor_fusion_config.pb.txt ， 各参数含义如下

| 参数类型    | 参数名                                | 默认值                                       | 含义                                                      |
| ----------- | ------------------------------------- | -------------------------------------------- | --------------------------------------------------------- |
| PluginParam | fusion_param                          | /                                            | 融合模块使用的融合方法名称和配置路径                      |
| bool        | object_in_roi_check                   | true                                         | 是否进行障碍物在roi区域内的检查（实际未使用）             |
| double      | radius_for_roi_object_check           | 120                                          | 以主车为原点，读取高精地图ROI区域的搜索半径（实际未使用） |
| string      | output_obstacles_channel_name         | /apollo/perception/obstacles                 | 融合模块最终对外发布输出的通道名称                        |
| string      | output_viz_fused_content_channel_name | /perception/inner/visualization/FusedObjects | 融合模块用于后续可视化的通道名称（未使用）                |

根据 fusion_param 获取到的方法名称为 ProbabilisticFusion ， 配置文件 probablistic_fusion.pb.txt 各参数含义如下

| 参数类型    | 参数名                      | 默认值                       | 含义                                                  |
| ----------- | --------------------------- | ---------------------------- | ----------------------------------------------------- |
| bool        | use_lidar                   | true                         | 是否使用激光雷达检测跟踪结果                          |
| bool        | use_radar                   | true                         | 是否使用毫米波雷达检测跟踪结果                        |
| bool        | use_camera                  | true                         | 是否使用相机检测跟踪结果                              |
| PluginParam | track_param                 | /                            | 定义历史track的名称和配置文件路径                     |
| PluginParam | data_association_param      | HMTrackersObjectsAssociation | 使用的数据关联算法名称【没有相关配置文件】            |
| PluginParam | gatekeeper_param            | /                            | 定义门限条件判断的名称和配置文件路径                  |
| string      | prohibition_sensors         | radar_front                  | repeated字段，可指定多个；不允许该传感器新建历史track |
| double      | max_lidar_invisible_period  | 0.25                         | 超过特定时间没有更新就删除激光雷达观测                |
| double      | max_radar_invisible_period  | 0.50                         | 超过特定时间没有更新就删除毫米波雷达观测              |
| double      | max_camera_invisible_period | 0.75                         | 超过特定时间没有更新就删除相机观测                    |
| int         | max_cached_frame_num        | 50                           |

根据 track_param 获取到的方法名称为 PbfTrack ， 配置文件 pbf_tracker.pb.txt 各参数含义如下

| 参数类型    | 参数名                  | 默认值             | 含义                                              |
| ----------- | ----------------------- | ------------------ | ------------------------------------------------- |
| PluginParam | type_fusion_param       | /                  | 类别融合方法名称和配置文件路径                    |
| PluginParam | motion_fusion_param     | KalmanMotionFusion | 运动融合（含速度、加速度信息）方法名称            |
| PluginParam | shape_fusion_param      | PbfShapeFusion     | 形状融合（含尺寸、方向、polygon、中心点）方法名称 |
| PluginParam | existance_fusion_params | /                  | 存在概率融合方法名称和配置文件路径                |

根据 type_fusion_param 获取到的方法名称为 DstTypeFusion ， 配置文件 dst_type_fusion.pb.txt 各参数含义如下

| 参数类型                 | 参数名                                 | 默认值                              | 含义                                                          |
| ------------------------ | -------------------------------------- | ----------------------------------- | ------------------------------------------------------------- |
| CameraDstTypeFusionParam | camera_params::name                    | 两个相机名称为front_6mm和front_12mm | 相机的sensor_name                                             |
|                          | camera_params::valid_dist              | 6/12mm分别对应110m和150m            | lidar/radar向相机投影设置的最大距离，超过此值，距离得分会减小 |
|                          | camera_params::reliability             | 6/12mm分别对应0.95和0.5             | 历史track匹配上时，单帧检测概率分布的置信度权重               |
|                          | camera_params::reliability_for_unknown | 6/12mm分别对应0.2和0.2              | 未匹配上时，预测的单帧检测概率分布的置信度权重                |
| LidarDstTypeFusionParam  | lidar_params::name                     | 激光雷达名称为velodyne64            | 激光雷达的sensor_name                                         |
|                          | lidar_params::reliability              | 0.5                                 | 历史track匹配上时，单帧检测概率分布的置信度权重               |
|                          | lidar_params::reliability_for_unknown  | 0.5                                 | 未匹配上时，预测的单帧检测概率分布的置信度权重                |

根据 existance_fusion_params 获取到的方法名称为 DstExistanceFusion ， 配置文件 dst_existance_fusion.pb.txt 各参数含义如下

| 参数类型        | 参数名                          | 默认值                              | 含义                                                             |
| --------------- | ------------------------------- | ----------------------------------- | ---------------------------------------------------------------- |
| double          | track_object_max_match_distance | 4.0                                 | 在相机匹配到历史track时，根据匹配距离与该值的比例赋值DST属性概率 |
| CameraValidDist | camera_valid_dist::camera_name  | 两个相机名称为front_6mm和front_12mm | 相机的sensor_name                                                |
|                 | camera_valid_dist::valid_dist   | 6/12mm分别对应110m和150m            | lidar/radar向相机投影设置的最大距离，超过此值，距离得分会减小    |

根据 gatekeeper_param 获取到的方法名称为 PbfGatekeeper ， 配置文件 pbf_gatekeeper.pb.txt 各参数含义如下

| 参数类型 | 参数名                       | 默认值 | 含义                                                                    |
| -------- | ---------------------------- | ------ | ----------------------------------------------------------------------- |
| bool     | publish_if_has_lidar         | true   | 有激光雷达检测是否输出                                                  |
| bool     | publish_if_has_radar         | true   | 有毫米波雷达检测是否输出                                                |
| bool     | publish_if_has_camera        | true   | 有相机雷达检测是否输出                                                  |
| bool     | use_camera_3d                | true   | 是否使用相机的3d信息                                                    |
| double   | min_radar_confident_distance | 40     | 毫米波检测可信的最小检测距离                                            |
| double   | max_radar_confident_angle    | 20     | 毫米波检测可信的最大检测角度                                            |
| double   | min_camera_publish_distance  | 50     | 非锥桶条件下，相机的最小输出距离                                        |
| double   | invisible_period_threshold   | 0.001  | 【没有使用】                                                            |
| double   | toic_threshold               | 0.8    | 该track的toic置信度阈值                                                 |
| bool     | use_track_time_pub_strategy  | true   | 是否使用跟踪时间的输出策略                                              |
| int      | pub_track_time_thresh        | 3      | 跟踪帧数小于该值，不输出                                                |
| double   | existance_threshold          | 0.7    | 在锥桶或者相机超过最小输出距离的前提下，该track存在概率大于该值才可输出 |
| double   | radar_existance_threshold    | 0.9    | 后毫米波雷达，该track存在概率大于该值才可输出                           |

### 包列表

| 包名                                                                               |
| ---------------------------------------------------------------------------------- |
| [perception-multi-sensor-fusion](modules/perception/multi_sensor_fusion/README.md) |
