# perception-traffic-light-detection

## 介绍

经过预处理阶段的投影，在图片上得到了一个投影框，得到的投影框并不完全可靠，需要通过模型计算出信号灯的准确边界框。

## 目录结构

```
├── traffic_light_detection // 交通灯检测模块
    ├── conf            // 模块配置文件
    ├── dag             // dag文件
    ├── data            // 模型参数
    ├── detection       // 检测的主要部分
    ├── interface       // 函数接口文件
    ├── launch          // launch启动文件
    ├── proto           // proto文件
    ├── traffic_light_detection_component.cc // 组件入口
    ├── traffic_light_detection_component.h
    ├── cyberfile.xml   // 包管理配置文件
    ├── README.md
    ├── README_cn.md
    └── BUILD
```

## 模块

###TrafficLightDetectComponent

apollo::perception::onboard::TrafficLightDetectComponent

#### 输入

| 名称 | 类型 | 描述 | 输入通道 |
| ------- | --------------------------------------------------- | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | Trafficlight 消息 | /perception/inner/Detection |

>注意：输入通道为结构体类型数据，默认触发通道为 `/perception/inner/Detection`，详细的输入通道信息在 `modules/perception/traffic_light_detection/dag/traffic_light_detection.dag` 文件中，默认情况下组件接收的消息上游组件包括 `traffic_light_region_proposal`。

#### 输出

| 名称 | 类型 | 描述 | 输出通道 |
| ------- | --------------------------------------------------- | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | Trafficlight 消息 | /perception/inner/Retection |

>注意：输出通道为结构体类型数据。消息定义在`modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h`文件中，输出通道消息数据可由同一进程中的组件订阅，详细的输出通道信息在`modules/perception/traffic_light_detection/conf/traffic_light_detection_config.pb.txt`文件中。

#### 配置参数

组件配置文件： modules/perception/traffic_light_detection/conf/traffic_light_detection_yolox_config.pb.txt

| 参数类型 | 参数名称 | 默认值 | 含义 |
| -------- | ----------------------------------------- | --------------------------------------- | ---------------- |
| string | PluginParam.name | TrafficLightTLDetectorYolox | 检测算法名称 |
| string | PluginParam.config_path | perception/traffic_light_detection/data | 配置文件路径 |
| string | PluginParam.config_file | detection_yolox.pb.txt | 配置文件名称 |
| string | detection_output_channel_name | /perception/inner/Retection | 检测结果输出通道 |
| int32 | gpu_id | 0 | gpu id |

Yolox模型配置文件位置：modules/perception/traffic_light_detection/data/detection_yolox.pb.txt

| 参数类型 | 参数名称 | 默认值 | 含义 |
| -------- | --------------------- | ---------------------------- | ----------------------------- |
| string | name | tl_detection_yolox | 模型名称 |
| string | framework | Onnx -> TensorRT | 模型推理框架 |
| string | ModelFile.proto_file | yolox.onnx | 模型网络结构 |
| string | ModelBlob.inputs | images [1,3,384,384] | 模型输入数据名称及维度 |
| int32 | ModelBlob.outputs | bbox、conf、cls | 模型输出数据名称及维度 |
| int32 | max_batch_size | 3 | 多批次推理输入模型检测维度 |
| int32 | min_crop_size | 400 |裁剪框大小 |
| int32 | classify_resize_width | 384 | 模型输入图片调整宽度 |
| int32 | classify_resize_height | 384 | 模型输入图片调整高度 |

#### 如何启动

1. 在配置文件中的 modules/perception/data/params 中添加车辆参数配置文件，对应 frame_id 和 sensor_name，启动transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```
2. 启动trafficlight感知
```bash
cyber_launch start /apollo/modules/perception/launch/perception_trafficlight.launch