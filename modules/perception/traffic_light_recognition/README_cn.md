# perception-traffic-light-recognition

## 简介

交通信号灯识别模块的目的是识别交通信号灯的颜色。识别模块的输入是一幅带有ROI信息的图像和一组边界框信息作为输入数据，输出是一个四维向量，分别表示每个边界框为黑色、红色、黄色和绿色的概率，当且仅当概率足够大时，概率最大的类将被识别为信号灯的状态。否则信号灯状态设置为Unknown，表示状态未确定。

## 目录结构

```
├── traffic_light_recognition // 交通灯识别模块
    ├── conf            // 模块配置文件
    ├── dag             // dag文件
    ├── data            // 模型参数
    ├── detection       // 检测实现
    ├── interface       // 函数接口文件
    ├── launch          // launch启动文件
    ├── proto           // proto文件
    ├── traffic_light_recognition_component.cc // 组件入口
    ├── traffic_light_recognition_component.h
    ├── cyberfile.xml   // 包管理配置文件
    ├── README.md
    ├── README_cn.md
    └── BUILD
```

## 模块

###TrafficLightRecognComponent

apollo::perception::onboard::TrafficLightRecognComponent

#### 输入

| 名称 | 类型 | 描述 | 输入通道 |
| ------- | --------------------------------------------------- | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | Trafficlight 消息 | /perception/inner/Retection |

>注意：输入通道为结构体类型数据，默认触发通道为 `/perception/inner/Retection`，详细的输入通道信息在 `modules/perception/traffic_light_recognition/dag/traffic_light_recognition.dag` 文件中，默认情况下组件接收的消息上游组件包括 `traffic_light_detection`。

#### 输出

| 名称 | 类型 | 描述 | 输出通道 |
| ------- | --------------------------------------------------- | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | Trafficlight 消息 | /perception/inner/Tracking |

>注意：输出通道为结构体类型数据。消息定义在`modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h`文件中，输出通道消息数据可供同进程的组件订阅，详细的输出通道信息在`modules/perception/traffic_light_recognition/conf/traffic_light_recognition_config.pb.txt`文件中。

#### 配置参数

组件配置文件：modules/perception/traffic_light_recognition/conf/traffic_light_recognition_config.pb.txt

| 参数类型 | 参数名称 | 默认值 | 含义 |
| -------- | ----------------------------------------- | --------------------------------------- | ---------------- |
| string | PluginParam.name | EfficientNetRecognition | 检测算法名称 |
| string | PluginParam.config_path | perception/traffic_light_recognition/data | 配置文件路径 |
| string | PluginParam.config_file | efficient_net.pb.txt | 配置文件名称 |
| string | detection_output_channel_name | /perception/inner/Tracking | 检测结果输出通道 |
| int32 | gpu_id | 0 | gpu id |

efficient_net模型配置文件位置：modules/perception/traffic_light_recognition/data/efficient_net.pb.txt

| 参数类型 |参数名称 | 默认值 | 含义 |
| -------- | --------------------- | ---------------------------- | ----------------------------- |
| string | 名称 | efficient_net | 模型名称 |
| string | 框架 | Onnx -> TensorRT | 模型推理框架 |
| string | ModelFile.proto_file | efficient_net.onnx | 模型网络结构 |
| string | ModelBlob.inputs | 输入 [1,3,96,96] | 模型输入数据名称及维度 |
| int32 | ModelBlob.outputs | output_cls、outputs_status | 模型输出数据名称及维度 |
| int32 | max_batch_size | 3 | 多批次推理输入模型检测维度 |
| float | classify_threshold | 0.5 | 未知分类阈值 |
| int32 | classify_resize_width | 96 | 模型输入图片调整宽度 |
| int32 | classify_resize_height | 96 | 模型输入图片调整大小高度 |

#### 如何启动

1. 在配置文件中的 modules/perception/data/params 中添加车辆参数配置文件，对应 frame_id 和 sensor_name，启动transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```
2. 启动trafficlight感知
```bash
cyber_launch start /apollo/modules/perception/launch/perception_trafficlight.launch