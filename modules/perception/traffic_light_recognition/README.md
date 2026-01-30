# perception-traffic-light-recognition

## Introduction

The purpose of the traffic light recognition module is to recognize the color of the traffic light. This task is
completed using a conventional convolutional neural network. The input of the recognition module is an image with ROI
information and a set of bounding box information as input data, and the output is a four-dimensional vector. ,
respectively represent the probability that each bounding box is black, red, yellow and green, if and only if the
probability is large enough, the class with the highest probability will be identified as the state of the signal light.
Otherwise the semaphore state is set to Unknown, indicating that the state is undetermined.

## Directory Structure

```
├── traffic_light_recognition // trafficlight recognition module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // model params
    ├── detection       // main part for recogn
    ├── interface       // function interface folder
    ├── launch          // launch files
    ├── proto           // proto files
    ├── traffic_light_recognition_component.cc // component interface
    ├── traffic_light_recognition_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    ├── README_cn.md
    └── BUILD
```

## Modules

### TrafficLightRecognComponent

apollo::perception::onboard::TrafficLightRecognComponent

#### Input

| Name    | Type                                                | Description          | Input channal |
| ------- | --------------------------------------------------- | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Retection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/Retection`. The detailed input channel information is in `modules/perception/traffic_light_recognition/dag/traffic_light_recognition.dag` file. By default, the upstream components of the messages received by the component include `traffic_light_detection`.

#### Output

| Name    | Type                                                | Description          | Output channal |
| ------- | --------------------------------------------------- | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Tracking |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/traffic_light_recognition/conf/traffic_light_recognition_config.pb.txt` file.

#### configuration

component configuration files： modules/perception/traffic_light_recognition/conf/traffic_light_recognition_config.pb.txt

| parameter type | parameter name                       | default value                                | meaning             |
| -------- | ----------------------------- | --------------------------------------- | ---------------- |
| string   | PluginParam.name              | EfficientNetRecognition                   | detection algorithm name     |
| string   | PluginParam.config_path       | perception/traffic_light_recognition/data | configuration file path    |
| string   | PluginParam.config_file       | efficient_net.pb.txt                  | profile name     |
| string   | detection_output_channel_name | /perception/inner/Tracking             | detection result output channel |
| int32    | gpu_id                        | 0                                       | gpu id           |

efficient_net model configuration file location：modules/perception/traffic_light_recognition/data/efficient_net.pb.txt

| parameter type | parameter name                 | default value                       | meaning                          |
| -------- | --------------------- | ---------------------------- | ----------------------------- |
| string   | name                  | efficient_net           | model name |
| string   | framework             | Onnx -> TensorRT                     | model inference framework                  |
| string   | ModelFile.proto_file  | efficient_net.onnx            | model network structure                  |
| string   | ModelBlob.inputs      | inputs [1,3,96,96]                 | model input data name and dimension        |
| int32    | ModelBlob.outputs     | outputs_cls、outputs_status                       | model output data name and dimension        |
| int32    | max_batch_size     | 3                      | Multi-batch inference input model detection dimension        |
| float    | classify_threshold     | 0.5                     | Unknown classification threshold       |
| int32    | classify_resize_width     | 96                     | model input image resize width       |
| int32    | classify_resize_height     | 96                     | model input image resize height      |

#### How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```
2. Launch trafficlight perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_trafficlight.launch
