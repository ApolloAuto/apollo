# perception-traffic-light-detection

## Introduction

After the previous projection in the preprocessing stage, a projection frame is obtained on the picture, but the
obtained projection frame is not completely reliable, so a larger region of interest (Region of Interest ROI) to be
calculated by the projected signal light position is used. to determine the exact bounding box of the semaphore. Signal
light detection (detect) is a conventional convolutional neural network detection task, which receives images with ROI
information as input data and sequentially outputs bounding boxes.

## Directory Structure

```
├── traffic_light_detection // trafficlight detection module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // model params
    ├── detection       // main part for detect
    ├── interface       // function interface folder
    ├── launch          // launch files
    ├── proto           // proto files
    ├── traffic_light_detection_component.cc // component interface
    ├── traffic_light_detection_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    ├── README_cn.md
    └── BUILD
```

## Modules

### TrafficLightDetectComponent

apollo::perception::onboard::TrafficLightDetectComponent

#### Input

| Name    | Type                                                | Description          | Input channal |
| ------- | --------------------------------------------------- | -------------------- | ------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Detection |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/Detection`. The detailed input channel information is in `modules/perception/traffic_light_detection/dag/traffic_light_detection.dag` file. By default, the upstream components of the messages received by the component include `traffic_light_region_proposal`.

#### Output

| Name    | Type                                                | Description          | Output channal |
| ------- | --------------------------------------------------- | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Retection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/traffic_light_detection/conf/traffic_light_detection_config.pb.txt` file.


#### configuration

component configuration files： modules/perception/traffic_light_detection/conf/traffic_light_detection_yolox_config.pb.txt

| parameter type | parameter name                       | default value                                | meaning             |
| -------- | ----------------------------- | --------------------------------------- | ---------------- |
| string   | PluginParam.name              | TrafficLightTLDetectorYolox                   | detection algorithm name     |
| string   | PluginParam.config_path       | perception/traffic_light_detection/data | configuration file path    |
| string   | PluginParam.config_file       | detection_yolox.pb.txt                  | profile name     |
| string   | detection_output_channel_name | /perception/inner/Retection             | detection result output channel |
| int32    | gpu_id                        | 0                                       | gpu id           |

Yolox model configuration file location：modules/perception/traffic_light_detection/data/detection_yolox.pb.txt

| parameter type | parameter name                 | default value                       | meaning                          |
| -------- | --------------------- | ---------------------------- | ----------------------------- |
| string   | name                  | tl_detection_yolox           | model name |
| string   | framework             | Onnx -> TensorRT                     | model inference framework                  |
| string   | ModelFile.proto_file  | yolox.onnx             | model network structure                  |
| string   | ModelBlob.inputs      | images [1,3,384,384]                 | model input data name and dimension        |
| int32    | ModelBlob.outputs     | bbox、conf、cls                       | model output data name and dimension        |
| int32    | max_batch_size     | 3                      | Multi-batch inference input model detection dimension        |
| int32    | min_crop_size     | 400                      | Crop box size       |
| int32    | classify_resize_width     | 384                     | model input image resize width       |
| int32    | classify_resize_height     | 384                     | model input image resize height      |

#### How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```
2. Launch trafficlight perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_trafficlight.launch
```
