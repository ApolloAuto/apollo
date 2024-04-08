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

#### How to Launch

```bash
cyber_launch start modules/perception/traffic_light_detection/launch/traffic_light_detection.launch
```
