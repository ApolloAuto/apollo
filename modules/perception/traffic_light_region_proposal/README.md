# perception-traffic-light-region-proposal

## Introduction

This module is used to query positioning and map signal light information, select a camera for detecting traffic lights
according to the projection of the signal light on the image plane, and finally save the camera selection result.

## Directory Structure

```
├── traffic_light_region_proposal // trafficlight region proposal module
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── interface       // function interface folder
    ├── launch          // launch files
    ├── preprocessor    // main part for preprocess
    ├── proto           // proto files
    ├── traffic_light_region_proposal_component.cc // component interface
    ├── traffic_light_region_proposal_component.h
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## Modules

### TrafficLightsPerceptionComponent

apollo::perception::onboard::TrafficLightsPerceptionComponent

#### Input

| Name     | Type                                  | Description         | Input channal |
| -------- | ------------------------------------- | ------------------- | ------------- |
| `msg`    | `apollo::drivers::Image`              | camera sensor image | /apollo/sensor/camera/front_6mm/image <br/> /apollo/sensor/camera/front_6mm/image |
| `hd-map` | `apollo::perception::map::HDMapInput` | HD map              | - |

>Note: Image data from driver. The default trigger channel include `/apollo/sensor/camera/front_6mm/image` and `/apollo/sensor/camera/front_12mm/image`. The detailed input channel information is in `modules/perception/traffic_light_region_proposal/conf/traffic_light_region_proposal_config.pb.txt` file.

#### Output

| Name    | Type                                                | Description          | Output channal |
| ------- | --------------------------------------------------- | -------------------- | -------------- |
| `frame` | `apollo::perception::onboard::TrafficDetectMessage` | trafficlight message | /perception/inner/Detection |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/traffic_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process.

#### How to Launch

```bash
cyber_launch start modules/perception/traffic_light_region_proposal/launch/traffic_light_region_proposal.launch
```
