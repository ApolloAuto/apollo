# Module Name
traffic_light_region_proposal

# Introduction
This module is used to query positioning and map signal light information, select a camera for detecting traffic lights according to the projection of the signal light on the image plane, and finally save the camera selection result.

# Directory Structure
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

# Module Input and Output
## Input
| Name              | Type                            | Description         |
| ----------------- | ------------------------------- | -----------------   |
| `msg`             | `apollo::drivers::Image`        | camera sensor image |
| `hd-map`          | `HDMapInput`                    | HD map              |

## Output
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `frame`           | `onboard::TrafficDetectMessage` | trafficlight message |

# How to Launch
```bash
cyber_launch start modules/perception/traffic_light_region_proposal/launch/traffic_light_region_proposal.launch
```