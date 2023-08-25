# Module Name
traffic_light_detection

# Introduction
After the previous projection in the preprocessing stage, a projection frame is obtained on the picture, but the obtained projection frame is not completely reliable, so a larger region of interest (Region of Interest ROI) to be calculated by the projected signal light position is used. to determine the exact bounding box of the semaphore. Signal light detection (detect) is a conventional convolutional neural network detection task, which receives images with ROI information as input data and sequentially outputs bounding boxes.

# Directory Structure
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

# Module Input and Output
## Input
| Name              | Type                            | Description         |
| ----------------- | ------------------------------- | -----------------   |
| `frame`           | `onboard::TrafficDetectMessage` | trafficlight message |


## Output
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `frame`           | `onboard::TrafficDetectMessage` | trafficlight message |

# How to Launch
```bash
cyber_launch start modules/perception/traffic_light_detection/launch/traffic_light_detection.launch
```