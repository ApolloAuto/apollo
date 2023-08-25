# Module Name
traffic_light_recognition

# Introduction
The purpose of the traffic light recognition module is to recognize the color of the traffic light. This task is completed using a conventional convolutional neural network. The input of the recognition module is an image with ROI information and a set of bounding box information as input data, and the output is a four-dimensional vector. , respectively represent the probability that each bounding box is black, red, yellow and green, if and only if the probability is large enough, the class with the highest probability will be identified as the state of the signal light. Otherwise the semaphore state is set to Unknown, indicating that the state is undetermined.

# Directory Structure
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
cyber_launch start modules/perception/traffic_light_recognition/launch/traffic_light_recognition.launch
```