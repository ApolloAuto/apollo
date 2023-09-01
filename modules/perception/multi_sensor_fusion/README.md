# perception-multi-sensor-fusion

## Introduction

The multi-sensor fusion module fuses the output results of Lidar, Camera, and Radar multiple sensors to make the
detection results more reliable.

It uses post-processing technology, the algorithm used is probabilistic fusion.

## Directory Structure

```
multi_sensor_fusion
├── BUILD      // bazel build file
├── README.md
├── base       // base data structure
├── common     // filter and utils
├── conf       // dag config file
├── cyberfile.xml  // package description file
├── dag
├── data        // lib's configuration file
├── fusion      // fusion function
├── interface
├── multi_sensor_fusion_component.cc  // component
├── multi_sensor_fusion_component.h
└── proto       // proto file
```

## Input and Output

### MultiSensorFusionComponent

apollo::perception::fusion::MultiSensorFusionComponent

#### Input

| Channel                             | Type                                              | Description                     |
| ----------------------------------- | ------------------------------------------------- | ------------------------------- |
| `/perception/inner/PrefusedObjects` | `apollo::perception::onboard::SensorFrameMessage` | frame contains object detection |

#### Output

| Channel                        | Type                                      | Description                    |
| ------------------------------ | ----------------------------------------- | ------------------------------ |
| `/apollo/perception/obstacles` | `apollo::perception::PerceptionObstacles` | detection results after fusion |

#### How to run

The multi-sensor fusion module does not support running alone, it needs to run together with lidar, camera and radar
detection modules.

You can use the following command to start the whole perception function, including lidar, camera, and radar target
detection, and output their results after fusion.

```
cyber_launch start modules/perception/launch/perception_all.launch
```
