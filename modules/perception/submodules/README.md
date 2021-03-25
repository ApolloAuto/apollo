# Perception Submodules

## Introduction
Perception is composed of several submodules. A submodule means a specific algorithm in a module.
For example, a specific lidar segmentation algorithm is implemented in LidarSegmentationSubmodule in Perception.
Configs for submodules and algorithms can be found in /apollo/modules/perception/conf.
Among these configs, perception_common.flag is a collection of most frequently used gflag config items.
In most cases, modifying perception_common.flag can satisfy the requirements for Perception.

## Submodules
List of submodules in Perception:
* Camera detection submodule
* Lidar detection submodule
* Lidar segmentation submodule
* Lidar tracking submodule
* Radar detection submodule
* Lane detection submodule
* Traffic light submodule
* Fusion submodule
* Motion submodule

### Camera detection submodule
This submodule detects obstacles using multiple camera sensors. It is changed from 
a component called FusionCameraDetectionComponent before. It utilizes YOLO and SMOKE as the
detection algorithms now.

### Lidar detection submodule
This submodule detects obstacles using an end-to-end model which outputs cuboid-shape
polygons directly. It is transferred from DetectionComponent. For now, 
its detection model is PointPillars which predicts obstacle's position, size, 
category and direction at the same time.

### Lidar segmentation submodule
Moved from SegmentationComponent, this submodule detects obstacles from lidar point cloud.
It uses a segmentation based method, CNNSegmentation, to segment foreground cells 
from pre-processed point cloud first.
After that, these cells are clustered into obstacle objects, followed by post-processing.
Finally, a MinBox are build, recovering the bounding box given the polygon points of an obstacle.
For complete explanation of the algorithm, please read
[3D Obstacle Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md).

### Lidar tracking submodule
Lidar tracking submodule, moved from RecognitionComponent, is to track the obstacles
and update measurements of the obstacle objects including tracking id, velocity, category, etc.
More details are explained in
[3D Obstacle Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md).

### Radar detection submodule
This submodule mainly adjusts the obstacles detected by the Radar sensor and tracks them.
The output obstacles are of cuboid shape.

### Lane detection submodule
Lane detection submodule is transferred from LaneDetectionComponent. It is capable of 
detecting lane lines. So far there are two algorithms can be used to perform lane detection.
They are DarkSCNN and DenseLine. Besides, lane detection service is directly invoked by 
camera detection submodule. Therefore, it is no need to launch lane detection submodule
when camera detection submodule is already running.

### Traffic light submodule
Traffic light submodule, transferred from TrafficLightsPerceptionComponent, is a
major submodule in Perception. It is in charge of recognizing traffic light signals
corresponding to the current traffic lane. The detailed explanation of the algorithm
is in
[Traffic Light](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/traffic_light.md).

### Fusion submodule
Fusion submodule, which was FusionComponent before, performs multi sensor fusion for obstacles.
It receives obstacles from sensors like camera, lidar and radar, updates tracks of the 
obstacles, and fuses them considering the position, category and motion, etc.
Some details are explained in
[3D Obstacle Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md).

### Motion submodule
This submodule is modified from MotionService which is a component as well.
Currently, only CIPV (closest in path vehicle) service in camera obstacle detection
needs motion submodule which provides ego vehicle status, including velocity and rotation rate.