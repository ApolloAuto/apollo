# Lidar Perception

## Introduction

Lidar perception module receives messages from lidar driver, processes them with
detection and tracking algorithms as well as output tracked cuboid-shape obstacles.
In detail, Apollo uses PointPillars model as the detection algorithm to detect
obstacles and classify them as cars, buses, trucks, pedestrians, etc. Next,
the tracking algorithm tracks these obstacles according to detection results from
preceding frames. At the end of lidar perception module, tracked obstacles are
sent to fusion component to get fusion with obstacles from other components such
as camera and radar components.

## Architecture

The processing flow of lidar perception module is shown below:
![](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/images/lidar_perception_data_flow.png)

## PointPillars Detection Model

Lidar perception module utilizes [PointPillars](https://arxiv.org/abs/1812.05784)
as the main detection method. PointPillars model only takes point cloud as input,
without images or other sensor data. It consists of 2 stages. The first stage
is the pillar feature extractor (PFE) which extracts feature from pillars that are grid
representations of the point cloud from bird's eye view (BEV). After PFE layers,
point cloud is transformed into 2D feature maps. Taking these BEV pseudo-images,
the second stage, region proposal network (RPN), which is mainly composed of convolution
layers generates bounding boxes and classification labels of obstacles.

Performance of PointPillars model on NuScenes benchmark is shown below:
- bus Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 24.74, 49.15, 62.74, 64.49
- car Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 63.49, 76.87, 80.41, 82.35
- truck Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 13.40, 27.94, 34.53, 37.48
- barrier Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 9.89, 26.34, 35.82, 43.16
- motorcycle Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 13.90, 20.43, 21.06, 21.23
- pedestrian Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 63.86, 65.36, 67.34, 69.39
- traffic_cone Nusc dist AP@0.5, 1.0, 2.0, 4.0
  - 33.03, 35.34, 38.05, 43.38