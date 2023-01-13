# Perception

## Introduction

The perception module incorporates the capability of using multiple cameras, radars (front and rear) and LiDARs to recognize obstacles and fuse their individual tracks to obtain a final track list.
The obstacle sub-module detects, classifies and tracks obstacles.
This sub-module also predicts obstacle motion and position information (e.g., heading and velocity).
For lane line, we construct lane instances by postprocessing lane parsing pixels and calculate the lane relative location to the ego-vehicle (L0, L1, R0, R1, etc.).

## Model repository

| Model               | Model type               | Model download address                                       |
| ------------------- | ------------------------ | ------------------------------------------------------------ |
| PETR_V1_paddle      | bev_detection            | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/petrv1.zip |
| horizontal_torch    | trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/horizontal_torch.zip |
| quadrate_torch      | trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/quadrate_torch.zip |
| vertical_torch      | trafficlight_recognition | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/vertical_torch.zip |
| denseline_caffe     | lane_detection           | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/denseline_caffe.zip |
| cnnseg16_caffe      | lidar_3d_segmentation    | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/cnnseg16_caffe.zip |
| cnnseg64_caffe      | lidar_3d_segmentation    | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/cnnseg64_caffe.zip |
| center_point_paddle | lidar_3d_detection       | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/center_point_paddle.zip |
| mask_pillars_torch  | lidar_3d_detection       | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/mask_pillars_torch.zip |
| point_pillars_torch | lidar_3d_detection       | https://apollo-pkg-beta.cdn.bcebos.com/perception_model/point_pillars_torch.zip |

## Record repository
| Record name       | Verification                                              | Vehicle configuration      | Map            | Size   | Download address                                             |
| ----------------- | --------------------------------------------------------- | -------------------------- | -------------- | ------ | ------------------------------------------------------------ |
| bev_test.tar.xz   | bev_detection                                             | Apollo Perception Test Bev | default        | 1.93GB | https://apollo-pkg-beta.cdn.bcebos.com/perception_record/bev_test.tar.xz |
| sensor_rgb.tar.xz | camera_detection、lidar_detection、trafficlight_detection | Perception Test V1         | Sunnyvale Loop | 4.4GB  | https://apollo-system.bj.bcebos.com/dataset/6.0_edu/sensor_rgb.tar.xz |
## Remote installation model

Download and install the model in the remote model warehouse to the local through the model deployment tool, for example, download and install PETR_V1_paddle

```python
python modules/tools/amodel/amodel.py install https://apollo-pkg-beta.cdn.bcebos.com/perception_model/petrv1.zip
```

Download record verification model PETR_V1_paddle

```bash
wget https://apollo-pkg-beta.cdn.bcebos.com/perception_record/bev_test.tar.xz
```

Unzip the downloaded record

```bash
tar -xvf bev_test.tar.xz
```

Note: bev_test.tar.xz is only used to test the bev algorithm, it's recommended to download data on nuScenes and make it into a record
## Architecture

The general architecture of the perception module is shown:
![](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/images/Apollo3.5_perception_sensor_based.png)

The detailed perception modules are displayed below.
![](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/images/Apollo6.0_perception_detail.png)

## Input

The perception module inputs are:

- 128 channel LiDAR data (cyber channel /apollo/sensor/velodyne128)
- 16 channel LiDAR data (cyber channel /apollo/sensor/lidar_front, lidar_rear_left, lidar_rear_right)
- Radar data (cyber channel /apollo/sensor/radar_front, radar_rear)
- Image data (cyber channel /apollo/sensor/camera/front_6mm, front_12mm)
- Extrinsic parameters of radar sensor calibration (from YAML files)
- Extrinsic and Intrinsic parameters of front camera calibration (from YAML files)
- Velocity and Angular Velocity of host vehicle (cyber channel /apollo/localization/pose)

## Output

The perception module outputs are:

* The 3D obstacle tracks with the heading, velocity and classification information (cyber channel /apollo/perception/obstacles)
* The output of traffic light detection and recognition (cyber channel /apollo/perception/traffic_light)

### Note
1. Nvidia GPU and CUDA are required to run the perception module with Caffe. Apollo provides the CUDA and Caffe libraries in the release docker image. However, the Nvidia GPU driver is not installed in the dev docker image.

2. To run the perception module with CUDA acceleration, install the exact same version of the Nvidia driver in the docker image that is installed on your host machine, and then build Apollo with the GPU option (i.e., using `./apollo.sh build_opt_gpu`).

    See [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).

3. This module contains a redistribution in binary form of a modified version of [caffe](https://github.com/BVLC/caffe).
A copy of the caffe's original copyright statement is included below:

```
COPYRIGHT

All contributions by the University of California:
Copyright (c) 2014-2017 The Regents of the University of California (Regents)
All rights reserved.

All other contributions:
Copyright (c) 2014-2017, the respective contributors
All rights reserved.

Caffe uses a shared copyright model: each contributor holds copyright over their contributions to Caffe. The project versioning records all such contribution and copyright details. If a contributor wants to further mark their specific copyright on a particular contribution, they should indicate their copyright solely in the commit message of the change when it is committed.

LICENSE

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    CONTRIBUTION AGREEMENT

    By contributing to the BVLC/caffe repository through pull-request, comment, or otherwise, the contributor releases their content to the license and copyright terms herein.
```