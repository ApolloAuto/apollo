# Perception 3.0

## Introduction
Apollo 3.0 has following new features:

 * **CIPV(Closest In-Path Vehicle) detection and Tailgaiting**: The vehicle in front of the ego-car is detected and its trajectory is estimated for more efficient tailgating and lane keeping when lane detection is unreliable.
  * **Asynchronous sensor fusion**: unlike the previous version, Perception in Apollo 3.0 is capable of consolidating all the information and data points by asynchronously fusing LiDAR, Radar and Camera data. Such conditions allow for more comprehensive data capture and reflect more practical sensor environments.
  * **Online pose estimation**: This new feature estimates the pose of an ego-vehicle for every single frame. This feature helps to drive through bumps or slopes on the road with more accurate 3D scene understanding.
  * **Ultrasonic sensors**: Perception in Apollo 3.0 now works with ultrasonic sensors. The output can be used for Automated Emergency Brake (AEB) and vertical/perpendicular parking.
  * **Whole lane line**: Unlike previous lane line segments, this whole lane line feature will provide more accurate and long range detection of lane lines. 
  * **Visual localization**: Camera's are currently being tested to aide and enhance localization
  * **16 beam LiDAR support**

The perception module incorporates the capability of using a front camera and a front radar to recognize obstacles and fuse their individual tracks to obtain a final track list.  The obstacle sub-module detects, classifies and tracks obstacles. This sub-module also predicts obstacle motion and position information (e.g., heading and velocity). For lane line, we construct lane instances by postprocessing lane parsing pixels and calculate the lane relative location to the ego-vehicle (L0, L1, R0, R1, etc)

See Also:

 [Perception algorithms in Apollo 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/perception_apollo_3.0.md).

 [Guideline of sensor installation for Apollo 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Guideline_sensor_Installation_apollo_3.0.md).

## Input

The perception module inputs are:

- Radar data (ROS topic _/apollo/sensor/conti_radar_)
- Image data (ROS topic _/apollo/sensor/camera/obstacle/front_6mm_)
- Extrinsic parameters of radar sensor calibration (from YAML files)
- Extrinsic and Intrinsic parameters of front camera calibration (from YAML files)
- Velocity and Angular Velocity of host vehicle (ROS topic /apollo/localization/pose)

## Output

The perception module outputs are:

* The 3D obstacle tracks with the heading, velocity and classification information (ROS topic _/apollo/perception/obstacles_)
* The lane marker information with fitted curve parameter, spatial information(l0,r0, etc) as well as semantic information (lane type) (ROS topic _/apollo/perception/obstacles_)

    1. Set up the general settings in the configuration file `modules/perception/conf/perception_lowcost.conf`.
    2. Run the command  `./scripts/bootstrap.sh` to launch the web GUI.
    3. Select the vehicle model in the web GUI.
    4. Launch the perception module using the command `./scripts/perception_lowcost_vis.sh start` or by enabling the perception button on the *Module Controller* page of the web GUI. The command for stopping perception is `./scripts/perception_lowcost_vis.sh stop`. Note: please do not try to use GUI to enable perception but use script to stop it, vice versa. 
    5. Download the demo data from the Apollo [Open Data Platform](http://data.apollo.auto).

## Function enable/disable
The perception framework is a directed acyclic graph (DAG). There are three components in DAG configuration, including sub-nodes, edges and shared data. Each function is implemented as a sub-node in DAG. The sub-nodes that share data have an edge from producer to customer.

A typical DAG configuration for a perception module is shown in the example below.  The example DAG configuration features the following:  

- Default obstacle perception that consists of "CameraProcessSubnode", "RadarProcessSubnode" and "FusionSubnode", as shown in *subnode_config*.
- The "CameraProcessSubnode" and "RadarProcessSubnode" that receive sensor data and output obstacle data independently, i.e., the "CameraObjectData" and "RadarObjectData" in *data_config*.
- The "FusionSubnode" that both subscribes the obstacle data and publishes the final results.
- The "LanePostProcessSubnode" processes the lane parsing output from camera detection module and generates lane instances and attributes
- The edge and data configuration that define the links.
- Each function can be disabled by removing the corresponding sub-node, edge, and shared data configuration. However you must ensure that all the input and output configurations are correct.

``` protobuf

# Nvidia Driver and CUDA are required for these 2 subnodes
subnode_config {
    # Camera node
    subnodes {
        id: 3
        name: "CameraProcessSubnode"
        reserve: "device_id:camera;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 4
        name: "MotionService"
        reserve: "device_id:motion_service;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 5
        name: "LanePostProcessingSubnode"
        reserve: "device_id:camera;motion_event_id:1021"
        type: SUBNODE_NORMAL
    }
    subnodes {
        id: 2
        name: "RadarProcessSubnode"
        reserve: "device_id:radar_front;"
        type: SUBNODE_IN
    }
    subnodes {
        id: 31
        name: "FusionSubnode"
        reserve: "pub_driven_event_id:1009;lane_event_id:1010;camera_event_id:1009;radar_event_id:1013;"
        type: SUBNODE_OUT
    }
}

###################################################################
# Define all edges which link nodes.
edge_config {

    # CameraDetectorSubnode -> LanePostProcessingSubnode
    edges {
        id: 106
        from_node: 3
        to_node: 5
        events {
            id: 1004
            name: "lane_postprocessing"
        }
    }

    # CameraProcessSubnode -> FusionSubnode
    edges {
        id: 109
        from_node: 3
        to_node: 31
        events {
            id: 1009
            name: "camera_fusion"
        }
    }

    # LanePostProcessingSubnode -> FusionSubnode
    edges {
        id: 110
        from_node: 5
        to_node: 31
        events {
            id: 1010
            name: "lane_fusion"
        }
    }

    # RadarSubnode -> FusionSubnode
    edges {
        id: 113
        from_node: 2
        to_node: 31
        events {
            id: 1013
            name: "radar_fusion"
        }
    }
}

# Shared Data
data_config {
    datas {
        id: 5
        name: "CameraObjectData"
    }
    datas {
        id: 7
        name: "CameraSharedData"
    }
    datas {
        id: 8
        name: "LaneSharedData"
    }
    datas {
        id: 9
        name: "FusionSharedData"
    }
    datas {
        id: 10
        name: "RadarObjectData"
    }
}


```

### Note
1. Nvidia GPU and CUDA are required to run the perception module with Caffe. Apollo provides the CUDA and Caffe libraries in the release docker image. However, the Nvidia GPU driver is not installed in the dev docker image.

2. To run the perception module with CUDA acceleration, install the exact same version of the Nvidia driver in the docker that is installed in your host machine, and then build Apollo with the GPU option (i.e., using `./apollo.sh build_gpu` or `./apollo.sh build_opt_gpu`).

    See [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).

3. This module contains a redistribution in binary form of a modified version of caffe(https://github.com/BVLC/caffe). 
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