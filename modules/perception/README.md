# Perception 2.5

## Introduction
In Apollo 2.5, the perception module incorporates the capability of using front camera and front radar to recognize obstacles and fuse their individual tracks to obtain final track list.  The obstacle submodule detects, classifies and tracks obstacles. The submodule also predicts obstacle motion and position information (e.g., heading and velocity). For lane line, we construct lane instances by postprocessing lane parsing pixels and calculate out lane relative location to the ego-vehicle (L0, L1, R0, R1, etc)

See Also:

 [Perception algorithms in Apollo 2.5](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/perception_apollo_2.5.md).

 [Guideline of sensor installation for Apollo 2.5](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Guideline_sensor_Installation_apollo_2.5.md).

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
* The lane marker information with fitted curve paramter, spatial information(l0,r0, etc) as well as semantic information (lane type) (ROS topic _/apollo/perception/obstacles_)

- [ ] 1. Set up the general settings in the configuration file `modules/perception/conf/perception_lowcost.conf`.
- [ ] 2. Run the command  `./scripts/bootstrap.sh` to launch the web GUI.
- [ ] 3. Select the vehicle model in the web GUI.
- [ ] 4. Launch the perception module using the command `./scripts/perception_lowcost.sh start` or by enabling the perception button on the *Module Controller* page of the web GUI. The command for stopping perception is `./scripts/perception_lowcost.sh stop`. Note: please do not try to use GUI to enable perception but use script to stop it, vice versa. 

- [ ] 5. Download the demo data from the Apollo [Open Data Platform](http://data.apollo.auto).

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



 **Note**: Nvidia GPU and CUDA are required to run the perception module with Caffe. Apollo provides the CUDA and Caffe libraries in the release docker image. However, the Nvidia GPU driver is not installed in the dev docker image.

To run the perception module with CUDA acceleration, install the exact same version of the Nvidia driver in the docker that is installed in your host machine, and then build Apollo with the GPU option (i.e., using `./apollo.sh build_gpu` or `./apollo.sh build_opt_gpu`).

See [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).
