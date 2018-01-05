# Perception

## Introduction
The perception module incorporates the capability of recognizing obstacles and traffic lights. The obstacle submodule detects, segments, classifies and tracks obstacles in the ROI that is defined by the high-resolution (HD) map. The submodule also predicts obstacle motion and position information (e.g., heading and velocity). Also, the traffic light submodule detects traffic lights and recognizes their status in the images. 

See Also:

 [3D obstacles perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md). 

 [Traffic light perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/traffic_light.md).

## Input

The perception module inputs are:

- Point cloud data (ROS topic _/apollo/sensor/velodyne64/compensator/PointCloud2_)
- Radar data (ROS topic _/apollo/sensor/conti_radar_)
- Image data (ROS topic _/apollo/sensor/camera/traffic/image_long_ & _/apollo/sensor/camera/traffic/image_short_)
- Coordinate frame transformation information over time (ROS topic _/tf_)
- HDMap
- Extrinsic parameters of LiDAR sensor calibration (ROS topic _/tf_static_)
- Extrinsic parameters of radar sensor calibration (from YAML files)
- Extrinsic and Intrinsic parameters of all camera calibration (from YAML files)
- Velocity of host vehicle (ROS topic /apollo/localization/pose)

## Output

The perception module outputs are:

* The 3D obstacle tracks with the heading, velocity and classification information (ROS topic _/apollo/perception/obstacles_)
* Traffic light bounding box and status (ROS topic _/apollo/perception/traffic_light_)
## Instructions

- [ ] 1. Set up the general settings in the configuration file `modules/perception/conf/perception.conf`.
- [ ] 2. Run the command  `./scripts/bootstrap.sh` to launch the web GUI.
- [ ] 3. Select the vehicle model and HD map in the web GUI.
- [ ] 4. Launch the perception module using the command `./scripts/perception.sh start` or by enabling the perception button on the *Module Controller* page of the web GUI. The command for stopping perception is `./scripts/perception.sh stop`.


- [ ] 5. Download the demo data from the Apollo [Open Data Platform](https://console.bce.baidu.com/apollo/task/download).

## Function enable/disable
The perception framework is a directed acyclic graph (DAG). There are three components in DAG configuration, including sub-nodes, edges and shared data. Each function is implemented as a sub-node in DAG. The sub-nodes that share data have an edge from producer to customer.

A typical DAG configuration for a perception module is shown in the example below.  The example DAG configuration features the following:  

- Default obstacle perception that consists of "LidarProcessSubnode", "RadarProcessSubnode" and "FusionSubnode", as shown in *subnode_config*. 
- The "LidarProcessSubnode" and "RadarProcessSubnode" that receive sensor data and output obstacle data independently, i.e., the "LidarObjectData" and "RadarObjectData" in *data_config*. 
- The "FusionSubnode" that both subscribes the obstacle data and publishes the final results. 
- Traffic light perception that is composed of "TLPreprocessorSubnode" and "TLProcSubnode". 
- The edge and data configuration that define the links. 
- Each function can be disabled by removing the corresponding sub-node, edge, and shared data configuration. However you must ensure that all the input and output configurations are correct.

``` protobuf
  # Define all nodes in DAG streaming.
  subnode_config {
      # 64-Lidar Input nodes.
      subnodes {
          id: 1
          name: "LidarProcessSubnode"
          reserve: "device_id:velodyne64;"
          type: SUBNODE_IN
      }

      # Front radar Input nodes.
      subnodes {
          id: 2
          name: "RadarProcessSubnode"
          reserve: "device_id:radar;"
          type: SUBNODE_IN
      }

      # Fusion node.
      subnodes {
          id: 31
          name: "FusionSubnode"
          reserve: "pub_driven_event_id:1001;lidar_event_id:1001;radar_event_id:1002;"
          type: SUBNODE_OUT
      }

      # TrafficLight Preprocess node.
      subnodes {
          id: 41
          name: "TLPreprocessorSubnode"
          type: SUBNODE_IN
      }
      
      # TrafficLight process node.
      subnodes {
          id: 42
          name: "TLProcSubnode"
          type: SUBNODE_OUT
      }
  }

  ###################################################################
  # Define all edges linked nodes.
  edge_config {
      # 64-Lidar LidarProcessSubnode -> FusionSubnode
      edges {
          id: 101
          from_node: 1
          to_node: 31
          events {
              id: 1001
              name: "lidar_fusion"
          }
      }

      # Radar RadarProcessSubnode -> FusionSubnode
      edges {
          id: 102
          from_node: 2
          to_node: 31
          events {
              id: 1002
              name: "radar_fusion"
          }
      }
      
      # TLPreprocessorSubnode -> TLProcSubnode
      edges {
          id: 201
          from_node: 41
          to_node: 42
          events {
              id: 1003
              name: "traffic_light"
          }
      }
  }

  ###################################################################
  # Define all shared data.
  data_config {
      datas {
          id: 1
          name: "LidarObjectData"
      }
      datas {
          id: 2
          name: "RadarObjectData"
      }
      datas {
          id: 3
          name: "TLPreprocessingData"
      }
  }
```

**Note**: Nvidia GPU and CUDA are required to run the perception module with Caffe. Apollo provides the CUDA and Caffe libraries in the released docker. However, the Nvidia GPU driver is not installed in the released dev docker image. 

To run the perception module with CUDA acceleration, install the exact same version of the Nvidia driver in the docker that is installed in your host machine, and then build Apollo with the GPU option (i.e., using `./apollo.sh build_gpu` or `./apollo.sh build_opt_gpu`). 

See [How to Run Perception Module on Your Local Computer](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_perception_module_on_your_local_computer.md).
