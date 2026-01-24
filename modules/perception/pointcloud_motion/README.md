# Module Name
pointcloud_motion

# Introduction
The pointcloud motion module assigns a motion label to each point for pointcloud.

# Directory Structure
```
├── pointcloud_motion   // pointcloud_motion component
    ├── conf               // configuration folder
    ├── dag                // module dag file
    ├── data               // module configuration parameters
    ├── interface          // interface of parser
    ├── launch             // launch file
    ├── parser             // parser method
    ├── proto              // pointcloud motion module configuration proto file
    ├── BUILD              // build file
    ├── cyberfile.xml      // package management file
    ├── pointcloud_motion_component.cc  // component entrance
    ├── pointcloud_motion_component.h
    └── README
```

# Module Input and Output
## Input
| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `msg`             | `onboard::LidarFrameMessage`    | lidar frame message |

## Output
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `frame`           | `onboard::LidarFrameMessage`    | lidar frame message |

# How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params in profile, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start /apollo/modules/transform/launch/static_transform.launch
```

2. Modify modules/perception/launch/perception_lidar.launch
- select the dag file to start, add `pointcloud_motion.dag` to perception_lidar.launch
- modify msg_adapter. It is used to wrap messages sent by other steps as /apollo/perception/obstacles, this can be used for individual debugging. Modify relevant channel configurations in modules/perception/data/flag/perception_common.flag

3. Modify parameters of modules/perception/pointcloud_motion/conf/pointcloud_motion.pb.txt in profile
- output_channel_name: output channel name
- plugin_param: plugin parameters
  - name: method name
  - config_path: configuration folder
  - config_file: configuration file name

4. Launch pointcloud perception
```bash
cyber_launch start /apollo/modules/perception/launch/perception_lidar.launch
```
