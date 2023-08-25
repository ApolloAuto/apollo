# Module Name
lidar_tracking

# Introduction
The tracking module is to track the trajectory of obstacles, update their motion status and geometric shape, and assign tracking id.

# Directory Structure
```
├── lidar_tracking         // tracking module
    ├── classifier         // classfier
    ├── conf               // configuration folder
    ├── dag                // module startup file
    ├── data               // module configuration parameters
    ├── launch             // launch file
    ├── interface          // interface
    ├── proto              // tracking module configuration proto file
    ├── tracker            // tracking functions
    ├── lidar_tracking_component.cc // component entrance
    ├── lidar_tracking_component.h
    ├── cyberfile.xml      // package management file
    ├── README.md          // readme file
    └── BUILD              // compile file
```

# Module Input and Output
## Input
| Name              | Type                            | Description       |
| ----------------- | ------------------------------- | ----------------- |
| `frame`           | `onboard::LidarFrameMessage`    | lidar frame message |

## Output
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `frame`           | `onboard::SensorFrameMessage`   | sensor frame message |

SensorFrameMessage is used in multi sensor fusion.

# How to Launch

1. Add vehicle parameter configuration file to modules/perception/data/params, corresponding frame_id and sensor_name, launch transform
```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Modify modules/perception/launch/perception_lidar.launch
- select the dag file to start, use `lidar_tracking.dag` here
- modify msg_adapter. It is used to wrap messages sent by other steps as /apollo/perception/obstacles, this can be used for individual debugging. Modify relevant channel configurations in modules/perception/data/flag/perception_common.flag

3. Modify parameter of modules/perception/lidar_tracking/conf/lidar_tracking_config.pb.txt
- main_sensor_name: main sensor name, same as pointcloud_preprocess component sensor_name
- output_channel_name: output channel name
- multi_target_tracker: multi target tracking function
- fusion_classifier: classification function
- multi_target_tracker_param: multi traget tracking parameters
  - name: method name
  - config_path: configuration path
  - config_file: configuration file name
- fusion_classifier_param: classification parameters
  - name: method name
  - config_path: configuration folder
  - config_file: configuration file name

4. Launch tracking component
```bash
cyber_launch start modules/perception/lidar_tracking/launch/lidar_tracking.launch
```
