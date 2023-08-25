# Component Name
lidar_output

## Introduction
Convert the SensorFrameMessage to PerceptionObstacles.

## Structure
```
├── lidar_output
    ├── conf         // component config file
    ├── dag          // component dag file
    ├── proto        // definition of data structure
    ├── lidar_output_component.cc // component inference
    ├── lidar_output_component.h
    ├── cyberfile.xml   // package configs
    ├── README.md
    └── BUILD
```

## 模块输入输出

### 输入
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -----------------    |
| `message`         | `SensorFrameMessage`            | sensor frame message |

### 输出
| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `out_message`     | `PerceptionObstacles`           | /apollo/perception/obstacles |

## How to use
1. Add vehicle params configs to modules/perception/data/params，keep frame_id and sensor_name consistent, start the tf
```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Select the lidar model，download the model files and put them into modules/perception/data/models using amodel tool.
```bash
amodel install https://xxx.zip
```

3. Modify modules/perception/lidar_output/dag/lidar_output.dag
- config_file_path: path of config path
- reader channel: the name of input channel

4. Modify modules/perception/lidar_output/conf/lidar_output.pb.txt
- output_channel_name: the name of output channel
  
5. start the lidar component
```bash
mainboard -d modules/perception/lidar_output/dag/lidar_output.dag
```