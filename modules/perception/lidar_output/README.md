# perception-lidar-output

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

## Modules

### LidarOutputComponent

apollo::perception::lidar::LidarOutputComponent

#### Input

| Name      | Type                                              | Description          | Input channal |
| --------- | ------------------------------------------------- | -------------------- | ------------- |
| `message` | `apollo::perception::onboard::SensorFrameMessage` | sensor frame message | /perception/inner/PrefusedObjects |

>Note: The input channel is structure type data. The default trigger channel is `/perception/inner/PrefusedObjects`. The detailed input channel information is in `modules/perception/lidar_output/dag/lidar_output.dag` file.

#### Output

| Name          | Type                                               | Description                  | Output channal |
| ------------- | -------------------------------------------------- | ---------------------------- | -------------- |
| `out_message` | `apollo::perception::onboard::PerceptionObstacles` | lidar output message         | /apollo/perception/obstacles |

>Note: The output channel is proto type data. The upstream and downstream components will be started in different processes and can receive the message normally. The detailed output channel information is in `modules/perception/lidar_output/conf/lidar_output_config.pb.txt` file.

#### How to use

1. Add vehicle params configs to `modules/perception/data/params`，keep frame_id and sensor_name consistent, start the
   tf

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```

2. Select the lidar model，download the model files and put them into `modules/perception/data/models` using `amodel`
   tool.

```bash
amodel install https://xxx.zip
```

3. Modify `modules/perception/lidar_output/dag/lidar_output.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

4. Modify `modules/perception/lidar_output/conf/lidar_output_config.pb.txt`

- output_channel_name: the name of output channel

5. start the lidar component

```bash
mainboard -d modules/perception/lidar_output/dag/lidar_output.dag
```
