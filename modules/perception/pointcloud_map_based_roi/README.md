# perception-pointcloud-map-based-roi

## Introduction

Filter the point cloud outside the ROI.

The Region of Interest (ROI) specifies the drivable area that includes road surfaces and junctions that are retrieved
from the HD (high-resolution) map. The HDMap ROI filter processes LiDAR points that are outside the ROI, removing
background objects, e.g., buildings and trees around the road. What remains is the point cloud in the ROI for subsequent
processing.

Given an HDMap, the affiliation of each LiDAR point indicates whether it is inside or outside the ROI. Each LiDAR point
can be queried with a lookup table (LUT) of 2D quantization of the region around the car. The input and output of the
HDMap ROI filter module are summarized in the table below.

## Structure

```
├── pointcloud_map_based_roi
    ├── conf            // component config file
    ├── dag             // component dag file
    ├── data            // config of map_manager and roi_filter
    ├── map_manager     // Set the ROI based on the hdmap
    ├── roi_filter      // filter point cloud outside the ROI
    ├── interface       // definition of BaseROIFilter
    ├── proto           // definition of data structure
    ├── pointcloud_map_based_roi_component.cc // omponent inference
    ├── pointcloud_map_based_roi_component.h
    ├── cyberfile.xml   // package configs
    ├── README.md
    └── BUILD
```

## Modules

### PointCloudMapROIComponent

apollo::perception::lidar::PointCloudMapROIComponent

#### 输入

| Name    | Type                                             | Description         | Input channal |
| ------- | ------------------------------------------------ | ------------------- | ------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message | /perception/lidar/pointcloud_preprocess |

>Note: The input channel is structure type data. The default trigger channel is `/perception/lidar/pointcloud_preprocess`. The detailed input channel information is in `modules/perception/pointcloud_map_based_roi/dag/pointcloud_map_based_roi.dag` file. By default, the upstream components of the messages received by the component include `pointcloud_preprocess`.

#### 输出

| Name    | Type                                             | Description                                                        | Output channal |
| ------- | ------------------------------------------------ | ------------------------------------------------------------------ | -------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | LidarFrame's roi_indices: save the indices of point inside the ROI | /perception/lidar/pointcloud_map_based_roi |

>Note: The output channel is structure type data. The message is defined in the `modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h` file. The output channel message data can be subscribed by components in the same process. The detailed output channel information is in `modules/perception/pointcloud_map_based_roi/conf/pointcloud_map_based_roi_config.pb.txt` file.

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

3. Modify `modules/perception/pointcloud_map_based_roi/dag/pointcloud_map_based_roi.dag`

- config_file_path: path of config path
- reader channel: the name of input channel

4. Modify `modules/perception/pointcloud_map_based_roi/conf/pointcloud_map_based_roi_config.pb.txt`

- output_channel_name: the name of output channel
- use_map_manager: whether use map_manager
- enable_hdmap: whether has hdmap as input
- roi_filter: name of roi filter
- config_path: the path of config
- config_file: the name of config

5. start the lidar component

```bash
cyber_launch start modules/perception/launch/perception_lidar.launch
```
