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

| Name    | Type                                             | Description         |
| ------- | ------------------------------------------------ | ------------------- |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | lidar frame message |

#### 输出

| Name    | Type                                             | Description                                                        |
| ------- | ------------------------------------------------ | ------------------------------------------------------------------ |
| `frame` | `apollo::perception::onboard::LidarFrameMessage` | LidarFrame's roi_indices: save the indices of point inside the ROI |

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
