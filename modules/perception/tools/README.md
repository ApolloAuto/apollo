# perception-tools

## Introduction

`perception-tools` provides debug tools for perception modules.

## Directory Structure

```
tools
├── BUILD
├── README.md
├── common
├── cyberfile.xml
├── exporter         // message decompression tool
├── offline
└── offline_camera_detection  // camera offline detection tool
```

## exporter
`exporter` is used to subscribe and save lidar/camera messages.

```shell
cd /apollo

# run
/apollo/bazel-bin/modules/perception/tools/exporter/msg_exporter modules/perception/tools/exporter/export_msg.config
```
Messages are saved in a folder named by the corresponding channel.


#### Parameters
Write the channel names to be subscribed to in the `export_msg.config` file. Line breaks are required between multiple channels.


## Offline_camera_detection
`offline_camera_detection` is used to test camera image detection. Its input is an image and its output are detected 2D bounding boxes.

```shell
cd modules/perception/tools/offline_camera_detection

# run
bash offline_camera_detection.sh --config_file=yolo.pb.txt --detector_name=YoloObstacleDetector
```
The detection results are saved in the `data/output` directory


#### Parameters
The supported parameters are as follows.

| Parameter type | Parameter name | Default value                                              | Description                          |
|----------------|----------------|------------------------------------------------------------|--------------------------------------|
| int32          | height         | 1080                                                       | image height                         |
| int32          | width          | 1920                                                       | image width                          |
| int32          | gpu_id         | 0                                                          | gpu id                               |
| string         | dest_dir       | data/output                                                | output dir                           |
| string         | dist_type      | ""                                                         | dist pred type: H-on-h, H-from-h     |
| string         | kitti_dir      | ""                                                         | pre-detected obstacles (skip Detect) |
| string         | root_dir       | /apollo/modules/perception/tools/offline_camera_detection/ | image root dir                       |
| string         | image_ext      | .jpg                                                       | extension of image name              |
| string         | config_path    | perception/camera_detection_multi_stage/data                        | config path                          |
| string         | config_file    | yolox3d.pb.txt                                               | config file                          |
| string         | camera_name    | front_6mm                                                  | camera name                          |
| string         | detector_name  | Yolox3DObstacleDetector                                      | detector name                        |
