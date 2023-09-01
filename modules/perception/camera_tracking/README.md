# perception-camera-tracking

## Introduction

The function of camera tracking module is to process the camera detection obstacles by using the omt obstacle tracker,
and output the tracked obstacles for following multi sensor fusion.

## directory structure

```
├── camera_tracking     // camera object tracking module
    ├── base            // definition of tracked_target, frame_list, etc.
    ├── common          // common used functions
    ├── conf            // module configuration files
    ├── dag             // dag files
    ├── data            // tracker configuration files
    ├── feature_extract // feature extractor for camera tracking
    ├── interface       // function interface files
    ├── proto           // proto files
    ├── tracking        // omt obstacle tracker
    ├── camera_tracking_component.cc
    ├── camera_tracking_component.h // component interface
    ├── cyberfile.xml   // package management profile
    ├── README.md
    └── BUILD
```

## modules

### CameraTrackingComponent

apollo::perception::camera::CameraTrackingComponent

#### input

| Name    | Type                                       | Description            |
| ------- | ------------------------------------------ | ---------------------- |
| `frame` | `apollo::perception::onboard::CameraFrame` | camera detection frame |

#### output

| Name    | Type                                              | Description              |
| ------- | ------------------------------------------------- | ------------------------ |
| `frame` | `apollo::perception::onboard::SensorFrameMessage` | camera tracked obstacles |

#### how to launch module

1. modify the file: `modules/perception/camera_tracking/dag/camera_tracking.dag`

- config_file_path: the path of module config files
- reader channel: the channel name of input message

2. modify the file: `modules/perception/camera_tracking/conf/camera_tracking_config.pb.txt`

- output_channel_name: the channel name of output message
- image_width and image height: the size of input images
- gpu_id: the gpu id
- plugin_param.name: the name of used camera tracker
- plugin_param.config_path: the config path of used camera tracker
- plugin_param.config_file: the config name of used camera tracker

3. run `mainboard -d modules/perception/camera_tracking/dag/camera_tracking.dag`
