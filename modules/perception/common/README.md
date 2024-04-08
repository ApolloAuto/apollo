# perception-common

## Introduction

`perception-common` is a library that other modules in perception depend on. It mainly includes public algorithm libraries, maps, inference and interface definitions.

## Directory Structure

```
common
├── BUILD
├── README.md
├── algorithm  // algorithm library
├── base       // common data structure
├── camera     // camera data structure
├── cyberfile.xml
├── hdmap      // hdmap wrapper
├── inference  // model inference and acceleration
├── interface
├── lib
├── lidar      // lidar data structure
├── onboard
├── perception_gflags.cc
├── perception_gflags.h
├── proto
├── radar      // radar data structure
├── util.cc
└── util.h
```

## Global gflag configuration

Global configuration is defined in the gflag file, including`modules/perception/common/perception_gflags.h`。

| parameter name                       |  default value                                                                                                             | meaning                                 |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| obs_sensor_intrinsic_path    | /apollo/modules/perception/data/params                                                                                | Sensor internal parameter path        |
| obs_sensor_meta_file         | sensor_meta.pb.txt                                                                                                    | Sensor meta file name           |
| enable_base_object_pool      | enable_base_object_pool                                                                                               | Enable object pool     |
| config_manager_path          | ./                                                                                                                    | Configuration manager path     |
| work_root                    | /apollo/modules/perception                                                                                            | Work list                             |
| lidar_sensor_name            | velodyne128                                                                                                           | lidar sensor name        |
| use_trt                      | false                                                                                                                 | Whether to use tensorrt|
| trt_precision                | 1                                                                                                                     | Precision of tensorrt, 0: 32float, 1: kInt8， 2: kHalf|
| trt_use_static               | true                                                                                                                  | Whether to load tensorrt graph optimization from disk path    |
| use_calibration              | true                                                                                                                  | Whether to use a correction table      |
| use_dynamicshape             | true                                                                                                                  | Whether to use dynamic shapes    |
| collect_shape_info           | true                                                                                                                  | Whether to collect dynamic shape information      |
| dynamic_shape_file           | /apollo/modules/perception/lidar_detection/data/center_point_paddle/pillar_20_0625/collect_shape_info_3lidar_20.pbtxt | Dynamic file path |
| object_template_file         | object_template.pb.txt                                                                                                | Object template configuration file          |
| hdmap_sample_step            | 5                                                                                                                     | High-precision map sampling rate     |

### Modify configuration

The global configuration is modified in `/apollo/modules/perception/data/flag/perception_common.flag`. For example, if we want to modify the value of `obs_enable_hdmap_input` to false, we can add it to the file:

```bash
--obs_enable_hdmap_input=false