# perception-msg-adapter

## Introduction

The msg adapter module is used to forwarding message inside the perception module.

Channel names for internal messages usually have an inner field `inner`. These messages are only passed between threads,
not between processes, so you can't use `cyber_monitor` to view them. Because these messages are generally relatively
large, such as image and point cloud data, in order to avoid copying, serialization and deserialization, we use the
method of directly passing objects, so these messages can only be passed within the thread.

For the convenience of debugging, you can use the msg adapter module to forward messages, so that you can view them
through `cyber_monitor`.

## Directory Structure

```
msg_adapter
├── BUILD       // bazel build file
├── README.md
├── common      // gflags parameters
├── convert     // convert functions
├── cyberfile.xml   // package description file
├── dag
├── launch
├── msg_adapter_component.cc   // component
├── msg_adapter_component.h
└── msg_converter.h
```

## Modules

### MsgAdapterComponent

apollo::perception::MsgAdapterComponent

#### Input

| Input channel                                 | Type                                              | Description                         |
| --------------------------------------- | ------------------------------------------------- | ----------------------------------- |
| `/perception/inner/location_refinement` | `apollo::perception::onboard::CameraFrame`        | camera detection message            |
| `/perception/lidar/detection`           | `apollo::perception::onboard::LidarFrameMessage`  | lidar detection message             |
| `/perception/inner/PrefusedObjects`     | `apollo::perception::onboard::SensorFrameMessage` | camera/lidar/radar tracking message |

Each of the above channels corresponds to a conversion function. It is best to enable only one at a time. If multiple
channels are to be enabled, it is best to modify the output channel at the same time.

#### Output

| Out channel                        | Type                                      | Description        |
| ------------------------------ | ----------------------------------------- | ------------------ |
| `/apollo/perception/obstacles` | `apollo::perception::PerceptionObstacles` | obstacles detected |

#### How to run

The msg adapter module is used with other modules to forward messages, so it is generally not started separately.

#### How to modify the input and output topics

You can modify input and output topics in `modules/perception/data/flag/perception_common.flag`.

```
--cameraframe_to_obstacles_in=/perception/inner/location_refinement
# --lidarframe_to_obstacles_in=/perception/lidar/detection
--sensorframe_message_to_obstacles_in=/perception/inner/PrefusedObjects
```

#### How to add my own convert function

You can follow below steps to add new convert.

1. Get the type of message to convert, typically including input and output messages. Add conversion function in
   `modules/perception/msg_adapter/convert/convert.h`.
2. Register conversion function in `modules/perception/msg_adapter/msg_adapter_component.cc`. And add input and output
   topics.
