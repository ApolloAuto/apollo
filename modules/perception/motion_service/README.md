# perception-motion-service

## Introduction

The motion service module provides current motion estimation.

## Directory Structure

```
motion_service
├── BUILD      // bazel build file
├── README.md
├── conf       // dag config file
├── cyberfile.xml  // package description file
├── dag
├── launch
├── motion     // main process
├── motion_service_component.cc   // component
├── motion_service_component.h
└── proto      // proto file
```

## Modules

### MotionServiceComponent

apollo::perception::camera::MotionServiceComponent

#### Input

| Channel                                 | Type                                              | Description          | Name |
| --------------------------------------- | ------------------------------------------------- | -------------------- | ---- |
| `/apollo/sensor/camera/front_6mm/image` | `apollo::perception::camera::ImageMsgType`        | camera drive message | `msg`|
| `/apollo/localization/pose`             | `apollo::perception::camera::LocalizationMsgType` | localization message | `msg`|

#### Output

| Channel                             | Type                                | Description            | Name |
| ----------------------------------- | ----------------------------------- | ---------------------- | ---- |
| `/apollo/perception/motion_service` | `apollo::perception::MotionService` | motion service message | `msg`|

#### How to run

You can start the lane detection module with the following command.

```bash
cyber_launch start modules/perception/motion_service/launch/motion_service.launch
```
