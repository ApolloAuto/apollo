# drivers-video

## Introduction
This package is responsible for receiving the native camera image from upd socket and compressing it in h265 encoding.

## Directory Structure
```shell
modules/drivers/video/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── driver.cc
├── driver.h
├── input.h
├── launch
├── proto
├── README.md
├── socket_input.cc
├── socket_input.h
├── tools                       // tool of converting h265 encoding video to jpeg
├── video_driver_component.cc
└── video_driver_component.h
```

## Modules

### SmartereyeComponent

apollo::drivers::video::CompCameraH265Compressed


#### Input

The package will received image is consistent with the user configuration information, only then it starts to parse the data received from the udp socket and compress the image in h265 encoding. In the end send the message and write it to the file specified.

#### Output

| Name  | Type                                               |      Description             |
| ----- | -------------------------------------------------- | ---------------------------- |
| `msg` |         `apollo::drivers::CompressedImage`         |    image in h265 encoding    |

#### configs

| file path                                            | type / struct                                                | Description           |
| ---------------------------------------------------- | ------------------------------------------------------------ | --------------------- |
| `modules/drivers/video/conf/video_front_6mm.pb.txt`  |       `apollo::drivers::video::config::CameraH265Config`     |    video config       |

#### How to Launch

```bash
cyber_launch start modules/drivers/video/launch/video.launch
```
