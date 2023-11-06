# drivers-smartereye

## Introduction
This package is responsible for receiving the native camera image of smartereye device inputed from upd socket and parsing it to send it to the perception-camera-detecetion package for subsequent processing.

## Directory Structure
```shell
modules/drivers/smartereye/
├── BUILD
├── compress_component.cc
├── compress_component.h
├── conf
├── cyberfile.xml
├── dag
├── drivers-smartereye.BUILD
├── launch
├── proto
├── README.md
├── smartereye_component.cc
├── smartereye_component.h
├── smartereye_device.cc
├── smartereye_device.h
├── smartereye_handler.cc
└── smartereye_handler.h
```

## Modules

### SmartereyeComponent

apollo::drivers::smartereye::SmartereyeComponent


#### Input

The package will received image is consistent with the user configuration information, only then it starts to parse the data received from the udp socket and send the message.

#### Output

| Name  | Type                                                 |      Description             |
| ----- | ---------------------------------------------------- | ---------------------------- |
| `msg` |         `apollo::drivers::smartereye::Image`         |    parsed smartereye image   |
| `msg` |   `apollo::drivers::smartereye::SmartereyeObstacles` | obstacles detected by device |
| `msg` |   `apollo::drivers::smartereye::SmartereyeLanemark`  |    lane detected by device   |

#### configs

| file path                                            | type / struct                                           | Description           |
| ---------------------------------------------------- | ------------------------------------------------------- | --------------------- |
| `modules/drivers/smartereye/conf/smartereye.pb.txt`  |       `apollo::drivers::smartereye::config::Config`     |    smartereye config  |

#### How to Launch

```bash
cyber_launch start modules/drivers/smartereye/launch/smartereye.launch
```
