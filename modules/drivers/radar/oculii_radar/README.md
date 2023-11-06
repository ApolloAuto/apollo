# drivers-oculii-radar

## Introduction
This package is responsible for receiving the native 4d radar of oculii input from upd socket and parsing it to send it to the perception-4d-radar-detecetion package for subsequent processing.

## Directory Structure
```shell
modules/drivers/radar/oculii_radar/
├── BUILD
├── common                              // common data structure
├── conf
├── dag
├── launch
├── oculii_radar_component.cc
├── oculii_radar_component.h
├── parser                              // parser implementation of raw 4d radar message
├── proto
└── README.md
```

## Modules

### OculiiRadarComponent

apollo::drivers::radar::OculiiRadarComponent

#### Input

The package will first send commands to the can card to configure the radar according to the configuration file. When the received radar status information is consistent with the user configuration information, only then it starts to parse the data received from the udp socket and send the message.

#### Output

| Name  | Type                                          | Description             |
| ----- | ----------------------------------------------| ----------------------- |
| `msg` |   `apollo::drivers::radar::OculiiPointCloud`  |   4d radar pointcloud   |

#### Topic
**topic name**: /apollo/sensor/oculii/PointCloud2
**channel ID**: CHANNEL_ID_ONE

#### configs

| file path                                                         | type / struct                                  | Description              |
| ----------------------------------------------------------------- | ---------------------------------------------- | ------------------------ |
| `modules/drivers/radar/oculii_radar/conf/oculii_radar_conf.pb.txt`|`apollo::drivers::oculii_radar::OculiiRadarConf`|   oculii radar config    |


#### How to Launch

```bash
cyber_launch start modules/drivers/radar/oculii_radar/launch/oculii_radar.launch
```
