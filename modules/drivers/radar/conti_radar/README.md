# drivers-conti-radar

## Introduction
This package is responsible for receiving the native conti radar input from can and parsing it to send it to the perception-radar-detecetion package for subsequent processing.

## Directory Structure
```shell
modules/drivers/radar/conti_radar/
├── BUILD
├── conf
├── conti_radar_canbus_component.cc
├── conti_radar_canbus_component.h
├── conti_radar_message_manager.cc
├── conti_radar_message_manager.h
├── dag
├── launch
├── proto
├── protocol                        // parseing protocol
├── README_cn.md
└── README.md
```

## Modules

### ContiRadarCanbusComponent

apollo::drivers::conti_radar::ContiRadarCanbusComponent


#### Input

The package will first send commands to the can card to configure the radar according to the configuration file. When the received radar status information is consistent with the user configuration information, only then it starts to parse the data received from the can and send the message.

#### Output

| Name  | Type                                       | Description           |
| ----- | ------------------------------------------ | --------------------- |
| `msg` | `apollo::drivers::conti_radar::ContiRadar` |   Parsed radar data   |

#### Topic
**topic name**: /apollo/sensor/conti_radar
**channel ID**: CHANNEL_ID_ONE
**proto file**: [modules/drivers/proto/conti_radar.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/conti_radar.proto)

#### configs

| file path                                                         | type / struct                                  | Description              |
| ----------------------------------------------------------------- | ---------------------------------------------- | ------------------------ |
| `modules/drivers/radar/conti_radar/conf/conti_radar_conf.pb.txt`  | `apollo::drivers::conti_radar::ContiRadarConf` |   conti radar config     |


#### How to Launch

```bash
cyber_launch start modules/drivers/radar/conti_radar/launch/conti_radar.launch
```
