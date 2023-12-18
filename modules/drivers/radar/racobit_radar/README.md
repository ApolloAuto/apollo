# drivers-racobit-radar

## Introduction
This package is responsible for receiving the native conti radar input from can and parsing it to send it to the perception-radar-detecetion package for subsequent processing.

## Directory Structure
```shell
modules/drivers/radar/racobit_radar/
├── BUILD
├── conf
├── dag
├── launch
├── proto
├── protocol                            // parseing protocol
├── racobit_radar_canbus_component.cc
├── racobit_radar_canbus_component.h
├── racobit_radar_message_manager.cc
├── racobit_radar_message_manager.h
├── README_cn.md
└── README.md
```

## Modules

### RacobitRadarCanbusComponent

apollo::drivers::racobit_radar::RacobitRadar


#### Input

The package will first send commands to the can card to configure the radar according to the configuration file. When the received radar status information is consistent with the user configuration information, only then it starts to parse the data received from the can and send the message.

#### Output

| Name  | Type                                           | Description           |
| ----- | -----------------------------------------------| --------------------- |
| `msg` | `apollo::drivers::racobit_radar::RacobitRadar` |   Parsed radar data   |

#### Topic
**topic name**: /apollo/sensor/racobit_radar
**channel ID**: CHANNEL_ID_ONE
**proto file**: [modules/drivers/proto/racobit_radar.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/racobit_radar.proto)

#### configs

| file path                                                            | type / struct                                    | Description              |
| -------------------------------------------------------------------- | ------------------------------------------------ | ------------------------ |
| `modules/drivers/radar/racobit_radar/conf/oracobit_radar_conf.pb.txt`|`apollo::drivers::racobit_radar::RacobitRadarConf`|   racobit radar config                   |



#### How to Launch

```bash
cyber_launch start modules/drivers/radar/racobit_radar/launch/racobit_radar.launch
```
