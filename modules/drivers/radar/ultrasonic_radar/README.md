# drivers-ultrasonic-radar

## Introduction
This package is responsible for receiving the native conti radar input from can and parsing it to send it to the perception-radar-detecetion package for subsequent processing.

## Directory Structure
```shell
modules/drivers/radar/ultrasonic_radar/
├── BUILD
├── conf
├── dag
├── launch
├── proto
├── README_cn.md
├── README.md
├── ultrasonic_radar_canbus.cc
├── ultrasonic_radar_canbus_component.cc
├── ultrasonic_radar_canbus_component.h
├── ultrasonic_radar_canbus.h
├── ultrasonic_radar_message_manager.cc
└── ultrasonic_radar_message_manager.h
```

## Modules

### UltrasonicRadarCanbusComponent

apollo::drivers::ultrasonic_radar::UltrasonicRadarCanbusComponent


#### Input

The package will first send commands to the can card to configure the radar according to the configuration file. When the received radar status information is consistent with the user configuration information, only then it starts to parse the data received from the can and send the message.

#### Output

| Name  | Type                                            | Description           |
| ----- | ----------------------------------------------- | --------------------- |
| `msg` | `apollo::drivers::ultrasonic_radar::Ultrasonic` |   Parsed radar data   |

#### configs

| file path                                                                    | type / struct                                          | Description                |
| ---------------------------------------------------------------------------- | ------------------------------------------------------ | -------------------------- |
|  `modules/drivers/radar/ultrasonic_radar/conf/ultrasonic_radar_conf.pb.txt`  |`apollo::drivers::ultrasonic_radar::UltrasonicRadarConf`|   ultrasonic radar config    |

#### Topic
**topic name**: /apollo/sensor/ultrasonic_radar
**channel ID**: CHANNEL_ID_ONE

#### How to Launch

```bash
cyber_launch start modules/drivers/radar/ultrasonic_radar/launch/ultrasonic_radar.launch
```
