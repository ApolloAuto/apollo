# drivers-conti-radar

## Introduction
This package is responsible for receiving the native radar input and parsing it to send it to the perception package for subsequent processing.

## Directory Structure
```shell
modules/drivers/radar/
├── conti_radar/
├── oculii_radar/
├── racobit_radar/
├── ultrasonic_radar/
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
