# drivers-gnss

## Introduction
The GNSS driver is mainly responsible for receiving and processing GNSS signals and passing them to the positioning module to realize high-precision positioning.

## Directory Structure
```shell
modules/drivers/gnss/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── gnss_component.cc
├── gnss_component.h
├── launch
├── parser              // parser to parse different kinds of gnss data
├── proto
├── README.md
├── stream              // stream handler to process gnss data from different data streams
├── test
├── test_data
└── util                // common function
```

## Modules

### GnssDriverComponent

apollo::drivers::gnss::GnssDriverComponent

#### Input

|     Name    | Type                             | Description         |
| ----------- | -------------------------------- | ------------------- |
| data stream |   binary data from gnss stream   |          -          |
|    `msg`    |    `apollo::canbus::Chassis`     |  chassis message    |


#### Output

| Name  | Type                                    | Description           |
| ----- | --------------------------------------- | --------------------- |
| `msg` | `apollo::drivers::gnss::RawData`        |    gps bin raw data   |
| `msg` | `apollo::drivers::gnss::RawData`        |   gps rtcm raw data   |
| `msg` | `apollo::drivers::gnss::RawData`        |  gnss stream raw data |
| `msg` | `apollo::drivers::gnss::StreamStatus`   |  gnss stream status   |

#### configs

| file path                                | type / struct                         | Description          |
| ---------------------------------------- | ------------------------------------- | -------------------- |
| `modules/drivers/gnss/gnss_conf.pb.txt`  | `apollo::drivers::gnss::conf::Config` |      gnss config     |


#### How to Launch

```bash
cyber_launch start modules/drivers/gnss/launch/gnss.launch
```