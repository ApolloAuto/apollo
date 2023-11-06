# v2x

## Introduction
The Guardian module in Apollo is a safety module that monitors the status of the autonomous driving system. When the module fails, it actively cuts off the control command output and triggers the brakes. This module is a bit like a fuse, with a fallback mechanism. It is triggered by two main conditions: the interval between messages reporting the module status exceeds kSecondsTillTimeout (2.5 seconds), or there is a safety_mode_trigger_time field in the reported status message. In safety mode, the vehicle stops immediately if the emergency brake is detected or an obstacle is detected by the ultrasound. In addition, the Guardian module is a timing module with a frequency of 10ms, so it will increase the delay of the control command by a maximum of 10ms.

## Directory Structure
```shell
modules/guardian/
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── guardian_component.cc
├── guardian_component.h
├── launch
├── proto
└── README.md
```

#### Input

| Name  | Type                                      | Description         |
| ----- | ----------------------------------------- | ------------------- |
| `msg` | `apollo::canbus::Chassis`                 |  chassis msg        |
| `msg` | `apollo::control::ControlCommand`         | ControlCommand msg  |
| `msg` | `apollo::monitor::SystemStatus`           |  SystemStatus msg   |

#### Output

| Name  | Type                                       | Description         |
| ----- | ------------------------------------------ | ------------------- |
| `msg` | `apollo::guardian::GuardianCommand`        | GuardianCommand msg |


## configs

| file path                                              | type / struct                    | Description   |
| ------------------------------------------------------ | -------------------------------- | ------------- |
|  `modules/guardian/conf/guardian_conf.pb.txt`          | `apollo::guardian::GuardianConf` |       -       |

#### How to Launch

```bash
cyber_launch start modules/guardian/launch/guardian.launch
```