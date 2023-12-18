# task-manager

## Introduction
  Task manager generates a series of tasks, including path planning, obstacle avoidance, traffic rule compliance, etc., based on the input map data and routing instructions (start and end positions). These tasks will be scheduled and executed in the form of Cyber concatenation to ensure that the vehicle is able to make correct driving decisions based on the real-time traffic conditions while traveling.


## Directory Structure
```shell
modules/task_manager
├── BUILD
├── common
├── conf
├── cyberfile.xml
├── cycle_routing_manager.cc
├── cycle_routing_manager.h
├── dag
├── parking_routing_manager.cc
├── parking_routing_manager.h
├── proto
├── README.md
├── task_manager_component.cc
└── task_manager_component.h
```
 
#### Input

| Name  | Type                                                 | Description         |
| ----- | ---------------------------------------------------- | ------------------- |
| `msg` | `apollo::task_manager::Task`                         | task                |
| `msg` | `apollo::localization::LocalizationEstimate`         | localization        |
| `msg` | `apollo::planning::PlanningCommand`                  | planning command    |
| `msg` | `planning::ADCTrajectory`                            | planning trajectory |


#### Output

| Name  | Type                                                 | Description         |
| ----- | ---------------------------------------------------- | ------------------- |
| `msg` | `apollo::external_command::LaneFollowCommand`        | lane follow command |


## configs

| file path                                              | type / struct                                 | Description           |
| ------------------------------------------------------ | --------------------------------------------- | --------------------- |
| `modules/task_manager/conf/task_manager_config.pb.txt` | `apollo::task_manager::TaskManagerConfig`     | task manager config   |
| `modules/task_manager/conf/task_manager.conf`          | `gflags`                                      |   gflags config       |

## Flags

| flagfile                                            | type | Description                     |
| --------------------------------------------------- | ---- | ------------------------------- |
| `modules/task_manager/common/task_manager_gflags.cc`| `cc` | task manager flags define       |
| `modules/task_manager/common/task_manager_gflags.h` | `h`  | task manager flags header       |

#### How to Launch

```bash
cyber_launch start modules/task_manager/launch/task_manager.launch
```