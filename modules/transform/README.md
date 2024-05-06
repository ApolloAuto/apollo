# transform

## Introduction
  The transform module is used to read the transform configuration file to publish static transform relationships. This module can publish different static coordinate transformations by defining different configurations.


## Directory Structure
```shell
modules/transform
├── buffer.cc
├── buffer.h
├── buffer_interface.h
├── BUILD
├── conf
├── cyberfile.xml
├── dag
├── launch
├── proto
├── README.md
├── static_transform_component.cc
├── static_transform_component.h
├── static_transform_component_test.cc
├── transform_broadcaster.cc
└── transform_broadcaster.h
```

#### Input

None 

#### Output

| Name  | Type                                  | Description         |
| ----- | ------------------------------------- | ------------------- |
| `msg` | `apollo::transform::Transform`        |        tf msg       |


## configs

| file path                                              | type / struct                        | Description           |
| ------------------------------------------------------ | ------------------------------------ | --------------------- |
|  `modules/transform/conf/static_transform_conf.pb.txt` | `apollo::static_transform::Conf`     |       tf config       |

#### How to Launch

```bash
cyber_launch start modules/transform/launch/static_transform.launch
```