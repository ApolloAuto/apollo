# drivers-camera

## Introduction
This package is based on the V4L USB camera device implementation package that provides image capture and distribution. This driver includes image capture and compression functions, the captured image is encoded into RGB format and written to the channel, and the compressed image is encoded into jpeg format and written to the channle.

## Directory Structure
```shell
modules/drivers/camera/
├── AUTHORS.md
├── BUILD
├── camera_component.cc
├── camera_component.h
├── CHANGELOG.rst
├── CMakeLists.txt
├── compress_component.cc
├── compress_component.h
├── conf
├── cyberfile.xml
├── dag
├── launch
├── LICENSE
├── mainpage.dox
├── proto
├── README_cn.md
├── README.md
├── usb_cam.cc
├── usb_cam.h
├── util.cc
└── util.h
```

## Modules

### CameraComponent

apollo::drivers::camera::CameraComponent

#### Input

| Name  | Type                             | Description         |
| ----- | -------------------------------- | ------------------- |
| buffer| camera image from v4l2 interface |          -          |

#### Output

| Name  | Type                               | Description           |
| ----- | ---------------------------------- | --------------------- |
| `msg` | `apollo::drivers::Image`           |  camera sensor image  |


#### configs

| file path                                     | type / struct                           | Description          |
| --------------------------------------------- | --------------------------------------- | -------------------- |
| `modules/drivers/camera/camera_front_6mm.pb`  | `apollo::drivers::camera::conf::Config` |   compress config    |

#### How to Launch

```bash
cyber_launch start modules/drivers/camera/launch/camera.launch
```

### CompressComponent

apollo::drivers::camera::CompressComponent

#### Input

| Name  | Type                             | Description         |
| ----- | -------------------------------- | ------------------- |
| `msg` | `apollo::drivers::Image`         | camera sensor image |

#### Output

| Name  | Type                               | Description           |
| ----- | ---------------------------------- | --------------------- |
| `msg` | `apollo::drivers::CompressedImage` | compress sensor image |


#### configs

| file path                                     | type / struct                                   | Description          |
| --------------------------------------------- | ----------------------------------------------- | -------------------- |
| `modules/drivers/camera/camera_front_6mm.pb`  | `apollo::drivers::camera::conf::CompressConfig` |   compress config    |

#### How to Launch

```bash
cyber_launch start modules/drivers/camera/launch/camera.launch

## License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

## Non Apollo contributors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
