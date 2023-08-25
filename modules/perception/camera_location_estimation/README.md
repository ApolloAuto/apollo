# Module Name
camera_location_estimation

# Introduction
The current module is able to calculate the rotation angle of the target in the camera coordinate system based on the network predicted 2D target box information, 3D observation angle information, 3D dimension information, and camera internal parameters Then, combined with the obstacle size template, the module solves the target 3dbbox, and converts the coordinates to the world coordinate system.

# Directory Structure
```
├── camera_location_estimation  // camera location estimate module
    ├── conf                    // module configuration files
    ├── dag                     // dag files
    ├── data                    // model params
    ├── interface               // function interface folder
    ├── proto                   // proto files
    ├── transformer             // implementation of position estimation algorithm
    │   ├── multicue
    │   └── ...
    ├── camera_location_estimation_component.cc // component interface
    ├── camera_location_estimation_component.h
    ├── cyberfile.xml           // package management profile
    ├── README.md
    └── BUILD
```

# Module Input and Output
## Input
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `frame`           | `onboard::CameraFrame`          | camera frame message |

## Output
| Name              | Type                            | Description          |
| ----------------- | ------------------------------- | -------------------- |
| `frame`           | `onboard::CameraFrame`          | camera frame message |


# Reference
1. [3D Bounding Box Estimation Using Deep Learning and Geometry](https://arxiv.org/abs/1612.00496)