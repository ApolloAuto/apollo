# Localization

## Introduction
  This module provides localization services. There are two ways in which localization is provided:
  -  The RTK (Real Time Kinematic) based method which incorporates GPS and IMU (Inertial Measurement Unit) information
  - The  multi-sensor fusion method which incorporates GPS, IMU, and LiDAR information.

## Input
  In the provided RTK localization method, there are two inputs:
  - GPS - The Global Positioning System
  - IMU - Inertial Measurement Unit

  In the  multi-sensor fusion localization method, there are three inputs:
  - GPS - The Global Positioning System
  - IMU - Inertial Measurement Unit
  - LiDAR - Light Detection And Ranging Sensor

  For more information, refer to

  Guowei Wan, Xiaolong Yang, Renlan Cai, Hao Li, Yao Zhou, Hao Wang, Shiyu Song. "Robust and Precise Vehicle Localization Based on Multi-Sensor Fusion in Diverse City Scenes," 2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, 2018, pp. 4670-4677.
  doi: 10.1109/ICRA.2018.8461224. [link](https://ieeexplore.ieee.org/document/8461224)

## Output
An object instance defined by Protobuf message `LocalizationEstimate`, which can be found in file `localization/proto/localization.proto`.

## Implementing Localization
  Currently the RTK based localization is implemented in class `RTKLocalization`. If you are implementing a new localization method, for example, `FooLocalization`, you would need to perform the following steps:

  1. In `proto/localization_config.proto`, add a value `FOO` to the `LocalizationType` enum type

  2. Go to the `modules/localization` directory, and create a `foo` directory. In the `foo` directory, implement the class `FooLocalization` by following the code in the `RTKLocalization` class in the `rtk` directory. `FooLocalization` has to be a subclass of `LocalizationBase`. Also create a file `foo/BUILD` by following the file `rtk/BUILD`

  3. You need to register the `FooLocalization` class in the function `Localization::RegisterLocalizationMethods()`, which is located in the `localization.cc` file. You can register it by inserting the following code at the end of the function:

  ```
   localization_factory_.Register(LocalizationConfig::FOO, []()->LocalizationBase* { return new FooLocalization(); });
  ```

  4. Make sure your code compiles by including the header files that defines class `FooLocalization`

  1. Now you can go back to the `apollo` root directory and build your code with command `bash apollo.sh build`
