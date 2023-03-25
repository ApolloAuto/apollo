![](docs/02_Quick%20Start/demo_guide/images/Apollo_logo.png)

[![Build Status](http://180.76.142.62:8111/app/rest/builds/buildType:Apollo_Build/statusIcon)](http://180.76.142.62:8111/viewType.html?buildTypeId=Apollo_Build&guest=1)
[![Simulation Status](https://azure.apollo.auto/dailybuildstatus.svg)](https://azure.apollo.auto/daily-build/public)

```

We choose to go to the moon in this decade and do the other things,

not because they are easy, but because they are hard.

-- John F. Kennedy, 1962

```

Welcome to Apollo's GitHub page!

[Apollo](http://apollo.auto) is a high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.

For business and partnership, please visit [our website](http://apollo.auto).

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Individual Versions](#individual-versions)
4. [Architecture](#architecture)
5. [Installation](#installation)
6. [Quick Starts](#quick-starts)
7. [Documents](#documents)

## Introduction

Apollo is loaded with new modules and features but needs to be calibrated and configured perfectly before you take it for a spin. Please review the prerequisites and installation steps in detail to ensure that you are well equipped to build and launch Apollo. You could also check out Apollo's architecture overview for a greater understanding of Apollo's core technology and platforms.

## Prerequisites

**[New 2021-01]** The Apollo platform (stable version) is now upgraded with
software packages and library dependencies of newer versions including:

1. CUDA upgraded to version 11.1 to support Nvidia Ampere (30x0 series) GPUs,
   with NVIDIA driver >= 455.32
2. LibTorch (both CPU and GPU version) bumped to version 1.7.0 accordingly.

We do not expect a disruption to your current work, but to ease your life of
migration, you would need to:

1. Update NVIDIA driver on your host to version >= 455.32.
  ([Web link](https://www.nvidia.com/Download/index.aspx?lang=en-us))
2. Pull latest code and run the following commands after restarting and
  logging into Apollo Development container:

```bash
# Remove Bazel output of previous builds
rm -rf /apollo/.cache/{bazel,build,repos}
# Re-configure bazelrc.
./apollo.sh config --noninteractive
```

---


* The vehicle equipped with the by-wire system, including but not limited to brake-by-wire, steering-by-wire, throttle-by-wire and shift-by-wire (Apollo is currently tested on Lincoln MKZ)

* A machine with a 8-core processor and 16GB memory minimum

* NVIDIA Turing GPU / AMD GFX9/RDNA/CDNA GPU is strongly recommended

* Ubuntu 18.04

* NVIDIA driver version 455.32.00 and above ([Web link](https://www.nvidia.com/Download/index.aspx?lang=en-us)) or [ROCm v5.1](https://docs.amd.com/bundle/ROCm-Installation-Guide-v5.1/page/Prerequisite_Actions.html) and above.

* Docker-CE version 19.03 and above ([Official doc](https://docs.docker.com/engine/install/ubuntu/))

* NVIDIA Container Toolkit ([Official doc](https://github.com/NVIDIA/nvidia-docker))

**Please note**, it is recommended that you install the versions of Apollo in the following order: **1.0 -> whichever version you would like to test out**. The reason behind this recommendation is that you need to confirm whether individual hardware components and modules are functioning correctly, and clear various version test cases before progressing to a higher and more capable version for your safety and the safety of those around you.

## Individual Versions:

The following diagram highlights the scope and features of each Apollo release:

![](docs/02_Quick%20Start/demo_guide/images/Apollo_Roadmap_8_0.png)

[**Apollo 1.0:**](docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_1_0_hardware_system_installation_guide.md)

Apollo 1.0, also referred to as the Automatic GPS Waypoint Following, works in an enclosed venue such as a test track or parking lot. This installation is necessary to ensure that Apollo works perfectly with your vehicle. The diagram below lists the various modules in Apollo 1.0.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_1.png)

[**Apollo 1.5:**](docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_1_5_hardware_system_installation_guide.md)

Apollo 1.5 is meant for fixed lane cruising. With the addition of LiDAR, vehicles with this version now have better perception of its surroundings and can better map its current position and plan its trajectory for safer maneuvering on its lane. Please note, the modules highlighted in Yellow are additions or upgrades for version 1.5.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_1_5.png)

[**Apollo 2.0:**](docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_2_0_hardware_system_installation_guide_v1.md#key-hardware-components)

Apollo 2.0 supports vehicles autonomously driving on simple urban roads. Vehicles are able to cruise on roads safely, avoid collisions with obstacles, stop at traffic lights, and change lanes if needed to reach their destination.  Please note, the modules highlighted in Red are additions or upgrades for version 2.0.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_2.png)

[**Apollo 2.5:**](docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_2_5_hardware_system_installation_guide_v1.md)

Apollo 2.5 allows the vehicle to autonomously run on geo-fenced highways with a camera for obstacle detection. Vehicles are able to maintain lane control, cruise and avoid collisions with vehicles ahead of them.

```
Please note, if you need to test Apollo 2.5; for safety purposes, please seek the help of the
Apollo Engineering team. Your safety is our #1 priority,
and we want to ensure Apollo 2.5 was integrated correctly with your vehicle before you hit the road.
```

![](docs/02_Quick%20Start/demo_guide/images/Apollo_2_5.png)

[**Apollo 3.0:**](docs/02_Quick%20Start/apollo_3_0_quick_start.md)

Apollo 3.0's primary focus is to provide a platform for developers to build upon in a closed venue low-speed environment. Vehicles are able to maintain lane control, cruise and avoid collisions with vehicles ahead of them.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_3.0_diagram.png)

[**Apollo 3.5:**](docs/02_Quick%20Start/apollo_3_5_quick_start.md)

Apollo 3.5 is capable of navigating through complex driving scenarios such as residential and downtown areas. The car now has 360-degree visibility, along with upgraded perception algorithms to handle the changing conditions of urban roads, making the car more secure and aware. Scenario-based planning can navigate through complex scenarios, including unprotected turns and narrow streets often found in residential areas and roads with stop signs.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_3_5_Architecture.png)

[**Apollo 5.0:**](docs/02_Quick%20Start/apollo_3_5_quick_start.md)

Apollo 5.0 is an effort to support volume production for Geo-Fenced Autonomous Driving.
The car now has 360-degree visibility, along with upgraded perception deep learning model to handle the changing conditions of complex road scenarios, making the car more secure and aware. Scenario-based planning has been enhanced to support additional scenarios like pull over and crossing bare intersections.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_5_0_diagram1.png)

[**Apollo 5.5:**](docs/02_Quick%20Start/apollo_5_5_quick_start.md)

Apollo 5.5 enhances the complex urban road autonomous driving capabilities of previous Apollo releases, by introducing curb-to-curb driving support. With this new addition, Apollo is now a leap closer to fully autonomous urban road driving. The car has complete 360-degree visibility, along with upgraded perception deep learning model and a brand new prediction model to handle the changing conditions of complex road and junction scenarios, making the car more secure and aware.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_5_5_Architecture.png)

[**Apollo 6.0:**](docs/02_Quick%20Start/apollo_6_0_quick_start.md)

Apollo 6.0 incorporates new deep learning models to enhance the capabilities for certain Apollo modules. This version works seamlessly with new additions of data pipeline services to better serve Apollo developers. Apollo 6.0 is also the first version to integrate certain features as a demonstration of our continuous exploration and experimentation efforts towards driverless technology.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_6_0.png)

**Apollo 7.0:**

Apollo 7.0 incorporates 3 brand new deep learning models to enhance the capabilities for Apollo Perception and Prediction modules. Apollo Studio is introduced in this version, combining with Data Pipeline, to provide a one-stop online development platform to better serve Apollo developers. Apollo 7.0 also publishes the PnC reinforcement learning model training and simulation evaluation service based on previous simulation service.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_7_0.png)

[**Apollo 8.0:**](docs/02_Quick%20Start/apollo_8_0_quick_start.md)

Apollo 8.0 is an effort to provide an extensible software framework and complete development cycle for Autonomous Driving developer. Apollo 8.0 introduces easily-reused “Package” to organize software modules.  Apollo 8.0 integrates the whole process of perception development ,by combining model training service, model deployment tool and end-to-end visual validation tool . And another 3 new deep learning models are incorporated in Apollo 8.0 for perception module. Simulation service is upgraded by integrating local simulator in Dreamview to provide powerful debug tool for PnC developer.

![](docs/02_Quick%20Start/demo_guide/images/Apollo_8_0.png)

## Architecture

* **Hardware/ Vehicle Overview**

![](docs/02_Quick%20Start/demo_guide/images/Hardware_overview_3_5.png)

* **Hardware Connection Overview**

![](docs/02_Quick%20Start/demo_guide/images/Hardware_connection_3_5_1.png)

* **Software Overview**

![](docs/02_Quick%20Start/demo_guide/images/Apollo_3_5_software_architecture.png)

## Installation

* [Hardware installation guide](docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_3_5_hardware_system_installation_guide.md)
* [Software installation guide](docs/01_Installation%20Instructions/apollo_software_installation_guide_package_method.md) - **This step is required**
* [Launch and run Apollo](docs/03_Package%20Management/launch_and_run_apollo_package_method.md)

Congratulations! You have successfully built out Apollo without Hardware. If you do have a vehicle and hardware setup for a particular version, please pick the Quickstart guide most relevant to your setup:

## Quick Starts:

* [Apollo 8.0 QuickStart Guide](docs/02_Quick%20Start/apollo_8_0_quick_start.md)

* [Apollo 6.0 QuickStart Guide](docs/02_Quick%20Start/apollo_6_0_quick_start.md)

* [Apollo 5.5 QuickStart Guide](docs/02_Quick%20Start/apollo_5_5_quick_start.md)

* [Apollo 5.0 QuickStart Guide](docs/02_Quick%20Start/apollo_5_0_quick_start.md)

* [Apollo 3.5 QuickStart Guide](docs/02_Quick%20Start/apollo_3_5_quick_start.md)

* [Apollo 3.0 QuickStart Guide](docs/02_Quick%20Start/apollo_3_0_quick_start.md)

* [Apollo 2.5 QuickStart Guide](docs/02_Quick%20Start/apollo_2_5_quick_start.md)

* [Apollo 2.0 QuickStart Guide](docs/02_Quick%20Start/apollo_2_0_quick_start.md)

* [Apollo 1.5 QuickStart Guide](docs/02_Quick%20Start/apollo_1_5_quick_start.md)

* [Apollo 1.0 QuickStart Guide](docs/02_Quick%20Start/apollo_1_0_quick_start.md)

## Documents

* [Installation Instructions](docs/01_Installation%20Instructions/)

* [Quick Start](docs/02_Quick%20Start/)

* [Package Management](docs/03_Package%20Management/)

* [CyberRT](docs/04_CyberRT/)

* [Localization](docs/05_Localization/)

* [Perception](docs/06_Perception/)

* [Prediction](docs/07_Prediction/)

* [Planning](docs/08_Planning/)

* [Decider](docs/09_Decider/)

* [Control](docs/10_Control/)

* [Hardware Integration and Calibration](docs/11_Hardware%20Integration%20and%20Calibration/)

* [Map acquisition](docs/12_Map%20acquisition/)

* [Apollo Tool](docs/13_Apollo%20Tool/)

* [Others](docs/14_Others/)

* [FAQs](docs/15_FAQS/README.md)

## Questions

You are welcome to submit questions and bug reports as [GitHub Issues](https://github.com/ApolloAuto/apollo/issues).

## Copyright and License

Apollo is provided under the [Apache-2.0 license](https://github.com/ApolloAuto/apollo/blob/master/LICENSE).

## Disclaimer

Apollo open source platform only has the source code for models, algorithms and processes, which will be integrated with cybersecurity defense strategy in the deployment for commercialization and productization.

Please refer to the Disclaimer of Apollo in [Apollo's official website](https://developer.apollo.auto/docs/disclaimer.html).

## Connect with us
* [Have suggestions for our GitHub page?](https://github.com/ApolloAuto/apollo/issues)
* [Twitter](https://twitter.com/apolloplatform)
* [YouTube](https://www.youtube.com/channel/UC8wR_NX_NShUTSSqIaEUY9Q)
* [Blog](https://www.medium.com/apollo-auto)
* [Newsletter](http://eepurl.com/c-mLSz)
* Interested in our turnKey solutions or partnering with us Mail us at: apollopartner@baidu.com
