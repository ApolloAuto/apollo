# Apollo 3.0 Hardware and System Installation Guide

- [Apollo 3.0 Hardware and System Installation Guide](#apollo-30-hardware-and-system-installation-guide)
  - [About This Guide](#about-this-guide)
    - [Document Conventions](#document-conventions)
  - [Introduction](#introduction)
    - [Documentation](#documentation)
  - [Key Hardware Components](#key-hardware-components)
    - [Additional Components Required](#additional-components-required)
  - [Steps for the Installation Tasks](#steps-for-the-installation-tasks)
    - [At the Office](#at-the-office)
    - [In the Vehicle](#in-the-vehicle)
      - [Prerequisites](#prerequisites)
      - [Diagrams of the Major Component Installations](#diagrams-of-the-major-component-installations)
  - [Additional Tasks Required](#additional-tasks-required)
  - [Time Sync Script Setup [Optional]](#time-sync-script-setup-optional)
  - [Next Steps](#next-steps)

## About This Guide

The *Apollo 3.0 Hardware and System Installation Guide* provides the instructions to install all of the hardware components and system software for the **Apollo Project**. The system installation information included pertains to the procedures to download and install the Apollo Linux Kernel.

### Document Conventions

The following table lists the conventions that are used in this document:

| **Icon**                            | **Description**                          |
| ----------------------------------- | ---------------------------------------- |
| **Bold**                            | Emphasis                                 |
| `Mono-space font`                   | Code, typed data                         |
| _Italic_                            | Titles of documents, sections, and headings Terms used |
| ![info](images/info_icon.png)       | **Info**  Contains information that might be useful.  Ignoring the Info icon has no negative consequences. |
| ![tip](images/tip_icon.png)         | **Tip**. Includes helpful hints or a shortcut that might assist you in completing a task. |
| ![online](images/online_icon.png)   | **Online**. Provides a link to a particular web site where you can get more information. |
| ![warning](images/warning_icon.png) | **Warning**. Contains information that must **not** be ignored or you risk failure when you perform a certain task or step. |

## Introduction

The **Apollo Project** is an initiative that provides an open, complete, and reliable software platform for Apollo partners in the automotive and autonomous driving industries. The aim of this project is to enable these entities to develop their own self-driving systems based on the Apollo software stack.

### Documentation

The following set of documentation describes Apollo 3.0:

- ***<u>[Apollo Hardware and System Installation Guide]</u>***  ─ Links to the Hardware Development Platform Documentation in Specs

  - **Vehicle**:

    - [Industrial PC (IPC)](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/IPC/Nuvo-6108GC_Installation_Guide.md)
    - [Global Positioning System (GPS)](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Navigation/README.md)
    - [Inertial Measurement Unit (IMU)](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Navigation/README.md)
    - Controller Area Network (CAN) card
    - GPS Antenna
    - GPS Receiver
    - [Light Detection and Ranging System (LiDAR)](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Lidar/README.md)
    - [Camera](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Camera/README.md)
    - [Radar](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Radar/README.md)
    - [Apollo Sensor Unit (ASU)](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Apollo_Sensor_Unit/Apollo_Sensor_Unit_Installation_Guide.md)

  - **Software**: Refer to the [Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/r3.0.0/docs/specs/Software_and_Kernel_Installation_guide.md) for information on the following:
    - Ubuntu Linux
    - Apollo Linux Kernel
    - NVIDIA GPU Driver

- ***<u>[Apollo Quick Start Guide]</u>*** ─ The combination of a tutorial and a roadmap that provides the complete set of end-to-end instructions. The Quick Start Guide also provides links to additional documents that describe the conversion of a regular car to an autonomous-driving vehicle.

## Key Hardware Components

The key hardware components to install include:

- Onboard computer system ─ Neousys Nuvo-6108GC
- Controller Area Network (CAN) Card ─ ESD CAN-PCIe/402-B4
- Global Positioning System (GPS) and Inertial Measurement Unit (IMU) ─
  You can select one of the following options:
  - NovAtel SPAN-IGM-A1
  - NovAtel SPAN® ProPak6™ and NovAtel IMU-IGM-A1
  - Navtech NV-GI120
- Light Detection and Ranging System (LiDAR) ─  You can select one of the following options:
  - Velodyne HDL-64E S3
  - Velodyne Puck series
  - Innovusion LiDAR
  - Hesai's Pandora
- Cameras —  You can select one of the following options:
  - Leopard Imaging LI-USB30-AR023ZWDR with USB 3.0 case
  - Argus Camera
  - Wissen Camera
- Radar —  You can select one of the following options:
  - Continental ARS408-21
  - Delphi ESR 2.5
  - Racobit B01HC

### Additional Components Required

You need to provide these additional components for the Additional Tasks Required:

- Apollo Sensor Unit (ASU)
- A 4G router for Internet access
- A USB hub for extra USB ports
- A monitor, keyboard, and mouse for debugging at the car onsite
- Cables: a Digital Visual Interface (DVI) cable (optional), a customized cable for GPS-LiDAR time synchronization
- Apple iPad Pro: 9.7-inch, Wi-Fi (optional)

The features of the key hardware components are presented in the subsequent sections.

## Steps for the Installation Tasks

This section describes the steps to install:

- The key hardware and software components
- The hardware in the vehicle

### At the Office

Perform the following tasks:

- Prepare the IPC:
  - Install the CAN card
  - Install or replace the hard drive
  - Prepare the IPC for powering up

- Install the software for the IPC:
  - Ubuntu Linux
  - Apollo Kernel
  - Nvidia GPU Driver

The IPC is now ready to be mounted on the vehicle.

### In the Vehicle

Perform these tasks:

- Make the necessary modifications to the vehicle as specified in the list of prerequisites
- Install the major components:
  - GPS Antenna
  - IPC
  - GPS Receiver and IMU
  - LiDAR's
  - Cameras
  - Radar

#### Prerequisites

**![warning_icon](images/warning_icon.png)WARNING**: Prior to mounting the major components (GPS Antenna, IPC, and GPS Receiver) in the vehicle, perform certain modifications as specified in the list of prerequisites. The instructions for making the mandatory changes in the list are outside the scope of this document.

The list of prerequisites are as follows:

- The vehicle must be modified for “drive-by-wire” technology by a professional service company. Also, a CAN interface hookup must be provided in the trunk where the IPC will be mounted.
- A power panel must be installed in the trunk to provide power to the IPC and the GPS-IMU. The power panel would also service other devices in the vehicle such as a 4G LTE router. The power panel should be hooked up to the power system in the vehicle.
- A custom-made rack must be installed to mount the GPS-IMU Antenna, the cameras and the LiDAR's on top of the vehicle.
- A custom-made rack must be installed to mount the GPS-IMU in the trunk.
- A custom-made rack must be installed in front of the vehicle to mount the front-facing radar.
- A 4G LTE router must be mounted in the trunk to provide Internet access for the IPC. The router must have built-in Wi-Fi access point (AP) capability to connect to other devices, such as an iPad, to interface with the autonomous driving (AD) system. A user would be able to use the mobile device to start AD mode or monitor AD status, for example.

#### Diagrams of the Major Component Installations

The following two diagrams indicate the locations of where the three major components (GPS Antenna, IPC, GPS Receiver and LiDAR) should be installed on the vehicle:

![major_component_side_view](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Hardware_overview.png)

![major_component_rear_view](images/Car_Rearview.png)

## Additional Tasks Required

Use the components that you were required to provide to perform the following tasks:

1. Connect a monitor using the DVI or the HDMI cables and connect the keyboard and mouse to perform debugging tasks at the car onsite.

2. Establish a Wi-Fi connection on the Apple iPad Pro to access the HMI and control the Apollo ADS that is running on the IPC.

## Time Sync Script Setup [Optional]

In order to, sync the computer time to the NTP server on the internet, you could use the [Time Sync script](https://github.com/ApolloAuto/apollo/blob/master/scripts/time_sync.sh)

## Next Steps

After you complete the hardware installation in the vehicle, see the [Apollo Quick Start](../../../02_Quick%20Start/apollo_3_0_quick_start.md) for the steps to complete the software installation.
