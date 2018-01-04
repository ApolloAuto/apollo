# Apollo 2.0 Quick Start Guide

The Apollo 2.0 Quick Start Guide focuses on Apollo 2.0 new features. For general Apollo
concepts, please refer to
[Apollo 1.0 Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md).

## Contents
* [Calibration Guide](#Calibration-Guide)
* [Hardware and Software Installation](#Hardware-and-Software-Installation)
* [Dreamview Usage Table](#Dreamview-Usage-Table)
* [Launch release env Docker Image](#Launch-release-env-Docker-Image)
* [Start Auto](#Start-Auto)

## Calibration Guide

Before doing the following steps, make sure you have calibrated the extrinsic
parameters between the LiDAR and the GNSS/INS. For sensor calibrations, please
refer to
[Apollo 2.0 Sensor Calibration Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide.md).

## Hardware and Software Installation

Please refer to [Apollo 2.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide%20v1.md)
for the steps to install the hardware components and the system software.

## Dreamview Usage Table

For questions regarding Dreamview icons refer to the [Dreamview Usage Table]( https://github.com/ApolloAuto/apollo/blob/master/docs/specs/dreamview_usage_table.md).

## Launch release env Docker Image

Run the following commands:

```bash
cd $APOLLO_HOME
bash docker/scripts/release_start.sh
```

When Docker starts, it creates a port mapping, which maps the Docker internal
port 8888 to the host port 8888. You can then visit the Dreamview web service in
your host machine browser:

Open the Chrome browser and start the Apollo Dreamview by going to
**localhost:8888**.
 ![](images/dreamview.png)

You'll be required to setup profile before doing anything else. Click the
dropdown menu on top right to select your HDMap and vehicle in use. The list are
defined in
[HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/dreamview/conf/hmi.conf).

## Start Auto

In Apollo 2.0, we have released auto driving on simple urban roads. The following guide serves as a user manual for launching the Apollo 2.0 software and hardware stack on vehicle.


- Step 1: Plug-in the hard drive to any available USB port. .
- Step 2: Turn on the vehicle, and then the IPC.
- Step 3: Once the IPC has fully booted, log-in to the machine .
- Step 4: Open a terminal.
- Step 5: Within a terminal window, type: rstart
- Step 6: Open up Dreamview in a web browser.
- Step 7: Select vehicle, and map from among the drop down options in the top right corner of Dreamview.
- Step 8: Under Tasks click Setup.
- Step 9: Under Module Controller confirm all modules are “green”. You may need to drive around a bit to get a good GPS signal.
- Step 10: Under Default Routing select your desired route.
- Step 11: Under Tasks click Start Auto. [The vehicle should now be in autonomous mode]
- Step 12: After autonomous testing is complete, under Tasks click Reset All, close all windows and shutdown the machine. 
- Step 13: Remove the hard drive.


