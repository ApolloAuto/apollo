# Apollo 1.0 Quick Start Guide
## Contents
* [About This Guide](#about-this-guide)
    * [Document Conventions](#document-conventions)
* [Overview of Apollo](#overview-of-apollo)
* [Description of the Vehicle Environment](#description-of-the-vehicle-environment)
* [Hardware Installation](#hardware-installation)
* [Apollo Software Installation](#apollo-software-installation)
* [Run the Demo on Vehicle](#run-the-demo-on-vehicle)
    * [Launch the Local Release Docker Image](#launch-the-local-release-env-docker-image)
    * [Record the Driving Trajectory](#record-driving-trajectory)
    * [Perform Autonomous Driving](#perform-autonomous-driving)
    * [Shut Down](#shut-down)
* [Run Offline Demo](#run-offline-demo)

# About This Guide

The _Apollo 1.0 Quick Start Guide_ provides all of the basic instructions to understand, install, and build Apollo.

## Document Conventions

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

# About Apollo 1.0

Apollo 1.0, also referred to as the _Automatic GPS Waypoint Following_, works in an enclosed venue such as a test track or parking lot. It accurately replays a trajectory and the speed of that trajectory that a human driver has traveled in an enclosed, flat area on solid ground.

At this stage of development, Apollo 1.0 **cannot** perceive obstacles in close proximity, drive on public roads, or drive in areas without GPS signals.

# Description of the Vehicle Environment

The Lincoln MKZ, enhanced by Autonomous Stuff, provides users with an accessible autonomous vehicle platform. The platform supplies users with a comprehensive stack of hardware and software solutions.

Users gain direct access to vehicle controls such as gear selection, speed, and indicator lights. Software interfaces have been created for steering, braking, acceleration, and gear selection to provide Developers with a workable user interface.

Additional features include:

- Power distributor terminals
- Integrated PC with ROS pre-installed and configured
- Emergency Stop using a drive-by-wire system
- Ethernet network and USB connections (to PC)

# Hardware Installation

Please refer to [Apollo 1.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md)
for the steps to install the hardware components and the system software.

# Apollo Software Installation

Please refer to [Apollo Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md).

# Run Demo on Vehicle

This section provides the instructions to run the Apollo 1.0 Demo on Vehicle.
1. Set up the hardware:

- Power-on the platform vehicle.
- Power-on the Industrial PC (IPC).
    ![](images/ipc_power_on.png)
- Power-on the modem by pressing and holding the power button until the lights turn on.
- Set up the network configuration for the IPC: static IP (for example, 192.168.10.6), subnet mask (for example, 255.255.255.0), and gateway (for example, 192.168.10.1)
   - Configurate your DNS server IP (for example, 8.8.8.8).
   - Use a tablet to access **Settings** and connect to MKZ wifi:

   ![](images/ipad_config_wifi.png)

2. Start the HMI in Docker **using Chrome only**:

   ![warning](images/warning_icon.png)**Warning:** Make sure that you are not starting HMI from two Docker containers concurrently.

## Launch the Local release env Docker Image

Run the following commands:
```
cd $APOLLO_HOME
bash docker/scripts/release_start.sh local_release
```
When Docker starts, it creates a port mapping, which maps the Docker internal port 8887 to the host port 8887. You can then visit the HMI web service in your host machine browser:

Open the Chrome browser and start the Apollo HMI by going to **192.168.10.6:8887**.
 ![](images/start_hmi.png)

## Record the Driving Trajectory

Follow these steps to record the driving trajectory:

1. In the Apollo HMI, under Quick Record, click **Setup** to start all modules and perform the hardware health check.
   ![](images/hmi_record_setup.png)
2. If the hardware health check passes, click the **Start** button to start to record the driver trajectory.
   ![](images/hmi_record_start.png)
3. After arriving at a destination, click the **Stop** button to stop recording.
   ![](images/hmi_record_stop.png)
4. If you want to record a *different* trajectory, click the **New** button to initiate recording again.
   ![](images/hmi_record_reset.png)

## Perform Autonomous Driving

Follow these steps to perform autonomous driving:

1. In the Apollo HMI, under Quick Play, click **Setup** to start all modules and perform a hardware health check.
   ![](images/hmi_play_setup.png)

2. If the vehicle successfully passes the Setup step, it is ready to enter the Autonomous mode. **MAKE SURE DRIVER IS READY!**  Click the **Start** button to start the autonomous driving.

   ![](images/hmi_play_start.png)

3. After arriving at your destination,  click the **Stop** button to stop replaying the recorded trajectory.
   ![](images/hmi_play_stop.png)

## Shut Down

1. Shut down the system from a terminal:
    ```sudo shutdown now```

2. Power-off the IPC (locate the icon on the top right of the desktop to click **Shut Down**).

3. Turn off the modem by pressing and holding the power button until the lights turn off.

4. Turn off the car.

# Run Offline Demo
See [Offline Demo Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/README.md)
