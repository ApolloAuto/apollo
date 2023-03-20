# Apollo 5.0 Quick Start Guide

The following guide serves as a user manual for launching the Apollo upgraded software and hardware stack on vehicle.

This Quick Start Guide focuses on the new features. For general Apollo concepts, please refer to
[Apollo 3.5 Quick Start](../02_Quick%20Start/apollo_3_5_quick_start.md)

## Contents

- [Calibration Guide](#calibration-guide)
- [Hardware and Software Installation](#hardware-and-software-installation)
- [Dreamview Usage Table](#dreamview-usage-table)
- [Onboard Test](#onboard-test)

## Calibration Guide

Apollo currently offers a robust calibration service to support your calibration requirements from LiDARs to IMU to Cameras. This service is being offered to selected partners only. If you would like to learn more about the calibration service, please reach out to us via email: **apollopartner@baidu.com**

## Hardware and Software Installation

The Hardware setup for Apollo 5.0 remains the same as Apollo 3.5, please refer to
[Apollo 3.5 Hardware and System Installation Guide](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_3_5_hardware_system_installation_guide.md)
for the steps to install the hardware components and the system software, as well as
[Apollo Software Installation Guide](../01_Installation%20Instructions/apollo_software_installation_guide.md).

## Dreamview Usage Table

For questions regarding Dreamview icons refer to the
[Dreamview Usage Table](../13_Apollo%20Tool/%E5%8F%AF%E8%A7%86%E5%8C%96%E4%BA%A4%E4%BA%92%E5%B7%A5%E5%85%B7Dremview/dreamview_usage_table.md).
For questions regarding Dreamland and the scenario editor, please visit our [Dreamland Introduction guide](../13_Apollo%20Tool/云平台Apollo Studio/Dreamland_introduction.md)

## Onboard Test

1. Plug-in an external hard-drive to any available USB port in the host machine

2. Turn on the vehicle, and then the host machine

3. Launch the Dev Docker Container

4. Launch DreamView

   Note\: Use your favorite browser to access Dreamview web service in your host
   machine browser with URL <http://localhost:8888>

   ![dreamview_2_5](images/dreamview_2_5.png)

5. Select Mode, Vehicle and Map

   ![dreamview_2_5_setup_profile](images/dreamview_2_5_setup_profile.png)

   Note\: You'll be required to setup profile before doing anything else. Click
   the dropdown menu to select **Navigation** mode, the HDMap and vehicle you
   want to use. The lists are defined in
   [HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/dreamview/conf/hmi.conf)

   Note\: It's also possible to change the profile on the right panel of the
   HMI, but just remember to click `Reset All` on the top-right corner to
   restart the system

6. Start the Modules.

   Click the `Setup` button

   ![dreamview_2_5_setup](images/dreamview_2_5_setup.png)

   Go to **Module Controller** tab, check if all modules and hardware are ready
   (Note\: In your offline environment, the hardware modules such as GPS,
   CANBus, Velodyne, Camera and Radar cannot be brought up)
   (Note\: You may need to drive around a bit to get a good GPS signal)

   ![dreamview_2_5_module_controller](images/dreamview_2_5_module_controller.png)

7. Under `Default Routing` select your desired route

8. Under Tasks click `Start Auto`. (Note: Be cautious when starting the autonomous
   driving, you should now be in autonomous mode)

   ![dreamview_2_5_start_auto](images/dreamview_2_5_start_auto.png)

9. After the autonomous testing is complete, under `Tasks` click `Reset All`, close all
   windows and shutdown the machine

10. Remove the hard drive
