# Apollo 6.0 Quick Start Guide

The following guide serves as a user manual for launching the Apollo upgraded software and hardware stack on vehicle, which
is similar to Apollo 5.5.

This Quick Start Guide focuses on the new features. For general Apollo concepts, please refer to
[Apollo 5.5 Quick Start](apollo_5_5_quick_start.md)

## Contents

- [Apollo 6.0 Quick Start Guide](#apollo-60-quick-start-guide)
  - [Contents](#contents)
  - [Emergency Audio Detection](#emergency-audio-detection)
  - [New Deep Learning Models](#new-deep-learning-models)
  - [Hardware and Software Installation](#hardware-and-software-installation)
  - [Dreamview Usage Table](#dreamview-usage-table)
  - [Onboard Test](#onboard-test)

## Emergency Audio Detection

Apollo currently integrates emergency vehicle detection through audio devices. Microphones are installed on the vehicle to collect audio signals around the autonomous vehicle, and the recorded soundtracks will be analized and processed to detect emergency vehicles in the surroundings. The module detail is [here](../../modules/audio).

## New Deep Learning Models

Apollo integrates new deep learning models including obstacle detection with PointPillars, pedestrian prediction with semantic map and learning based trajectory planning. Please find details in corresponding modules

## Hardware and Software Installation

The Hardware setup for Apollo 6.0 remains the same as Apollo 3.5, please refer to
[Apollo 3.5 Hardware and System Installation Guide](apollo_3_5_hardware_system_installation_guide.md)
for the steps to install the hardware components and the system software, as well as
[Apollo Software Installation Guide](apollo_software_installation_guide.md).

## Dreamview Usage Table

For questions regarding Dreamview icons refer to the
[Dreamview Usage Table](../specs/dreamview_usage_table.md).
For questions regarding Dreamland and the scenario editor, please visit our [Dreamland Introduction guide](../specs/Dreamland_introduction.md)

## Onboard Test

1. Plug-in an external hard-drive to any available USB port in the host machine

2. Turn on the vehicle, and then the host machine

3. Launch the Dev Docker Container

4. Launch DreamView

   Open URL <http://localhost:8888> to launch  Dreamview web service onyour host

   ![launch_dreamview](images/dreamview_2_5.png)

5. Select Mode, Vehicle and Map

   ![select_mode](images/dreamview_2_5_setup_profile.png)

   Note\: You'll be required to setup profile before doing anything else. Click
   the dropdown menu to select **Navigation** mode, the HDMap and vehicle you
   want to use. The lists are defined in
   [HMI config file](../../modules/dreamview/conf/hmi_modes)

   Note\: It's also possible to change the profile on the right panel of the
   HMI, but just remember to click `Reset All` on the top-right corner to
   restart the system

6. Start the Modules.

   Click the `Setup` button

   ![start_module](images/dreamview_2_5_setup.png)

   Go to **Module Controller** tab, check if all modules and hardware are ready
   (Note\: In your offline environment, the hardware modules such as GPS,
   CANBus, Velodyne, Camera and Radar cannot be brought up)
   (Note\: You may need to drive around a bit to get a good GPS signal)

   ![module_controller](images/dreamview_2_5_module_controller.png)

7. Under `Default Routing` select your desired route

8. Under Tasks click `Start Auto`. (Note: Be cautious when starting the autonomous
   driving, you should now be in autonomous mode)

   ![start_auto](images/dreamview_2_5_start_auto.png)

9. After the autonomous testing is complete, under `Tasks` click `Reset All`, close all
   windows and shutdown the machine

10. Remove the hard drive
