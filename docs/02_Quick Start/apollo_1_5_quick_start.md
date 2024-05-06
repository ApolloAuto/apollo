# Apollo 1.5 Quick Start Guide

This quick start focuses on Apollo 1.5 new features. For general Apollo concepts, please refer to [Apollo 1.0 Quick Start](../02_Quick%20Start/apollo_1_0_quick_start.md) and [Apollo Software Installation Guide](../01_Installation Instructions/apollo_software_installation_guide.md).

# Onboard Test

1. For vehicle onboard test, make sure you have calibrated the extrinsic parameters between the LiDAR and the GNSS/INS. For sensor calibration, please refer to [Apollo 1.5 LiDAR calibration guide](../11_Hardware%20Integration%20and%20Calibration/传感器标定/apollo_lidar_imu_calibration_guide.md) before you proceed.

2. Launch Docker Release Container

3. Launch HMI

    Use your favorite browser to access HMI web service in your host machine browser with URL http://localhost:8887

4. Select Vehicle and Map

    You'll be required to setup profile before doing anything else. Click the dropdown menu to select your HDMap and vehicle in use. The list are defined in [HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/hmi/conf/config.pb.txt).

    *Note: It's also possible to change profile on the right panel of HMI, but just remember to click "Reset All" on the top-right corner to restart the system.*

    ![](images/start_hmi.png)

5. Start Modules

    Set up the system by clicking the "Setup" button on left panel.

    ![](images/hmi_setup_1.5.png)

6. (*New!*) Be Cautious When Starting Autonomous Driving

    Make sure all modules are on and hardware is ready, and the vehicle is in a good state which is safe to enter auto mode to follow the traffic to destination.

    Click the "Start Auto" button, then it will drive you there!
    ![](images/hmi_start_auto_following.png)
