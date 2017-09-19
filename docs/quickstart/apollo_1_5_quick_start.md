# Apollo 1.5 Quick Start Guide

This quick start focuses on Apollo 1.5 new features. For general Apollo concepts, please refer to [Apollo 1.0 Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md). 

Before doing the following steps, make sure you have calibrated the extrinsic parameters between the LiDAR and the GNSS/INS. For sensor calibration, please refer to [Apollo 1.5 LiDAR calibration guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_lidar_calibration_guide.md).

## Launch release env Docker Image

Run the following commands:

```bash
cd $APOLLO_HOME
bash docker/scripts/release_start.sh
```

When Docker starts, it creates a port mapping, which maps the Docker internal port 8887 to the host port 8887. You can then visit the HMI web service in your host machine browser:

Open the Chrome browser and start the Apollo HMI by going to **localhost:8887**.
 ![](images/hmi_setup_profile.png)
You'll be required to setup profile before doing anything else. Click the
dropdown menu to select your HDMap and vehicle in use. The list are defined in
[HMI config file](https://raw.githubusercontent.com/ApolloAuto/apollo/master/modules/hmi/conf/config.pb.txt).

Then your HMI comes to live!

*Note: It's also possible to change profile on the right panel of HMI, but just
remember to click "Reset All" on the top-right corner to restart the system.*

 ![](images/start_hmi.png)

## (*New!*) Start Auto

In Apollo 1.5, we released the new feature, auto following the traffic until destination.

1. To make it work,  you need setup the system by clicking the "Setup"
   button on left panel.

 ![](images/hmi_setup_1.5.png)

2. Make sure all modules are on and hardware is ready, and the vehicle is in a

good state which is safe to enter auto mode to follow the traffic to destination.

Click the "Start Auto" button, then it will drive you there!

 ![](images/hmi_start_auto_following.png)
