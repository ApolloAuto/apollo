# Hardware FAQs

### What hardware is needed for Apollo?
The required hardware for each version of Apollo can be found at the following links:
* [Hardware for Apollo 1.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md)
* [Hardware for Apollo 1.5](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_hardware_system_installation_guide.md)
* [Hardware for Apollo 2.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md)
* [Hardware for Apollo 2.5](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)
* [Hardware for Apollo 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_hardware_system_installation_guide.md)

---
### Which types of vehicles can Apollo be run on?
Currently, the Apollo control algorithm is configured for our default vehicle, which is a Lincoln MKZ. If you would like to use a different vehicle type, please visit [this](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_add_a_new_vehicle.md) page.

---
### Which types of LiDAR are supported by Apollo?
In Apollo 1.5, only Velodyne 64 is supported. Users are welcome to add drivers to ROS in order to support other models of LiDAR.

---
### Do you have a list of Hardware devices that are compatible with Apollo

Refer to the [Hardware Installation Guide 3.0](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_0_hardware_system_installation_guide.md) for information on all devices that are compatible with Apollo 3.0.
If you are looking for a different version of Apollo, refer to that version's Hardware Installation guide found under `docs/quickstart`

---
### What is the difference between Apollo Platform Supported devices and Apollo Hardware Development Platform Supported device?

1. Apollo Platform Supported means 
    *	The device is part of the Apollo hardware reference design
    *	One or more device(s) has/have been tested and passed to become a fully functional module of the corresponding hardware category, which provides adequate support for upper software layers
2. Apollo Hardware Development Platform certified means 
one or more device(s) has/have been tested and passed for data collection purpose only. Please note, that in order to collect useful data, it is required for the device to work with the rest of necessary hardware devices listed in the Apollo Reference Design. In order to achieve the same performance in Perception and other upper software modules, it would require extra effort from the developers’ side, including the creation of new model(s), annotation of the data, training the new models, etc.

---
### I do not have an IMU, now what? 
Without an IMU, the localization would depend on GPS system which only updates once per second. On top of that, you wouldn't have velocity and heading information of your vehicle. That is probably not a good idea unless you have other solutions.

---
### I have only VLP16, can I work with it? The documentation advises me to use HDL64

HDL64 provides a much denser point cloud than VLP-16 can. It gives a further detection range for obstacles. That is why we recommend it in the reference design. Whether VLP-16 works for your project, you will need to find out. 

**More Hardware FAQs to follow.**
