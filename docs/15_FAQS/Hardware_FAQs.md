# Hardware FAQs

### What hardware is needed for Apollo?

The required hardware for each version of Apollo can be found at the following
links:

- [Hardware for Apollo 1.0](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_1_0_hardware_system_installation_guide.md)
- [Hardware for Apollo 1.5](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_1_5_hardware_system_installation_guide.md)
- [Hardware for Apollo 2.0](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_2_0_hardware_system_installation_guide_v1.md)
- [Hardware for Apollo 2.5](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_2_5_hardware_system_installation_guide_v1.md)

---

### Which types of vehicles can Apollo be run on?

Currently, the Apollo control algorithm is configured for our default vehicle,
which is a Lincoln MKZ. If you would like to use a different vehicle type,
please visit [this](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%80%82%E9%85%8D/how_to_add_a_new_vehicle.md) page.

---

### Which types of LiDAR are supported by Apollo?

In Apollo 1.5, only Velodyne 64 is supported. Users are welcome to add drivers
to ROS in order to support other models of LiDAR.

---

### Do you have a list of Hardware devices that are compatible with Apollo

Refer to the
[Hardware Installation Guide 3.0](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E7%A1%AC%E4%BB%B6%E5%AE%89%E8%A3%85hardware%20installation/apollo_3_0_hardware_system_installation_guide.md)
for information on all devices that are compatible with Apollo 3.0. If you are
looking for a different version of Apollo, refer to that version's Hardware
Installation guide found under `docs/quickstart`

---

### What is the difference between Apollo Platform Supported devices and Apollo Hardware Development Platform Supported device?

1. Apollo Platform Supported means
   - The device is part of the Apollo hardware reference design
   - One or more device(s) has/have been tested and passed to become a fully
     functional module of the corresponding hardware category, which provides
     adequate support for upper software layers
2. Apollo Hardware Development Platform supported means one or more device(s)
   has/have been tested and passed for data collection purpose only. Please
   note, that in order to collect useful data, it is required for the device to
   work with the rest of necessary hardware devices listed in the Apollo
   Reference Design. In order to achieve the same performance in Perception and
   other upper software modules, it would require extra effort from the
   developers’ side, including the creation of new model(s), annotation of the
   data, training the new models, etc.

---

### I do not have an IMU, now what?

Without an IMU, the localization would depend on GPS system which only updates
once per second. On top of that, you wouldn't have velocity and heading
information of your vehicle. That is probably not a good idea unless you have
other solutions.

---

### I have only VLP16, can I work with it? The documentation advises me to use HDL64

HDL64 provides a much denser point cloud than VLP-16 can. It gives a further
detection range for obstacles. That is why we recommend it in the reference
design. Whether VLP-16 works for your project, you will need to find out.

---

### Is HDL32 (Velodyne 32 line LiDAR) compatible with Apollo?

Apollo can work successfully for HDL32 Lidars. You could follow the
[Puck Series Guide](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Lidar/VLP_Series_Installation_Guide.md) alongwith
the following
[modification](https://github.com/ApolloAuto/apollo/commit/df37d2c79129434fb90353950a65671278a4229e#diff-cb9767ab272f7dc5b3e0d870a324be51).
However, please note that you would need to change the intrinsics for HDL32 in
order to avoid
[the following error](https://github.com/ApolloAuto/apollo/issues/5244).

---

### How to set the USB cameras to provide valid time stamp?

First use time_sync.sh script to sync the system clock to NTP servers. Then
reset UVCVideo module with clock set to realtime with root access.

```
rmmod UVCVideo; modprobe UVCVideo clock=realtime
```

---

**More Hardware FAQs to follow.**
