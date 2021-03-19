# Perception FAQs

## How many typer of sensors calibrations are present?

There are 5 types of calibration:

- Camera - Camera
- Camera - LiDAR
- Radar - Camera
- LiDAR - GNSS
- IMU - Vehicle

For additional information on Sensor Calibration, please refer to our
[calibration guide](../quickstart/apollo_2_0_sensor_calibration_guide.md)

For additional information on LiDAR - GNSS calibration please refer to our
[LiDAR - GNSS calibration guide](https://github.com/ApolloAuto/apollo/blob/r1.5.0/docs/quickstart/apollo_1_5_lidar_calibration_guide.md)

---

## Is the order of sensor calibration important at all ? Can I do IMU - vehicle before Radar - Camera ?

Yes it is important, but you could calibrate the IMU - Vehicle before Radar -
Camera

---

## Are you going to release the source code for the calibration tools ?

No, we currently have not released the source code for our calibration tools.

---

## How do you ensure that the right calibration files are loaded for the perception module ?

In each car, specific Camera parameters should be saved and the default
parameters will be replaced by the saved parameters when you install the release
version. When user select a vehicle, we'll copy files from
calibration/data/[vehicle]/[src_data] to target places. The code is at
[Link](../../modules/dreamview/backend/hmi/vehicle_manager.cc). This action is
totally configurable, see the
[example here](../../modules/dreamview/conf/vehicle_data.pb.txt)

---

## I am trying to run the extrinsic sensor calibration tools and they seem to complain about the INS_STAT not being 56. But when I echo the /apollo/sensor/gnss/ins_stat topic I can see that the position type is 56. What could be the problem?

The error log occurs (stat=-1) whenever the tool has not received any GNSS
message or the image stamp is not correct. Please check the topic of GNSS
message or image stamp.

---

**More Perception FAQs to follow**
