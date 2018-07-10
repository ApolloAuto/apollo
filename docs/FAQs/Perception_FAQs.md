# Perception FAQs

## How many typer of sensors calibrations are present?

There are 5 types of calibration:
* Camera - Camera 
* Camera - LiDAR
* Radar - Camera 
* LiDAR - GNSS 
* IMU - Vehicle

For additional information on Sensor Calibration, please refer to our [calibration guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_sensor_calibration_guide.md)

For additional information on LiDAR - GNSS calibration please refer to our [LiDAR - GNSS calibration guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_5_lidar_calibration_guide.md)

---
## Is the order of sensor calibration important at all ? Can I do IMU - vehicle before Radar - Camera ?

Yes it is important, but you could calibrate the IMU - Vehicle before Radar - Camera

---
## How do I run the Offline Perception Visualizer?

Refer to the How-To guide located [here](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md).

---
## Are you going to release the source code for the calibration tools ?

No, we currently have not released the source code for our calibration tools.

---
## How do you ensure that the right calibration files are loaded for the perception module ?

In each car, specific Camera parameters should be saved and the default parameters will be replaced by the saved parameters when you install the release version.
When user select a vehicle, we'll copy files from calibration/data/[vehicle]/[src_data] to target places.
The code is at [Link](https://github.com/ApolloAuto/apollo/blob/master/modules/dreamview/backend/hmi/vehicle_manager.cc#L43).
This action is totally configurable, see the [example here](https://github.com/ApolloAuto/apollo/blob/master/modules/dreamview/conf/vehicle_data.pb.txt)

---
**More Perception FAQs to follow**
