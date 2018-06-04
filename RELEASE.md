# Release 2.5
This release allows the vehicle to autonomously run on geo-fenced highways. Vehicles are able to do lane keeping cruise and avoid collisions with the leading vehicles.

## Major Features And Improvements
* Upgrade localization based on multiple sensor fusion (MSF)
* Upgrade DreamView with more visualization features
* Add HD map data collection tool
* Add vision based perception with obstacle and lane mark detections
* Add relative map to support ACC and lane keeping for planning and control
* Make dockerfile available

## Autonomous Drive Capabilities
Vehicles with this version can drive autonomously on highways at higher speed with limited HD map support. The highway needs to have clear white painted lane marks with minimum curvatures. The performance of vision based perception will degrade significantly at night or with strong light flares. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. PLEASE TEST APOLLO 2.5 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM**.

# Release 2.0
This release supports that vehicles autonomously drive on simple urban roads. Vehicles are able to cruise and avoid collisions with obstacles, stop at traffic lights and change lanes if needed to reach the destination.

## Major Features And Improvements
* Add traffic light detection
* Add obstacle classification and support obstacle categories: _vehicle_, _pedestrian_, _cyclist_ and _unknown_
* Upgrade planning capability to change lanes in order to reach the destination
* Add point cloud based localization algorithm fusion with RTK
* Add MPC based control algorithm
* Add RNN model for traffic prediction
* Integrate HMI and DreamView
* Redesign DreamView and upgrade it with additional debugging visualization tools
* Add additional debugging tools in `modules/tools`
* Add release docker image upgrade through secure OTA
* Add USB camera and radar driver support

## Autonomous Driving Capabilities
Vehicles with this version can drive autonomously on simple urban roads with light to medium traffic at slow to medium speed.

# Release 1.5
This release supports that vehicles autonomously cruise on fixed lanes.

## Major Features And Improvements
* Add routing, perception, prediction, planning and end-to-end
  * **_Perception_**: 3D point cloud based obstacle detection and tracking with GPU support
  * **_Prediction_**: Deep neural network MLP prediction model and multiple predictors handling different categories of obstacles
  * **_Planning_**: traffic law modules, multiple iterations of DP and QP optimizations for path and speed
  * **_End-to-end_**: Mixed deep neural network models with convolutional LSTM in longitudinal and FCNN in lateral
* Add HD Map engine APIs
* Add Velodyne 64 LiDAR driver support
* Add debugging tools in `modules/tools/`
* Improve HMI and DreamView features to allow realtime traffic display and traffic scenario replay.

## Autonomous Driving Capabilities
Vehicles with this version do **NOT** detect traffic lights. Vehicles will **NOT** stop at red traffic lights. Neither will them change lanes on the road.

# Release 1.0
Initial release of Apollo implements autonomous GPS waypoint following.

## Major Features And Improvements
* Include localization, control
  * **_Location_**: RTK
  * **_Control_**: calibration table in longitudinal and LQR in lateral
* Add GPS/IMU gnss driver support
* Use HMI to record and replay a trajectory, and DreamView to visualize vehicle trajectory
* Include debugging tools in `modules/tools/`

## Autonomous Driving Capabilities
Vehicles with this version do **NOT** perceive obstacles in close promixity. Neither can them drive on public roads or areas without GPS signals.
