# Release 5.5

Apollo 5.5 enhances the complex urban road autonomous driving capabilities of previous Apollo releases, by introducing curb-to-curb driving support. With this new addition, Apollo is now a leap closer to fully autonomous urban road driving. 
The car has complete 360-degree visibility, along with upgraded perception deep learning model a brand new prediction model to handle the changing conditions of complex road and junction scenarios, making the car more secure and aware. New Planning scenarios have been introduced to support curb-side functionality.

## Major Features And Improvements
* Brand new Data Pipeline Service
    * Sensor Calibration service
* Brand new module - Storytelling
* Scenario - Based Planning with a new planning scenarios to support curb-to-curb driving
    * Park-and-go
    * Emergency
* Prediction Model - Caution Obstacle 
    * Semantic LSTM evaluator
    * Extrapolation predictor
* Control module
    * Model Reference Adaptive Control (MRAC)
    * Control profiling service
* Simulation scenarios

## Autonomous Drive Capabilities
Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 5.0 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM, PLEASE AVOID DRIVING THE VEHICLE ON THE HIGHWAY OR AT SPEEDS THAT ARE ABOVE OUR SUPPORTED THRESHOLD**.

# Release 5.0

Apollo 5.0 is an effort to support volume production for Geo-Fenced Autonomous Driving. The car now has 360-degree visibility, along with upgraded perception deep learning model to handle the changing conditions of complex road scenarios, making the car more secure and aware. Scenario-based planning has been enhanced to support additional scenarios like pull over and crossing bare intersections.

## Major Features And Improvements
* Brand new Data Pipeline Service
    * Vehicle Calibration
* New Perception algorithms
* Sensor Calibration Service
* Scenario - Based Planning with a new planning algorithm, Open Space Planner and new scenarios supported
    * Intersection - STOP Sign, Traffic Light, Bare Intersection
    * Park - Valet, Pull Over
* Map Data Verification tool
* Prediction Evaluators
* Simulation web platform - Dreamland
    * Scenario Editor
    * Control-in-loop Simulation
* Apollo Synthetic Data Set

## Autonomous Drive Capabilities
Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 5.0 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM, PLEASE AVOID DRIVING THE VEHICLE ON THE HIGHWAY OR AT SPEEDS THAT ARE ABOVE OUR SUPPORTED THRESHOLD**.

# Release 3.5

Apollo 3.5 is capable of navigating through complex driving scenarios such as residential and downtown areas. With 360-degree visibility and upgraded perception algorithms to handle the changing conditions of urban roads, the car is more secure and aware.

## Major Features And Improvements
* Upgraded Sensor Suite
    * VLS - 128 Line LiDAR
    * FPD-Link Cameras
    * Continental high-range function radars
    * Apollo Expansion Unit (AXU)
    * Additional IPC
* Brand new Runtime Framework - **Apollo CyberRT** which is specifically targeted towards autonomous driving
* New Perception algorithms
* Scenario - Based Planning with a new planning algorithm, Open Space Planner
* New Localization algorithm
* V2X Capabilities
* Open Vehicle Certification platform - 2 new vehicles added **GAC GE3** and **GWM WEY VV6**

## Autonomous Drive Capabilities
Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 3.5 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM**.

# Release 3.0

Apollo 3.0 enables L4 product level solution that allows vehicles to drive in a closed venue setting at a low speed. Automakers can now leverage this one stop solution for autonomous driving without having to customize on their own.

## Major Features And Improvements
* New Safety module called Guardian
* Enhanced Surveillance module - Monitor
* Hardware service layer that will now act like a platform and not a product, giving developers the flexibility to integrate their own Hardware
* Apollo Sensor Unit (ASU)
* New Gatekeeper - Ultrasonic Sensor
* Perception module changes:
  * **CIPV(Closest In-Path Vehicle) detection and Tailgating**: The vehicle in front of the ego-car is detected and its trajectory is estimated for more efficient tailgating and lane keeping when lane detection is unreliable.
  * **Asynchronous sensor fusion**: unlike the previous version, Perception in Apollo 3.0 is capable of consolidating all the information and data points by asynchronously fusing LiDAR, Radar and Camera data. Such conditions allow for more comprehensive data capture and reflect more practical sensor environments.
  * **Online pose estimation**: This new feature estimates the pose of an ego-vehicle for every single frame. This feature helps to drive through bumps or slopes on the road with more accurate 3D scene understanding.
  * **Ultrasonic sensors**: Perception in Apollo 3.0 now works with ultrasonic sensors. The output can be used for Automated Emergency Brake (AEB) and vertical/perpendicular parking.
  * **Whole lane line**: Unlike previous lane line segments, this whole lane line feature will provide more accurate and long range detection of lane lines.
  * **Visual localization**: Cameras are currently being tested to aid and enhance localization
  * **16 beam LiDAR support**

## Autonomous Driving Capabilities
Vehicles with this version can drive autonomously in a Closed Venue setting. It is a production ready version for low-speed autonomous driving capabilities.

# Release 2.5
This release allows the vehicle to autonomously run on geo-fenced highways. Vehicles are able to do lane keeping cruise and avoid collisions with the leading vehicles.

## Major Features And Improvements
* Upgrade localization based on multiple sensor fusion (MSF)
* Upgrade DreamView with more visualization features
* Add HD map data collection tool
* Add vision based perception with obstacle and lane mark detections
* Add relative map to support ACC and lane keeping for planning and control
* Make docker file available

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
Vehicles with this version do **NOT** detect traffic lights. Vehicles will **NOT** stop at red traffic lights. Neither will they change lanes on the road.

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
Vehicles with this version do **NOT** perceive obstacles in close promixity. Neither can they drive on public roads or areas without GPS signals.
