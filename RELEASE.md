# Release 11.0
Apollo 8.0 introduced the concept of package management for user learning scenarios, making it easier and faster for users to deploy and use Apollo. With Apollo 9.0, we enabled users to more easily perform secondary development on top of Apollo through Package Management 2.0, allowing them to easily build their own autonomous driving applications. Apollo 10.0 focused on a comprehensive upgrade for large-scale scenario-based applications. In terms of performance, it optimized the performance and stability of various levels and modules, and provided rich tools to improve optimization efficiency. Apollo 11.0 focuses on the large-scale deployment of functional autonomous vehicles in high-value scenarios, comprehensively upgrading perception, localization, planning, and development toolchains, significantly lowering the hardware and software development threshold, and helping developers efficiently build end-to-end autonomous driving operating systems.

## Major New Features and Improvements:

## **Expanding Application Scenarios and Achieving End-to-End Operational Closure**
Apollo 11.0 is tailored for high-value scenarios such as **package delivery, street sweeping, security patrol, and campus shuttle services**, unlocking critical capabilities including:

* **Hill Start & Stop**: Meets vehicle start/stop requirements on complex terrains
* **Automatic Recovery**: Enhances autonomous recovery under abnormal road conditions
* **Edge-Hugging Driving**: Enables precise navigation along curbs or boundaries

Apollo 11.0 fully supports an end-to-end automated workflow:

**Vehicle Preparation → Route Planning → Autonomous Driving → Task Execution (e.g., cleaning, retail, passenger transport) → Anomaly Handling / Manual Takeover → Return-to-Depot Parking**

## **Enhanced Multi-Sensor Fusion Localization and Perception for Robust All-Terrain Operation**
* **Industrial-Grade High-Precision Localization**: Fuses **RTK, SLAM, vision, and wheel odometry** to achieve **centimeter-level accuracy** in GNSS-challenged environments such as **dense urban areas, underground parking garages, and fully indoor spaces**.
* **New Gate Recognition Capability**: Bridges **public roads and enclosed campuses**, enabling seamless autonomous transitions across domain boundaries.
* **Perception Upgrades**: Significantly improves detection accuracy for static obstacles including **curbs, barriers, and potholes**.
* **Upgraded BEV + OCC Perception Architecture**: Deeply integrated with the **Baidu Baige AI Computing Platform**, providing a streamlined pipeline from **model development → training → conversion → export**.
* **Support for Incremental Training**: Developers can rapidly fine-tune pre-trained models using their own data, accelerating model iteration and customization.

Through tight hardware-software co-design and a comprehensive ecosystem of developer tools, **Apollo 11.0 significantly lowers the R&D barrier and deployment cost for functional autonomous vehicles**, accelerating their commercialization and large-scale adoption.

# Release 10.0

In Apollo 8.0, the concept of package management tailored for user learning scenarios was introduced to enable users to deploy and use Apollo more conveniently and efficiently. In Apollo 9.0, the package management tool was updated to Version 2.0, making it easier for users to conduct secondary development and effortlessly build their own autonomous driving applications based on Apollo. In Apollo 10.0, we realize that autonomous driving cannot remain at the stage of local validation. Instead, it requires a comprehensive upgrade, and needs to be applied to scenarios on a large scale. In terms of performance, the performance and stability of various layers and modules are optimied, and extensive tools are provided to improve optimization efficiency. At the cost level, the hardware costs are recuded by enriching the hardware ecosystem which provides users with more options. Besides, the software development costs are lowered by upgrading the operating system, establishing communication with other frameworks, and reusing ecological software capabilities. Regarding safety, functional safety strategies and functional safety framework capabilities are reinforced.

## Major New Features and Improvements:

Better performance and enhanced stability to comprehensively reinforce Apollo's autonomous driving capabilities

- CyberRT Framework Upgrade: With industry-leading framework capabilities, low latency and low consumption, the performance is improved by 10 times, and the communication is more reliable. Apollo 10.0 features zero-copy communication, which ensures high performance and low consumption and achieves microsecond-level transmission latency. During inter-process communication, message are read and written directly on shared memory, avoiding memory copying, serialization, deserialization, and other resource-intensive operations. See [CyberRT Performance Report](docs/%E9%99%84%E5%BD%95/CyberRT%20Performance%20Report.md) for more details.
- Comprehensive and Detailed Performance Analysis Tool: It empowers developers to create high-quality autonomous driving system applications. Five key resources including CPU, GPU, memory, graphics memory, and IO are comprehensively monitored and tracked in real time with a visual interface to identify issues immediately. Function-level analysis capabilities ensure precise localization. Intuitively analysis of resource consumption distribution at the function-level granularity helps developers conduct in-depth optimizations to improve performance. See [Cyber RT Performance Analysis Tool](docs/%E6%A1%86%E6%9E%B6%E8%AE%BE%E8%AE%A1/%E8%BD%AF%E4%BB%B6%E6%A0%B8%E5%BF%83/CyberRT/Cyber%20RT%20Performance%20Analysis%20Tool.md) for more details.

Stronger capabilities, More various ecological support to significantly reduce R&D costs for both software and hardware

- Pure Vision Perception Solution with Low Cost and High Ceiling: The mainstream perception paradigms of visual BEV (Bird's Eye View) object detection and occupancy network are introduced into the Apollo open-source framework, and optimizations are performed based on industry-standard models, with each paradigm surpassing the industry-standard models in performance. Deployment is also extensively optimized, and now the inference frame rate can reach 5Hz on a single Orin platform. See [The Introduction and tutorials of vision object detection and occpancy prediction model](docs/%E5%BA%94%E7%94%A8%E5%AE%9E%E8%B7%B5/%E5%BC%80%E5%8F%91%E8%B0%83%E8%AF%95%E6%95%99%E7%A8%8B/Apollo%E6%84%9F%E7%9F%A5%E5%AE%9E%E8%B7%B5/%E6%84%9F%E7%9F%A5%E6%A8%A1%E5%9E%8B%E9%83%A8%E7%BD%B2%E8%AE%AD%E7%BB%83/The_introduction_and_tutorials_of_vision_obejct_detection_and_occpancy_prediction_model.md) for more details.
- Various Software Ecosystem and Accelerated Iteration Efficiency to Reduce Software Development and Deployment Costs: By integrating with the ROS ecosystem at the framework level, software reuse costs are lowered, R&D efficiency is enhanced, and more use scenarios are catered. The plugin engineering design simplifies expansion. Meanwhile, typical universal modules are ready to use at zero cost, while modules with customized capabilities can be integrated at low cost through conversion. Besides, it is more convenient and efficient to directly install and deploy multiple system versions on physical machines. Also, the deployment tool offers one-click packaging, enabling batch deployment of secondary development components. See [Cyber RT Communication with ROS](docs/%E6%A1%86%E6%9E%B6%E8%AE%BE%E8%AE%A1/%E8%BD%AF%E4%BB%B6%E6%A0%B8%E5%BF%83/CyberRT/Cyber%20RT%20Communication%20with%20ROS.md) and [Installation Guide](docs/%E5%AE%89%E8%A3%85%E6%8C%87%E5%8D%97/Installation%20Guide.md) for more details.
- Broader Hardware Ecosystem Support Offering More Options and More Suitable Scenarios to Reduce Hardware Selection Costs: An industry-leading hardware ecosystem is built through collaborations with over 32 manufacturers, supporting more than 73 devices and adding over 20 new models in our list. Particularly, the number of core devices (such as domain controllers, LiDARs, and inertial navigation systems) has doubled or tripled. We also provide a new hardware adaptation guide. All these provide a more convenient access mechanism for hardware ecosystem partners to connect with a broader market, fostering a collaborative ecosystem model.

Comprehensive Safety Coverage for More Stable and Reliable Autonomous Driving

- Compliance with Industry Safety Standards for Autonomous Driving: Functional safety software modules are designed and implemented based on relevant standards and specifications in ISO 26262 Road vehicles – Functional safety and ISO 21448 Road vehicles – Safety of the intended functionality.
- Comprehensive Functional Safety Strategies: These include hardware safety monitoring of sensors, computing units, chassis, vehicles, and other related components; full-link anomaly monitoring of sensor input to perception, prediction, planning, control, and chassis communication modules; runtime dependency environment anomaly monitoring, including system anomalies, resource occupancy, network, and IO; and independent capabilities for detecting collisions, traffic rule violations, and accidents.
- Robust Functional Safety Framework: With various detection items and easy expandability, the framework offers access standards for users to customize detection items at low cost. Besides, it is highly real-time, taking less than 1ms from anomaly detection to MRC (Minimum Risk Condition) strategy implementation. Meanwhile, it boasts diverse MRC strategy capabilities, with built-in configurations such as warnings, gentle braking, and emergency braking, and suopports custom development.

# Release 9.0

Apollo Open Source Platform 9.0 further focuses on enhancing the development and debugging experience, dedicated to provide autonomous driving developers with a unified development tool platform and easy-to-extend PnC and perception software framework interfaces. The new version reshapes the PnC and perception extension development method based on package management. It optimizes component splitting and configuration management according to business logic, simplifying the process of calling. In addition to the component extension method, a more lightweight plugin extension method has been added, simplifying the process of extending. The new version introduces Dreamview Plus, a brand-new developer tool that introduces modes for convenient multi-scenario use, a panel layout customizing visualization, and a resource center providing richer development resources. Furthermore, the LiDAR and Camera detection models in the new version have been upgraded for improved results, and incremental training methods have been opened up for easy extension. At the same time, support for 4D millimeter-wave radar has been added. Finally, the new version is adapted to the ARM architecture, and supports compilation and running on Orin, providing developers with additional device options.

## Major New Features and Improvements:

PnC Extension Development Pattern Based On Package Management

- Unified external interfaces to decouple the operation layer and PNC module.
- A brand-new plugin extension method to facilitate developers in developing and deploying their functionality.
- Global parameters and local parameters are divided to allow developers to query and modify parameters.

Perception Extension Development Pattern Based On Package Management

- Re-split the perception components based on "functional" granularity to facilitate reuse.
- A brand-new plugin development mode to facilitate the replacement of algorithms under the existing perception pipeline.
- Simplified and unified configuration to allow developers to query and modify parameters at any time.

Brand New Dreamview Plus Developer Tool

- Organize the usage scenarios of the development tool based on "mode", such as Perception mode, PnC mode, and Vehicle Test mode.
- Encapsulate each visualization functionality into an independent panel and support developers to customize the panel.
- Provide various resources such as maps, scenarios, vehicle configurations, and data records in the resource center to facilitate development and debugging.

Fully Upgraded Perception Model to Support Incremental Training

- CenterPoint is adopted to replace CNNSeg model by default in the LiDAR perception pipeline and YOLOX+YOLO3D is adopted to replace the original YOLO model by default in the Camera perception pipeline.
- Provide incremental training to improve perception model capabilities in specific scenarios by using a small amount of annotated data and Apollo pre-trained models.
- Support 4D millimeter-wave radar from hardware driver to perception model layer.

**[Note]** All models and methodologies included in Apollo 9.0 are for research purposes only. Productized and commercialized uses of these models are **NOT** encouraged, and it is at your own risk. Please be cautious to try Apollo 9.0 with sufficient safety protection mechanisms in place. Your feedback is highly appreciated so that we can continuously improve the models.

# Release 8.0

Apollo 8.0 is an effort to provide an extensible software framework and complete development cycle for Autonomous Driving developer. Apollo 8.0 introduces easily-reused Package to organize software modules. Apollo 8.0 integrates the whole process of perception development ,by combining model training service, model deployment tool and end-to-end visual validation tool . And another 3 new deep learning models are incorporated in Apollo 8.0 for perception module. Simulation service is upgraded by integrating local simulator in Dreamview to provide powerful debug tool for PnC developer.

## Major Features and Improvements

- Reusable software Package
  - Reorganize the modules based on Package to provide the functionality in an easy-to-consume manner
  - Fast installation experience based on Package, refer to [Installation - Package Method](docs/01_Installation%20Instructions/apollo_software_installation_guide_package_method.md)
  - Support customizing , publishing and sharing Package
- Brand New Deep Learning Models
  - CenterPoint, center-based two-stage 3D obstacle detection model
  - CaDDN, camera obstacle detection model
  - BEV PETR, camera obstacle detection model
- Complete Perception Development Process
  - Support Paddle3D to provide Model Training service
  - Provide model deployment tool by normalizing the model meta.
  - Provide visual validation tool in Dreamview
- Upgraded PnC Simulation Service
  - Provide PnC debug tool by integrating local simulator in Dreamview
  - Support scenario editing online and download in Dreamview

**[Note]** All models and methodologies included in Apollo 8.0 are for research purposes only. Productized and commercial uses of these models are NOT encouraged, and it is at your own risk. Please be cautious to try Apollo 8.0 with enough safety protection mechanism. Your feedback is highly appreciated for us to continuously improve the models.

# Release 7.0

Apollo 7.0 incorporates 3 brand new deep learning models to enhance the capabilities for Apollo Perception and Prediction modules. Apollo Studio is introduced in this version, combining with Data Pipeline, to provide a one-stop online development platform to better serve Apollo developers. Apollo 7.0 also publishes the PnC reinforcement learning model training and simulation evaluation service based on previous simulation service.

## Major Features and Improvements

- Brand New Deep Learning Models
  - Mask-Pillars obstacle detection model based on PointPillars
  - Inter-TNT prediction model based on interactive prediction & planning evaluator
  - Camera obstacle detection model based on SMOKE
- Apollo Studio Services
  - Practice environment service
  - Vehicle management service
- PnC Reinforcement Learning Services
  - Smart training and evaluation close-loop service
  - Extension Interface
- Upgraded Perception Module Code Structure

**[Note]** All models and methodologies included in Apollo 7.0 are for research purposes only. Productized and commercial uses of these models are **NOT** encouraged, and it is at your own risk. Please be cautious to try Apollo 7.0 with enough safety protection mechanism. Your feedback is highly appreciated for us to continuously improve the models.

# Release 6.0

Apollo 6.0 incorporates new deep learning models to enhance the capabilities for certain Apollo modules. This version works seamlessly with new additions of data pipeline services to better serve Apollo developers. Apollo 6.0 is also the first version to integrate certain features as a demonstration of our continuous exploration and experimentation efforts towards driverless technology.

## Major Features and Improvements

- Upgraded Deep Learning Models
  - PointPillars based obstacle detection model
  - Semantic map based pedestrian prediction model
  - Learning based trajectory planning model
- Brand New Data Pipeline Services
  - Low speed obstacle prediction model training service with semantic map support
  - PointPillars based obstacle detection model training service
  - Control profiling service
  - Vehicle dynamic model training service
  - Open space planner profiling service
  - Complete control parameter auto-tune service
- Driverless Research
  - Remote control interface with DreamView integration
  - Audio based emergency vehicle detection system

**[Note]** All models and methodologies included in Apollo 6.0 are for research purposes only. Productized and commercial uses of these models are **NOT** encouraged, and it is at your own risk. Please be cautious to try Apollo 6.0 with enough safety protection mechanism. Your feedback is highly appreciated for us to continuously improve the models.

# Release 5.5

Apollo 5.5 enhances the complex urban road autonomous driving capabilities of previous Apollo releases, by introducing curb-to-curb driving support. With this new addition, Apollo is now a leap closer to fully autonomous urban road driving. The car has complete 360-degree visibility, along with upgraded perception deep learning model a brand new prediction model to handle the changing conditions of complex road and junction scenarios, making the car more secure and aware. New Planning scenarios have been introduced to support curb-side functionality.

## Major Features And Improvements

- Brand new Data Pipeline Service
  - Sensor Calibration service
- Brand new module - Storytelling
- Scenario - Based Planning with a new planning scenarios to support curb-to-curb driving
  - Park-and-go
  - Emergency
- Prediction Model - Caution Obstacle
  - Semantic LSTM evaluator
  - Extrapolation predictor
- Control module
  - Model Reference Adaptive Control (MRAC)
  - Control profiling service
- Simulation scenarios

## Autonomous Drive Capabilities

Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 5.0 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM, PLEASE AVOID DRIVING THE VEHICLE ON THE HIGHWAY OR AT SPEEDS THAT ARE ABOVE OUR SUPPORTED THRESHOLD**.

# Release 5.0

Apollo 5.0 is an effort to support volume production for Geo-Fenced Autonomous Driving. The car now has 360-degree visibility, along with upgraded perception deep learning model to handle the changing conditions of complex road scenarios, making the car more secure and aware. Scenario-based planning has been enhanced to support additional scenarios like pull over and crossing bare intersections.

## Major Features And Improvements

- Brand new Data Pipeline Service
  - Vehicle Calibration
- New Perception algorithms
- Sensor Calibration Service
- Scenario - Based Planning with a new planning algorithm, Open Space Planner and new scenarios supported
  - Intersection - STOP Sign, Traffic Light, Bare Intersection
  - Park - Valet, Pull Over
- Map Data Verification tool
- Prediction Evaluators
- Simulation web platform - Dreamland
  - Scenario Editor
  - Control-in-loop Simulation
- Apollo Synthetic Data Set

## Autonomous Drive Capabilities

Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 5.0 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM, PLEASE AVOID DRIVING THE VEHICLE ON THE HIGHWAY OR AT SPEEDS THAT ARE ABOVE OUR SUPPORTED THRESHOLD**.

# Release 3.5

Apollo 3.5 is capable of navigating through complex driving scenarios such as residential and downtown areas. With 360-degree visibility and upgraded perception algorithms to handle the changing conditions of urban roads, the car is more secure and aware.

## Major Features And Improvements

- Upgraded Sensor Suite
  - VLS - 128 Line LiDAR
  - FPD-Link Cameras
  - Continental high-range function radars
  - Apollo Expansion Unit (AXU)
  - Additional IPC
- Brand new Runtime Framework - **Apollo CyberRT** which is specifically targeted towards autonomous driving
- New Perception algorithms
- Scenario - Based Planning with a new planning algorithm, Open Space Planner
- New Localization algorithm
- V2X Capabilities
- Open Vehicle Certification platform - 2 new vehicles added **GAC GE3** and **GWM WEY VV6**

## Autonomous Drive Capabilities

Vehicles with this version can drive autonomously in complex urban road conditions including both residential and downtown areas. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. URBAN DRIVING INVOLVES NAVIGATING HIGH RISK ZONES LIKE SCHOOLS, PLEASE TEST APOLLO 3.5 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM**.

# Release 3.0

Apollo 3.0 enables L4 product level solution that allows vehicles to drive in a closed venue setting at a low speed. Automakers can now leverage this one stop solution for autonomous driving without having to customize on their own.

## Major Features And Improvements

- New Safety module called Guardian
- Enhanced Surveillance module - Monitor
- Hardware service layer that will now act like a platform and not a product, giving developers the flexibility to integrate their own Hardware
- Apollo Sensor Unit (ASU)
- New Gatekeeper - Ultrasonic Sensor
- Perception module changes:
  - **CIPV(Closest In-Path Vehicle) detection and Tailgating**: The vehicle in front of the ego-car is detected and its trajectory is estimated for more efficient tailgating and lane keeping when lane detection is unreliable.
  - **Asynchronous sensor fusion**: unlike the previous version, Perception in Apollo 3.0 is capable of consolidating all the information and data points by asynchronously fusing LiDAR, Radar and Camera data. Such conditions allow for more comprehensive data capture and reflect more practical sensor environments.
  - **Online pose estimation**: This new feature estimates the pose of an ego-vehicle for every single frame. This feature helps to drive through bumps or slopes on the road with more accurate 3D scene understanding.
  - **Ultrasonic sensors**: Perception in Apollo 3.0 now works with ultrasonic sensors. The output can be used for Automated Emergency Brake (AEB) and vertical/perpendicular parking.
  - **Whole lane line**: Unlike previous lane line segments, this whole lane line feature will provide more accurate and long range detection of lane lines.
  - **Visual localization**: Cameras are currently being tested to aid and enhance localization
  - **16 beam LiDAR support**

## Autonomous Driving Capabilities

Vehicles with this version can drive autonomously in a Closed Venue setting. It is a production ready version for low-speed autonomous driving capabilities.

# Release 2.5

This release allows the vehicle to autonomously run on geo-fenced highways. Vehicles are able to do lane keeping cruise and avoid collisions with the leading vehicles.

## Major Features And Improvements

- Upgrade localization based on multiple sensor fusion (MSF)
- Upgrade DreamView with more visualization features
- Add HD map data collection tool
- Add vision based perception with obstacle and lane mark detections
- Add relative map to support ACC and lane keeping for planning and control
- Make docker file available

## Autonomous Drive Capabilities

Vehicles with this version can drive autonomously on highways at higher speed with limited HD map support. The highway needs to have clear white painted lane marks with minimum curvatures. The performance of vision based perception will degrade significantly at night or with strong light flares. **BE CAUTIOUS WHEN DRIVING AUTONOMOUSLY, ESPECIALLY AT NIGHT OR IN POOR VISION ENVIRONMENT. PLEASE TEST APOLLO 2.5 WITH THE SUPPORT FROM APOLLO ENGINEERING TEAM**.

# Release 2.0

This release supports that vehicles autonomously drive on simple urban roads. Vehicles are able to cruise and avoid collisions with obstacles, stop at traffic lights and change lanes if needed to reach the destination.

## Major Features And Improvements

- Add traffic light detection
- Add obstacle classification and support obstacle categories: _vehicle_, _pedestrian_, _cyclist_ and _unknown_
- Upgrade planning capability to change lanes in order to reach the destination
- Add point cloud based localization algorithm fusion with RTK
- Add MPC based control algorithm
- Add RNN model for traffic prediction
- Integrate HMI and DreamView
- Redesign DreamView and upgrade it with additional debugging visualization tools
- Add additional debugging tools in `modules/tools`
- Add release docker image upgrade through secure OTA
- Add USB camera and radar driver support

## Autonomous Driving Capabilities

Vehicles with this version can drive autonomously on simple urban roads with light to medium traffic at slow to medium speed.

# Release 1.5

This release supports that vehicles autonomously cruise on fixed lanes.

## Major Features And Improvements

- Add routing, perception, prediction, planning and end-to-end
  - **_Perception_**: 3D point cloud based obstacle detection and tracking with GPU support
  - **_Prediction_**: Deep neural network MLP prediction model and multiple predictors handling different categories of obstacles
  - **_Planning_**: traffic law modules, multiple iterations of DP and QP optimizations for path and speed
  - **_End-to-end_**: Mixed deep neural network models with convolutional LSTM in longitudinal and FCNN in lateral
- Add HD Map engine APIs
- Add Velodyne 64 LiDAR driver support
- Add debugging tools in `modules/tools/`
- Improve HMI and DreamView features to allow realtime traffic display and traffic scenario replay.

## Autonomous Driving Capabilities

Vehicles with this version do **NOT** detect traffic lights. Vehicles will **NOT** stop at red traffic lights. Neither will they change lanes on the road.

# Release 1.0

Initial release of Apollo implements autonomous GPS waypoint following.

## Major Features And Improvements

- Include localization, control
  - **_Location_**: RTK
  - **_Control_**: calibration table in longitudinal and LQR in lateral
- Add GPS/IMU gnss driver support
- Use HMI to record and replay a trajectory, and DreamView to visualize vehicle trajectory
- Include debugging tools in `modules/tools/`

## Autonomous Driving Capabilities

Vehicles with this version do **NOT** perceive obstacles in close promixity. Neither can they drive on public roads or areas without GPS signals.
