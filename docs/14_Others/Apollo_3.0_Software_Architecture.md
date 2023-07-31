# Apollo 3.0 Software Architecture

Core software modules running on the Apollo 3.0 powered autonomous vehicle include:

- **Perception** — The perception module identifies the world surrounding the autonomous vehicle. There are two important submodules inside perception: obstacle detection and traffic light detection.
- **Prediction** — The prediction module anticipates the future motion trajectories of the perceived obstacles.
- **Routing** — The routing module tells the autonomous vehicle how to reach its destination via a series of lanes or roads.
- **Planning** — The planning module plans the spatio-temporal trajectory for the autonomous vehicle to take.
- **Control** — The control module executes the planned spatio-temporal trajectory by generating control commands such as throttle, brake, and steering.
- **CanBus** — The CanBus is the interface that passes control commands to the vehicle hardware. It also passes chassis information to the software system.
- **HD-Map** — This module is similar to a library. Instead of publishing and subscribing messages, it frequently functions as query engine support to provide ad-hoc structured information regarding the roads.
- **Localization** — The localization module leverages various information sources such as GPS, LiDAR and IMU to estimate where the autonomous vehicle is located.
- **HMI** - Human Machine Interface or DreamView in Apollo is a module for viewing the status of the vehicle, testing other modules and controlling the functioning of the vehicle in real-time.
- **Monitor** - The surveillance system of all the modules in the vehicle including hardware.
- **Guardian** - A new safety module that performs the function of an Action Center and intervenes should Monitor detect a failure.


```
Note: Detailed information on each of these modules is included below.
```

The interactions of these modules are illustrated in the picture below.

![img](images/Apollo_3.0_SW.png)

Every module is running as a separate CarOS-based ROS node. Each module node publishes and subscribes certain topics. The subscribed topics serve as data input while the published topics serve as data output. The detailed interactions are described in the following sections.

## Perception

Perception depends on the raw sensor data such as LiDAR point cloud data and camera data. In addition to these raw sensor data inputs, traffic light detection also depends on the localization data as well as the HD-Map. Because real-time ad-hoc traffic light detection is computationally infeasible,  traffic light detection needs localization to determine when and where to start detecting traffic lights through the camera captured pictures.
Changes to Apollo 3.0:
 - CIPV detection/ Tailgating – moving within a single lane
 -	Whole lane line support - bold line support for long range accuracy. There are 2 different types on installations for Camera, low and high installation.
 -	Asynchronous sensor fusion – get all the information and get data points by asynchronously fusing LiDAR, Radar and Camera data. This is specifically important because of the frame rate differences in the different sensors – Radar is 10ms, Camera is 33ms and LiDAR is 100ms
 -	Online pose estimation – determines angle change and estimates it when there are bumps or slopes to ensure that the sensors move with the car and the angle/pose changes accordingly
 -	Visual localization – we now use camera for localization. This functionality is currently being tested.
 -	Ultrasonic Sensor – Currently being tested as the final gatekeeper to be used in conjunction with Guardian for Automated Emergency brake and vertical/perpendicular parking.

## Prediction

The prediction module estimates the future motion trajectories for all the perceived obstacles. The output prediction message wraps the perception information. Prediction subscribes to both localization and perception obstacle messages as shown below.

![img](images/prediction.png)

When a localization update is received, the prediction module updates its internal status. The actual prediction is triggered when perception sends out its published perception obstacle message.

## Localization

The localization module aggregates various data to locate the autonomous vehicle. There are two types of localization modes: OnTimer and Multiple SensorFusion.

The first localization method is RTK-based, with a timer-based callback function `OnTimer`, as shown below.

![img](images/localization.png)

The other localization method is the Multiple Sensor Fusion (MSF) method, where a bunch of event-triggered callback functions are registered, as shown  below.

![img](images/localization_2.png)

## Routing

The routing module needs to know the routing start point and routing end point, to compute the passage lanes and roads. Usually the routing start point is the autonomous vehicle location. The important data interface is an event triggered function called `OnRoutingRequest`, in which `RoutingResponse` is computed and published as shown below.

![img](images/routing.png)

## Planning

Apollo 2.0 uses several information sources to plan a safe and collision free trajectory, so the planning module interacts with almost every other module.

Initially, the planning module takes the prediction output. Because the prediction output wraps the original perceived obstacle, the planning module subscribes to the traffic light detection output rather than the perception obstacles output.

Then, the planning module takes the routing output. Under certain scenarios, the planning module might also trigger a new routing computation by sending a routing request if the current route cannot be faithfully followed.

Finally, the planning module needs to know the location (Localization: where I am) as well as the current autonomous vehicle information (Chassis: what is my status).  The planning module is also triggered by a fixed frequency, and the main data interface is the `OnTimer` callback function that invokes the `RunOnce` function.

![img](images/planning_1.png)

The data dependencies such as chassis, localization, traffic light, and prediction are managed through the `AdapterManager` class. The core software modules are similarly managed.  For example, localization is managed through `AdapterManager::GetLocalization()` as shown below.![img](images/planning_2.png)

## Control

As described in the planning module, control takes the planned trajectory as input, and generates the control command to pass to CanBus.  It has three main data interfaces: OnPad, OnMonitor, and OnTimer.

![img](images/control_1.png)

The `OnPad` and `OnMonitor` are routine interactions with the PAD-based human interface and simulations. The main data interface is the `OnTimer` interface, which periodically produces the actual control commands as shown below.

![img](images/control_2.png)

## CanBus

The CanBus has two data interfaces as shown below.

![img](images/canbus_1.png)

The first data interface is a timer-based publisher with the callback function `OnTimer`. This data interface periodically publishes the chassis information as well as chassis details, if enabled.

![img](images/canbus_2.png)

The second data interface is an event-based publisher with a callback function `OnControlCommand`, which is triggered when the CanBus module receives control commands.

## HMI

 Human Machine Interface or DreamView in Apollo is a web application that:
    -	visualizes the current output of relevant autonomous driving modules, e.g. planning trajectory, car localization, chassis status, etc.
    -	provides human-machine interface for user to view hardware status, turn on/off of modules, and start the autonomous driving car.
    -	provides debugging tools, such as PnC Monitor to efficiently track module issues.

## Monitor

The surveillance system of all the modules in the vehicle including hardware. Monitor receives Data from different modules and passes them on to HMI for the driver to view and ensure that all the modules are working without any issue. In the event of a module or hardware failure, monitor sends an alert to Guardian (new Action Center Module) which then decides on which action needs to be taken to prevent a crash.

## Guardian

This new module is basically an action center that takes a decision based on the data that is sent by Monitor. There are 2 main functions of Guardian:
- All modules working fine - Guardian allows the flow of control to work normally. Control signals are sent to CANBus as if Guardian were not present.
- Module crash is detected by Monitor - if there is a failure detected by Monitor, Guardian will prevent Control signals from reaching CANBus and bring the car to a stop. There are 3 ways in which Guardian decides how to stop the car, and to do so, Guardian turns to the final Gatekeeper, Ultrasonic sensors,
    - If the Ultrasonic sensor is running fine without detecting an obstacle, Guardian will bring the car to a slow stop 
    - If the sensor is not responding, Guardian applies a hard brake to bring the car to an immediate stop.
    - This is a special case, If the HMI informs the driver of an impending crash and the driver does not intervene for 10 seconds, Guardian applies a hard brake to bring the car to an immediate stop.

```
Note: 
1. In either case above, Guardian will always stop the car should Monitor detect a failure in any module or hardware.
2. Monitor and Guardian are decoupled to ensure that there is not a single point of failure and also that with a module approach, the action center can be modified to include additional actions without affecting the functioning of the surveillance system as Monitor also communicates with HMI.

```

