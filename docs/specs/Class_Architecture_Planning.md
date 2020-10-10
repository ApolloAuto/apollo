# Class Architecture and Overview -- Planning Module

## Data Output and Input

### Output Data

The planning output data is defined in `planning.proto`, as shown below.

![img](images/class_architecture_planning/image001.png)

#### *planning.proto*

Inside the proto data definition, the planning output includes the total planned time and length, as well as the actual trajectory for control to execute, and is defined in `repeated apollo.common.TrajectoryPointtrajectory_point`.  

A trajectory point is derived from a `path_point`, where speed, acceleration and timing attributes are added. Each `trajectory_point` as defined in `pnc_point.proto`, and contains detailed attributes of the trajectory.

![img](images/class_architecture_planning/image002.png)

![img](images/class_architecture_planning/image003.png)

In addition to the trajectory, the planning module also outputs rich annotation information. The important data fields are:

- Estop
- DecisionResult
- Debug information

`Estop` is a command that indicates errors and exceptions. For example, when the autonomous vehicle collides with an obstacle or cannot obey traffic rules, estop signals are sent. The `DecisionResult` data is used mainly for simulation display so that developers have a better understanding of the planning results. More detailed numerical intermediate results are stored in the debug information and sent out for debugging purposes.

## Input Data

To compute the final published trajectory, the planning module leverages various input data sources. The planning input data sources are:

- Routing
- Perception and Prediction
- Vehicle Status and Localization
- HD-Map

Routing defines the query concept “where I want to go” for the autonomous vehicle, and the message is defined in `routing.proto`. The `RoutingResponse` contains the `RoadSegment`, which identifies the road to follow or the lanes to use to reach the destination as shown below.

![img](images/class_architecture_planning/image004.png)

The messages regarding the query concept  “what is surrounding me” are defined mainly in `perception_obstacles.proto` and `traffic_light_detection.proto`. The `perception_obstacles.proto` defines the obstacles perceived by the perception module around the autonomous vehicle, while `traffic_light_detection` defines the perceived traffic light statuses (if any). In addition to the perceived obstacles, what is important for the planning module are the predicted trajectories for each perceived dynamic obstacle. Therefore, the `prediction.proto` wraps the `perception_obstacle` message with a predicted trajectory, as shown below.

![img](images/class_architecture_planning/image005.png)

Each predicted trajectory has a probability associated with it, and one obstacle might have multiple predicted trajectories. 

In addition to the query concepts “where I want to go” and “what is surrounding me”, another important query concept is “where am I”. Such information is obtained from the HD-Map and Localization modules. Both localization and vehicle chassis information are incorporated in the messages of `VehicleState` that is defined in the `vehicle_state.proto`, as shown below.

![img](images/class_architecture_planning/image009.png)

## Code Structure and Class Hierarchy

The code is organized as follows: The planning code entrance is the `planning.cc`. Inside the planner, the important class members are shown in the illustration below.

![img](images/class_architecture_planning/image006.png)

The `ReferenceLineInfo` is a wrapper of the `ReferenceLine` class, which represents a smoothed guideline for planning. 

**Frame** contains all the data dependencies including the perceived obstacles with their predicted trajectories, and the current status of the autonomous vehicle. 

**HD-Map** is leveraged as a library inside the planning module for ad-hoc fashioned map queries. 

**EM Planner** does the actual planning and derives from the **Planner** class. Both the Em Planner that is used in the Apollo 2.0 release, and the previously released **RTK Planner** derive from the Planner class.

![img](images/class_architecture_planning/image007.png)

For example, inside a planning cycle performed by the EM Planner, take an iterative approach where three categories of tasks interweave. The relationships of these “**decider/optimizer**” classes are illustrated below.

![img](images/class_architecture_planning/image008.png)

- **Deciders** include traffic decider, path decider and speed decider.

- **Path Optimizers** are the DP/QP path optimizers.

- **Speed Optimizers** are the DP/QP speed optimizers.


| **NOTE:**                                |
| ---------------------------------------- |
| DP means dynamic programming while QP means quadratic programming. After the computation, the final spatio-temporal trajectory is then discretized and published so that the downstream control module is able to execute it. |
