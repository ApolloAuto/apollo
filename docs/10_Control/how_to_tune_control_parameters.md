
# How to Tune Control Parameters

## Introduction
The objective of the control module is to generate control commands to the vehicle based on planning trajectory and the current car status.

## Background

### Input / Output

#### Input
* Planning trajectory
* Current car status
* HMI driving mode change request
* Monitor System

#### Output
The output control command governs functions such as steering, throttle, and brake in the `canbus`.

### Controller Introduction
The controllers include a lateral controller that manages the steering commands and a longitudinal controller that manages the throttle and brakes commands .

#### Lateral Controller
The lateral controller is an LQR-Based Optimal Controller. The dynamic model of this controller is a simple bicycle model with side slip. It is divided into two categories, including a closed loop and an open loop.

- The closed loop provides discrete feedback LQR controller with 4 states:
  - Lateral Error
  - Lateral Error Rate
  - Heading Error
  - Heading Error Rate
- The open loop utilizes the path curvature information to cancel the constant steady state heading error.


#### Longitudinal Controller
The longitudinal controller is configured as a cascaded PID + Calibration table. It is divided into two categories, including a closed loop and an open loop.

- The closed loop is a cascaded PID (station PID + Speed PID), which takes the following data as controller input:
  - Station Error
  - Speed Error
- The open loop provides a calibration table which maps acceleration onto throttle/brake percentages.


## Controller Tuning

### Useful tools
Tool like [diagnostics](../../modules/tools/diagnostics) and [realtime_plot](../../modules/tools/realtime_plot) are useful for controller tuning and can be found under `apollo/modules/tools/`.
### Lateral Controller Tuning
The lateral controller is designed for minimal tuning effort.  The basic lateral controller tuning steps for *all* vehicles are:

1. Set all elements in `matrix_q` to zero.

2. Increase the third element of `matrix_q`, which defines the heading error weighting, to minimize the heading error.

3. Increase the first element of `matrix_q`,  which defines the lateral error weighting to minimize the lateral error.

#### Tune a Lincoln MKZ

For a Lincoln MKZ,  there are four elements that refer to the diagonal elements of the state weighting matrix Q:

- Lateral error weighting
- Lateral error rate weighting
- Heading error weighting
- Heading error rate weighting

Tune the weighting parameters by following the basic lateral controller tuning steps listed above in *Lateral Controller Tuning*.  An example is shown below.

```
lat_controller_conf {
  matrix_q: 0.05
  matrix_q: 0.0
  matrix_q: 1.0
  matrix_q: 0.0
}
```

#### Tune Additional Vehicle Types

When tuning a vehicle type that is other than a Lincoln MKZ, first update the vehicle-related physical parameters as shown in the example below.  Then, follow the basic lateral controller tuning steps listed above in *Lateral Controller Tuning* and define the matrix Q parameters.

An example is shown below.
```
lat_controller_conf {
  cf: 155494.663
  cr: 155494.663
  wheelbase: 2.85
  mass_fl: 520
  mass_fr: 520
  mass_rl: 520
  mass_rr: 520
  eps: 0.01
  steer_transmission_ratio: 16
  steer_single_direction_max_degree: 470
}
```

### Longitudinal Controller Tuning
The longitudinal controller is composed of Cascaded PID controllers that include one station controller and a high/low speed controller with different gains for different speeds.  Apollo manages tuning in open loop and closed loop by:

- OpenLoop: Calibration table generation. Please refer to [how_to_update_vehicle_calibration.md](../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E6%A0%87%E5%AE%9A/how_to_update_vehicle_calibration.md) for detailed steps.
- Closeloop: Based on the order of High Speed Controller -> Low Speed Controller -> Station Controller.

#### High/Low Speed Controller Tuning

The high speed controller code is used mainly to track the desired speed above a certain speed value.  For example:

```
high_speed_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 1.0
  ki: 0.3
  kd: 0.0
}
```
1.  First set the values for `kp`, `ki`, and `kd` to zero.
2.  Then start to increase `kp` to reduce the rising time of step response to velocity changes.
3.  Finally, increase `ki` to reduce velocity controller steady state error.

Once you obtain relatively accurate speed tracking performance for the higher speed, you can start tuning the low speed PID controller for a comfortable acceleration rate from the start point.

 ```
 low_speed_pid_conf {
   integrator_enable: true
   integrator_saturation_level: 0.3
   kp: 0.5
   ki: 0.3
   kd: 0.0
 }
 ```
*Note:*  Apollo usually sets the switch speed to be a coasting speed when the gear switches to *Drive*.

#### Station Controller Tuning

Apollo uses the station controller for the vehicle to track the station error between the vehicle trajectory reference and the vehicle position.  A station controller tuning example is shown below.
```
station_pid_conf {
  integrator_enable: true
  integrator_saturation_level: 0.3
  kp: 0.3
  ki: 0.0
  kd: 0.0
}
```
## References
1. "Vehicle dynamics and control." Rajamani, Rajesh. Springer Science & Business Media, 2011.

2. "Optimal Trajectory generation for dynamic street scenarios in a Frenet
   Frame", M. Werling et., ICRA2010
