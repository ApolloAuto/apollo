# Control

## Introduction
Based on the planning trajectory and the car's current status, the Control module uses different control algorithms to generate a comfortable driving experience. The Control module can work both in normal and navigation modes.

Apollo 5.5 Control has following new features:
* **Model Reference Adaptive Control (MRAC)**: This new algorithm is designed to effectively offset the by-wire steering dynamic delay and time latency. It also ensures faster and more accurate steering control actions for tracking the planning trajectory. With MRAC, Apollo 5.5 Control now supports fast and sharp turning scenarios like successive side-pass. 
* **Control Profiling Service**: This service provides a systematic quantitative analysis of the vehicle's control performance based on road-tests or simulation data. The Profiling service not only helps support the high-efficiency iterative design cycle of the vehicle's control systems but also provides users with insights into the current performance and improvements seen within the module. 


## Input
  * Planning trajectory
  * Car status
  * Localization
  * Dreamview AUTO mode change request

## Output
  * Control commands (steering, throttle, brake) to the chassis.
