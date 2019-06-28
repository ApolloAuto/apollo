# Control

## Introduction
Based on the planning trajectory and the car's current status, the Control module uses different control algorithms to generate a comfortable driving experience. The Control module can work both in normal and navigation modes.

Apollo 5.0 Control has following new features:
 * **Low Speed / Reverse Driving**: The control module ensures accurate low-speed forward and backward driving, which supports complex and practical driving scenarios like valet parking and pull-over to name a few.
 * **RTK Recorder / Player Update**: The RTK recorder / player now supports both the forward and backward trajectory recording and re-play.


## Input
  * Planning trajectory
  * Car status
  * Localization
  * Dreamview AUTO mode change request

## Output
  * Control commands (steering, throttle, brake) to the chassis.
