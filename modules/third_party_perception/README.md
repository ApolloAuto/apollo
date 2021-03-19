# Third Party Perception

## Introduction
In Apollo 2.5, the third_party_perception module incorporates third-party sensors like Mobileye and Conti/Delphi Radar output with simple fusion and creates a similar perception output produced as obstacle/lane detection information as defined in [Perception Obstacles Interface](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/proto/perception_obstacle.proto). This module was only intend to serve for the Prediction/Planning/Control algorithm in real vehicle before perception modules fully ready before 2.5. We recommend using 'modules/perception' instead for your own test purpose after Apollo 2.5 officially released.

## Input

The perception module inputs are:

- Radar data (ROS topic _/apollo/sensor/conti_radar_ or _/apollo/sensor/delphi_esr_ )
- Mobileye data (ROS topic _/apollo/sensor/mobileye_)

## Output

The perception module outputs are:

* The 3D obstacle tracks with the heading, velocity and classification information (ROS topic _/apollo/perception/obstacles_)
* The lane marker information with fitted curve parameter, spatial information(l0,r0, etc) as well as semantic information (lane type) (ROS topic _/apollo/perception/obstacles_)
