
# Dreamview for Teleop

## Introduction
Dreamview module provides a web application that helps operate and visualize teleop feature.

## Teleop Services

### Send Audio Stream

### Send Mic Stream

### Send Video Stream

### Send Emergency Stop Command
- When "Stop" button is clicked, a pad message with `DrivingAction` of `STOP`, defined by Protobuf message `PadMessage` in file `planning/proto/pad_msg.proto`, will be puslished by **Dreamview**.
- Upon receiving a pad message with `DrivingAction` of `STOP`, **Planning** module will execute **emergency stop** by abruptly brakeing and stopping at the current route.

### Send Emergency Pull-over Command
- When "Pull Over" button is clicked, a pad message with `DrivingAction` of `PULL_OVER`, defined by Protobuf message `PadMessage` in file `planning/proto/pad_msg.proto`, will be puslished by **Dreamview**.
- Upon receiving a pad message with `DrivingAction` of `PULL_OVER`, **Planning** module will execute **emergency pull-over** by safely pulling over as soon as possible.

### Send Resume Cruise Command
- When "Resume Cruise" button is clicked, a pad message with `DrivingAction` of `RESUME_CRUISE`, defined by Protobuf message `PadMessage` in file `planning/proto/pad_msg.proto`, will be puslished by **Dreamview**.
- Upon receiving a pad message with `DrivingAction` of `RESUME_CRUISE`, **Planning** module will **resume cruising** based on current routing.


