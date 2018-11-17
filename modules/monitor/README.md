# Monitor

## Introduction
This module contains system level software such as code to check hardware status
and monitor system health.

## Hardware Monitors
Hardware related monitoring, e.g. CAN card / GPS status health check. Check
results are reported back to HMI.

## Software Monitors

### Process Monitor
It checks if a process is running or not. Config it with
apollo::monitor::ProcessConf proto, which works similar to
```bash
ps aux | grep <keyword1> | grep <keyword2> | ...
```

### Topic Monitor
It checks if a given topic is updated normally. Config it with
apollo::monitor::TopicConf proto.

### Summary Monitor
It summarizes all other specific monitor's results to a simple conclusion such
as OK, WARN, ERROR or FATAL.

## Reporters

### Static Information Reporter
It publishes the static information (apollo::data::StaticInfo) in very low
frequency, so you can get things like vehicle metadata or user information from
"rostopic echo" or a recorded rosbag.

### Vehicle State Reporter
It reports critical vehicle state (apollo::common::VehicleState) to a remote
endpoint, so you can monitor your running vehicles from the server. Note that
it's an advanced feature and disabled by default.
