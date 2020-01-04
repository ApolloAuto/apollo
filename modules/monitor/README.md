# Monitor

## Introduction
This module contains system level software such as code to check hardware status
and monitor system health.
In Apollo 5.5, the monitor module now performs the following checks among others:
1. Status of running modules 
2. Monitoring data integrity 
3. Monitoring data frequency 
4. Monitoring system health (e.g. CPU, memory, disk usage, etc) 
5. Generating end-to-end latency stats report

```
Note: You can configure the modules that you would like to monitor for the first 3 capabilities mentioned above.
```

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
