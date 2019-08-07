# Sensor Calibration

## ins_stat_publisher.py

Cause GNSS device NEWTON M2 can't generate topic "/apollo/sensor/gnss/ins_stat", this tool is used to publish a fake "/apollo/sensor/gnss/ins_stat"   

Run the following command from your Apollo root dir:

```
python modules/tools/sensor_calibration/ins_stat_publisher.py
```

## odom_publisher.py

Cause GNSS device NEWTON M2 can't generate topic "/apollo/sensor/gnss/odometry", this tool is used to publish topic "/apollo/sensor/gnss/odometry" through subscribe topic "/apollo/localization/pose" 

Run the following command from your Apollo root dir:

```
python modules/tools/sensor_calibration/odom_publisher.py
```

