# Sensor Calibration

## ins_stat_publisher.py

This tool is used to publish a fake "/apollo/sensor/gnss/ins_stat"

Run the following command from your Apollo root dir:

```
python modules/tools/sensor_calibration/ins_stat_publisher.py
```

## odom_publisher.py

This tool is used to publish topic "/apollo/sensor/gnss/odometry" through subscribe topic "/apollo/localization/pose" 

Run the following command from your Apollo root dir:

```
python modules/tools/sensor_calibration/odom_publisher.py
```

