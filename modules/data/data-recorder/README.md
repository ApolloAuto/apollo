
```
 __| | __ _| |_ __ _   _ __ ___  ___ ___  _ __ __| | ___ _ __
 / _` |/ _` | __/ _` | | '__/ _ \/ __/ _ \| '__/ _` |/ _ \ '__|
| (_| | (_| | || (_| | | | |  __/ (_| (_) | | | (_| |  __/ |
 \__,_|\__,_|\__\__,_| |_|  \___|\___\___/|_|  \__,_|\___|_|
```

Data-Recorder is responsiable for helping Apollo partner to record data.

## v1.0.0.1

## How to `use`

#### Start Data-Recorder.
 * bash data_reocrder_control.sh start # Start data-recoder with default task_purpose(debug).
 * python data_recorder_manager.py -c ../conf/recorder.debug.yaml # This is another way to start.

#### Stop Data-Recorder.
 * bash data-recorder_control.sh stop  # stop data-recorder.
 * CTRL + C if start data-recorder with python data_recorder_manager.py -c ../conf/conf.yaml

#### Send control commands to data-recorder.This feature depends on data-recorder has been started.
 * Send ROS message with rostopic pub. command rosbag_record_off means stop rosbag record.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "rosbag_record_off".

 * Send ROS message with rostopic pub. command rosbag_record_on means start rosbag record.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "rosbag_record_on".

 * Write ROS node to publish ROS message control data-recorder.

#### Subscribe  data-recorder status topic and get data-recorder running infomations.This feature depends on data-recorder has been started.
 * rostopic list  # List all the ros topics.                              
 * rostopic echo  /apollo/data_recorder/status # Read data from /apollo/data_recorder/status.
