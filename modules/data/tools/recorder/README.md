
```
 __| | __ _| |_ __ _   _ __ ___  ___ ___  _ __ __| | ___ _ __
 / _` |/ _` | __/ _` | | '__/ _ \/ __/ _ \| '__/ _` |/ _ \ '__|
| (_| | (_| | || (_| | | | |  __/ (_| (_) | | | (_| |  __/ |
 \__,_|\__,_|\__\__,_| |_|  \___|\___\___/|_|  \__,_|\___|_|
```

Data-Recorder is responsiable for helping Apollo partner to record data.

## v1.0.0.1

## How to `Use.`

#### Data-Recorder configuration.
There are recorder.global.yaml and recorder.debug.yaml in conf directory.
Modify the config file according to your system environments and data recording requirements.

#### Start Data-Recorder.
 * bash data_reocrder_control.sh start # Start data-recoder with default task_purpose(debug).
 * python modules/data/tools/recorder/data_recorder_manager.py -c modules/data/conf/recorder.debug.yaml # This is another way to start.

#### Stop Data-Recorder.
 * bash data-recorder_control.sh stop  # stop data-recorder.
 * CTRL + C if start data-recorder with python data_recorder_manager.py -c ../conf/conf.yaml

#### Send control commands to data-recorder.This feature depends on data-recorder has been started.
 * Send command rosbag_record_off to disable rosbag record.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "rosbag_record_off".

 * Send command rosbag_record_on to enable rosbag record.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "rosbag_record_on".

 * Send command data_sync_off to disable data sync.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "data_sync_off".

 * Send command data_sync_on to enable data sync.
 * rostopic pub /apollo/data_recorder/cmd --once std_msgs/String "data_sync_on".
 * Also user can write a rosnode to publish ros message to control data-recorder.

#### Subscribe  data-recorder status topic and get data-recorder running infomations.This feature depends on data-recorder has been started.
 * rostopic list  # List all the ros topics.                              
 * rostopic echo  /apollo/data_recorder/status # Read data from /apollo/data_recorder/status.
Try to use the following code to subscribe topic and get deserialized message.
```
    #!/usr/bin/env python
    import rospy
    import recorder_info_pb2
    from std_msgs.msg import String
    
    def callback(data):
        a = recorder_info_pb2.RecorderInfo()
        a.ParseFromString(data.data)
        print(a)
    
    def listener():
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/apollo/data_recorder/status", String, callback)
        rospy.spin()
    
    if __name__ == '__main__':
        listener() 
```


