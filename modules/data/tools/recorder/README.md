```
 __| | __ _| |_ __ _   _ __ ___  ___ ___  _ __ __| | ___ _ __
 / _` |/ _` | __/ _` | | '__/ _ \/ __/ _ \| '__/ _` |/ _ \ '__|
| (_| | (_| | || (_| | | | |  __/ (_| (_) | | | (_| |  __/ |
 \__,_|\__,_|\__\__,_| |_|  \___|\___\___/|_|  \__,_|\___|_|
```

Data-Recorder is responsiable for helping Apollo partners to record data.

## v1.0.0.1

## How to `Use.`

#### Data-Recorder configuration.
Modify modules/data/conf/recorder.global.yaml and modules/data/conf/recorder.debug.yaml according to your system environments and data recording requirements.
Examples:

```modules/data/conf/recorder.global.yaml```</br>
vehicle_id and output_path are two necessary items that should be modified.
```
# vehicle_id is the UID of vehicle. If VIN is VIN001, export CARID=VIN001 or set vehicle_id in modules/data/conf/recorder.global.yaml as below:

vehicle_id: VIN001
```

```
# If a portable hard disk is mounted on /media/apollo/data_storage, execute sudo chown -R apollo:apollo /media/apollo/data_storage, and set output_path as below:

output_path: /media/apollo/data_storage
```

```modules/data/conf/debug.global.yaml```</br>
If you want to record rosbag group be topics, please refer to the following confituration:
```
   rosbag_topic_group:
   - group_id: '1'
     group_name: 'default'
     group_topic_match_re: ''
     group_topic_exclude_re: '(.*(rosout|image_long|image_short|image_narrow|image_wide|PointCloud2)$)'
   - group_id: '2'
     group_name: 'test'
     group_topic_match_re: '/apollo/data_recorder/status'
     group_topic_exclude_re: ''

```

If you want to copy something from system to record data in portable hard disk, please refer to the following confituration:
```
    carversion:                       # date type.
      if_record: true                 # if record.
      record_method: rsync            # record method, defalt value is rsync, do not modify.
      data_property:                  #
        src: "/home/apollo/version/"  # the source directory of your system.
        dst: "carversion/"            # the destination directory of record data.
      action_args:
        trigger_interval: 900         # copy interval(second).
        sync_bwlimit: 102400          # max speed of copy (KB/s).
        with_remove: false            # remove files after copying.
```

#### Start Data-Recorder.
 * bash modules/data/tools/recorder/data_reocrder_control.sh start # Start data-recoder with default task_purpose(debug).
 * python modules/data/tools/recorder/data_recorder_manager.py -c modules/data/conf/recorder.debug.yaml # This is another way to start.

#### Stop Data-Recorder.
 * bash modules/data/tools/recorder/data-recorder_control.sh stop  # stop data-recorder.
 * CTRL + C if start data-recorder with python data_recorder_manager.py -c modules/data/conf/recorder.debug.yaml.

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
