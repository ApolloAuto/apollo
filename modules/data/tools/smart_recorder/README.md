# Smart Recorder

## Introduction
Smart Recorder is targeting to reduce the recording data size.  Instead of recording all the topics all the time, it selectively records by the following way.

1. All the small topics all the time.  These include "/apollo/localization/pose", "/apollo/canbus/chassis" and so on, which are samll in size.
2. Large topics only in specified scenarios.  These include sensor data include all "PointCloud" and "Camera" topics, which are large in size.  The specific scenarios are configurable, as well as the time range for recording when the specified scenario occurs.


## How to use

1. Build apollo
2. python3 /apollo/scripts/record_message.py --help


## How to add new scenarios

1. Configure the new scenario in conf/smart_recorder_config.pb.txt, including time range, name, description and etc.
2. Add new class inherit from base class "TriggerBase", and implement interface "Pull"
3. Add the instance of the new class into triggers pool inside function "ProcessRecord::InitTriggers"