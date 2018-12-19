### Common cyber commands



|                ROS                |      Cyber       |          Note          |
| :-------------------------------: | :--------------: | :--------------------: |
|              rosbag               |  cyber_recorder  |       data file        |
| scripts/diagnostics.sh (previous) |  cyber_monitor   |     channel debug      |
|   offline_lidar_visualizer_tool   | cyber_visualizer | point cloud visualizer |



### cyber Commands Samples

- record channel

```
cyber_recorder record /topic_name
```

- replay record file

```
cyber_recorder play recorder_filename
```

- view record file

```
cyber_recorder info recorder_filename
```

