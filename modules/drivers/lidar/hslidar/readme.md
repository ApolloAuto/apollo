# hesai 2.0 sdk apollo适配驱动

## 工程简介

基于禾赛开源sdk 2.0驱动适配的apollo版本驱动程序。

## 配置

默认配置文件是AT128的配置文件，路径`/apollo/modules/drivers/lidar/hslidar/conf/hslidar.pb.txt`

1. 更改输出配置，包括frame_id和点云、scan数据输出通道
2. 更改输入配置，包括设备地址`device_ip`、udp端口`udp_port`、ptc端口`ptc_port`。
3. 更改两个校正文件路径`correction_file_path`和`firetimes_path`，校正文件需要从官网下载。
4. 更改lidar型号`lidar_type`，默认为"AT128"
5. 更改`source_type`，1对应一般lidar输入，2对应pcap输入，3对应scan数据输入。使用scan数据输入需要先将config_base中的source_type调成RAW_PACKET，再将配置文件里source_type调成3

## 运行

**所有驱动均需要在Apollo Docker环境下运行**

```sh
mainboard -d /apollo/modules/drivers/lidar/hslidar/dag/hslidar.dag
```

或

```sh
cyber_launch start /apollo/modules/drivers/lidar/hslidar/launch/hslidar.launch
```