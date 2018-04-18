# Apollo 2.5地图采集功能使用指南

本文档主要用来说明如何在Apollo2.5中使用地图数据采集的功能.重点介绍了数据采集所需的软硬件环境,数据采集的流程和注意事项.

## 软硬件环境准备
硬件安装方法参见[Apollo 2.5硬件安装指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)


软件安装方法参见[Apollo 软件安装指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide_cn.md)


传感器标定方法参见[Apollo 传感器标定方法使用指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/multiple_lidar_gnss_calibration_guide.md)


## 数据采集流程

1、启动地图采集模式
Apollo环境启动参见[Apollo 2.5快速上手指南](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_quick_start_cn.md)

选择[Module Controller]、[Map Collection],打开[GPS]、[Camera]、[Velodyne]、[Velodyne16]开关。

![](images/map_collection_sensor_open.png)

确认各个传感器状态是否OK.

![](images/map_collection_sensor_check.png)

2、待确认各个传感器状态OK后，打开[Record Bag]开关，开始录制地图数据。

![](images/map_collection_sensor_start_record.png)

采集过程中需要保证双向车道全覆盖采集五圈以上，车速60KM/h以下，尽量每圈走不同的车道，覆盖完全。在路口区域无需刻意停留，慢速通过即可。

4、数据采集完成后，关闭[Record Bag]开关结束采集，然后关闭[GPS]、[Camera]、[Velodyne]、[Velodyne16]开关。

![](images/map_collection_sensor_stop_record.png)

5、数据上传

采集的数据放置在/apollo/data/bag/(采集开始时间,例如2018-04-14-21-20-24)目录，把该目录下的数据打包为tar.gz压缩文件，到[Apollo数据官网](http://data.apollo.auto/hd_map_intro)进行数据上传。