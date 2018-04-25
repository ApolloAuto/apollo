# Apollo 2.5 map collection guide

This guide is mainly used to explain how to use map collection in Apollo 2.5.

## Hardware and Software Requirement
Please refer to
[Apollo 2.5 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)
for the steps to install the hardware components and the system software, as well as
[Apollo Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md)

Please refer to
[Apollo Sensor Calibration Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/multiple_lidar_gnss_calibration_guide.md)
for Sensor Calibration.


## Data Collection Steps
1、Enter Into Map Collection Mode.
Please refer to
[Apollo 2.5 Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_quick_start.md)
for launching the Apollo 2.5 software and hardware stack on vehicle.

Choose [Module Controller] tab, select [Map Collection] mode, and switch on [GPS]、[Camera]、[Velodyne]、[Velodyne16].

![](images/map_collection_sensor_open.png)

confirm whether the sensors are ready.

![](images/map_collection_sensor_check.png)

2、After sensors are all ready, switch on [Record Bag] to start recording the map data.

![](images/map_collection_sensor_start_record.png)

we should ensure the same road can be covered more than five times in the speed below 60KM/h. and take different lanes as far as possible in every cycle during the map collection.

4、After the collection is finished, switch off [Record Bag] firstly, and then switch off [GPS], [Camera], [Velodyne] and [Velodyne16].

![](images/map_collection_sensor_stop_record.png)

5、Data Upload

The collected map data is placed in the /apollo/data/bag/(start time of collection, e.g.,2018-04-14-21-20-24) directory by default, package them as tar.gz compressed file and upload them to the [Apollo Data Official Website](http://data.apollo.auto/hd_map_intro).