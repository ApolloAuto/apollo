# Apollo 2.5 map collection guide

This guide is mainly used to explain how to use map collection in Apollo 2.5.

## Hardware and Software Requirement
1. Please refer to
[Apollo 2.5 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)
for the steps to install the hardware components.

2. Please refer to
[Apollo Software Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_software_installation_guide.md)
for steps to install system software.

3. Please refer to
[Apollo Sensor Calibration Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/multiple_lidar_gnss_calibration_guide.md)
for Sensor Calibration.

4. NVMe SSD. In order to avoid the possible data loss problem caused by IO bottleneck, it is recommended to install NVME SSD hard disk in IPC.

5. GPS signal. In order to get accurate mapping results, we need to ensure that the GPS signal is good and RTK can work normally in the whole acquisition process.

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

Before the map collection, the vehicle needs to be stationary for five minutes, and then circles the ribbon in figure eight for five minutes.
During the map collection, we should ensure the same road can be covered more than five times in the speed below 60KM/h. and take different lanes as far as possible in every cycle.
After the map collection is completed, the vehicle also needs to circle the ribbon in figure eight for five minutes, and to remain stationary for five minutes.

4、After the collection is finished, switch off [Record Bag] firstly, and then switch off [GPS], [Camera], [Velodyne] and [Velodyne16].

![](images/map_collection_sensor_stop_record.png)

5、Data Upload

The collected map data is placed in the /apollo/data/bag/(start time of collection, e.g.,2018-04-14-21-20-24) directory by default, package them as tar.gz compressed file and upload them to the [Apollo Data Official Website](http://data.apollo.auto/hd_map_intro/?locale=en-us).