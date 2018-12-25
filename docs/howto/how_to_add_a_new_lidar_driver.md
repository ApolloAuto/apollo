# How to add a new Lidar driver

## Introduction

Lidar is a commonly used environment-aware sensor. Lidar uses pulsed laser to illuminate a target, receives the reflected pulse from the target, then calculates the distance to the target based on the time of laser return. Differences in multiple measurements can then be used to make digital 3-D representations of the environment.

As default, Apollo platform support multiple types of Lidar drivers, including 16 channels, 32 channels, 64 channels and 128 channels Velodyne lidars. This manual describes the major functions of Lidar driver and how to add a new lidar driver in Apollo platform.

## What's inside a lidar driver

Taking velodyne lidar driver as an example, there are three major components:

1. [Driver](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/driver): Driver receives UDP data packets from lidar sensor, and packages the data packets into a frame of scanning data in the format of VelodyneScan. VelodyneScan is defined in file below:
```
modules/drivers/velodyne/proto/velodyne.proto
```

2. [Parser](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/parser): Parser takes one frame data in format of VelodyneScan as input, converts the cloud points in the frame from spherical coordinate system to Cartesian coordinates system, then sends out the point cloud as output. The pointcloud format is defined in file below:
```
modules/drivers/proto/pointcloud.proto
```

3. [Compensator](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/compensator): Compensator takes pointcloud data and pose data as inputs. Based on the corresponding pose information for each cloud point, it converts each cloud point information aligned with the latest time in the current lidar scan frame, minimizing the motion error due the movement of the vehicle. Thus, each cloud point needs carry its own timestamp information.

## Steps to add a new Lidar driver

#### 1. Get familiar with Apollo Cyber RT framework. 

Please refer to the [manuals of Apollo Cyber RT](https://github.com/ApolloAuto/apollo/tree/master/docs/cyber).


#### 2. Define message for raw data

Apollo already define the format of pointcloud. For new lidar, you only need to define the protobuf message for the raw scannning data. Those raw data will be used for archive and offline development. Compared to processed pointcloud data, raw data saves a lot of storage spaces for long term. The new message of the scan data can be define as below:

```c++
// a scan message sample
message ScanData {
	optional apollo.common.Header header = 1;  // apollo header
	optional Model model = 2;                  // device model
	optional Mode mode = 3;                    // work mode
	// device serial number, corresponds to a specific calibration file
	optional string sn = 4;
	repeated bytes raw_data = 5;               // raw scan data
}
```

In velodyne driver, the scan data message is define as [VelodyneScan](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/proto/velodyne.proto#L29).

#### 3. Access the raw data

Each seconds, Lidar will generate a lot of data, so it relied on UDP to efficiently transport the raw data. You need to create a DriverComponent class, which inherits the Component withotu any parameter. In its Init function, you need to start a async polling thread, whic will receive Lidar data from the specific port. Then depending on the Lidar's frequency, the DriverComponent needs to package all the packets in a fix period into a frame of ScanData. Eventually, the writer will send the ScanData through a corresponding channel.

```c++
// Inherit component with no template parameters, 
// do not receive message from any channel
class DriverComponent : public Component<> {
 public:
  ~VelodyneDriverComponent();
  bool Init() override {
  	poll_thread_.reset(new thread([this]{
  		this->Poll();
  	}));
  }
	
 private: 
  void Poll() {
  	while (apollo::cyber::Ok()) {
  	  // poll data from port xxx
  	  // ...
  	  austo scan = std::make_shared<ScanData>();
  	  // pack ScanData
  	  // ...
  	  writer_.write(scan);
  	}
  }
   
  std::shared_ptr<std::thread> poll_thread_;
  std::shared_ptr<apollo::cyber::Writer<ScanData>> writer_;
};
	
CYBER_REGISTER_COMPONENT(DriverComponent)
```

#### 4. Parse the scan data, convert to pointcloud

 If the new lidar driver already provides the pointcloud data in Cartesian coordinates system, then you just need to store those data in the protobuf format defined in Apollo.

The Parser converts the lidar raw data to the pointcloud format in Cartesian coordinates system. Parser takes ScanData as input. For each cloud point, it will parse the timestamp, x/y/z coordinates and intensity, then packages all the cloudpoint information into a frame of pointcloud. Each cloud point transformed into the FLU (Front: x, Left: y, Up: z）coordinates with Lidar as the origin point.
	
```c++
message PointXYZIT {
  optional float x = 1 [default = nan];
  optional float y = 2 [default = nan];
  optional float z = 3 [default = nan];
  optional uint32 intensity = 4 [default = 0];
  optional uint64 timestamp = 5 [default = 0];
}
```
	
Then you need to create a new ParserComponent，which inherits Components templates with ScanData. ParserComponent takes ScanData as input, then generates pointcloud message and sents it out.

```c++
...
class ParserComponent : public Component<ScanData> {
 public:
  bool Init() override {
  	...
  }
  
  bool Proc(const std::shared_ptr<ScanData>& scan_msg) override {
    // get a pointcloud object from objects pool
  	auto point_cloud_out = point_cloud_pool_->GetObject(); 
  	// clear befor using
  	point_cloud_out->clear();	
  	// parse scan data and generate pointcloud
  	parser_->parse(scan_msg, point_cloud_out);
  	// write pointcloud to a specific channel
  	writer_->write(point_cloud);
  }
	
 private:
  std::shared_ptr<Writer<PointCloud>> writer_;
  std::unique_ptr<Parser> parser_ = nullptr;
  
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr; 
  int pool_size_ = 8;
};
	
CYBER_REGISTER_COMPONENT(ParserComponent)
```

#### 5. Motion compensation for pointcloud

Motion compensation is optional depends on lidar hardware design. E.g. if the the pointcloud information from lidar already have the motion error included, then no compensator needed as extra steps. Otherwise, you need your own compensator. However, if each cloud point in your lidar's output carries its own timestamp information, you can probably reuse the current compensator without any changes.

#### 6. Configure the dag file

After done with each component, you just need to configure the DAG config file to add each component into the data processing pipeline. E.g.  lidar_driver.dag:
		
```python
# Define all coms in DAG streaming.
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/xxx/driver/libxxx_driver_component.so"
    components {
      class_name : "DriverComponent"
      config {
        name : "xxx_driver"
        config_file_path : "/path/to/lidar_driver_conf.pb.txt"
      }
    }
}
	
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/xxx/parser/libxxx_parser_component.so"
    components {
      class_name : "ParserComponent"
      config {
        name : "xxx_parser"
        config_file_path : "/path/to/lidar_parser_conf.pb.txt"
        readers { channel: "/apollo/sensor/xxx/Scan" }
      }
    }
}
	
module_config {
    module_library : "/apollo/bazel-bin/modules/drivers/xxx/compensator/libxxx_compensator_component.so"
    components {
      class_name : "CompensatorComponent"
      config {
        name : "pointcloud_compensator"
        config_file_path : "/apollo/modules/drivers/xxx/conf/xxx_compensator_conf.pb.txt"
        readers {channel: "/apollo/sensor/xxx/PointCloud2"}
      }
    }
}
```

#### 7. Run the lidar driver and visualize the pointlcoud output

After finishing all the previous steps, you can use the following command to start your new lidar driver.

```bash
mainboard -d /path/to/lidar_driver.dag
```
To visualize the pointcloud output, you can run `cyber_visualizer` and choose the right channel for the pointcloud.
