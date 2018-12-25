## 引言
Lidar是一种常用的环境感知传感器，利用脉冲激光来照射目标并接收目标的反射脉冲，根据激光返回的时间来计算与目标的距离。通过对目标多次全方位的测量，可以得到目标环境的数字3D结构模型。Apollo平台默认支持velodyne 16线，32线，64线和128线等多种型号的lidar。该说明主要介绍Lidar驱动的主要功能以及如何在apollo平台中添加一款新的lidar设备驱动。

## Velodyne驱动的主要部分

1. [Driver](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/driver): 通过网络端口接收lidar硬件产生的UDP数据包，将每一帧封装成VelodyneScan格式后发送。

2. [Parser](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/parser): 接收VelodyneScan数据，把VelodyneScan中的点由球面坐标系转换成空间直角坐标系下的pointcldoud点云格式后发送。

3. [Compensator](https://github.com/ApolloAuto/apollo/tree/master/modules/drivers/velodyne/compensator): 接收点云数据和pose数据，根据每个点的对应的pose信息把点转换到点云中最大时刻对应的坐标系下，减小由车辆自身的运动带来的误差。需要点云数据中包含每个点的时间戳信息。



## 添加新lidar驱动的步骤

#### 1. 熟悉cyber框架

cyber框架下系统中每一个功能单元都可以抽象为一个component，通过channel相互间进行通信，然后根据dag(有向无环图)配置文件，构建成相应的pipeline，实现数据的流式处理。

#### 2. 消息定义
	
apollo已经预定义了点云的消息格式，所以只需要为新lidar定义一个存储原始扫描数据的proto消息，用于数据的存档和离线开发调试，相比于点云数据，存档原始数据可以大量节省存储空间。一个新的扫描数据消息可以类似如下定义:

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
在velodyne驱动中，其扫描数据消息定义为[VelodyneScan](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/proto/velodyne.proto#L29).
	
#### 3. 读取原始数据

lidar每秒会产生大量数据，一般通过UDP协议来进行数据的高效传输。编写一个DriverComponent类，继承于无模版参数Component类；在Init函数中启动一个异步poll线程，不断从相应的端口读取lidar数据；然后根据需求如将一段时间内的数据打包为一帧ScanData，如扫描一圈为一帧；最后通过writer将ScanData写至相应的channel发送出去。

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

#### 4. 解析扫描数据，生成点云。

编写一个Parser类，输入为一帧ScanData，根据lidar自己的数据协议，解析出每一个点的时间戳，x/y/z三维坐标，以及反射强度，并组合成一帧点云。每个点都位于以lidar为原点的FLU（Front: x, Left: y, Up: z）坐标系下。
	
```c++
message PointXYZIT {
  optional float x = 1 [default = nan];
  optional float y = 2 [default = nan];
  optional float z = 3 [default = nan];
  optional uint32 intensity = 4 [default = 0];
  optional uint64 timestamp = 5 [default = 0];
}
```
	
然后定义一个ParserComponent，继承于ScanData实例的Component模板类。接收ScanData消息，生成点云消息，发送点云消息。

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
#### 5. 对点云进行运行补偿

运动补偿是一个通用的点云处理过程，可以直接复用velodyne driver中compensator模块的算法逻辑。
	
#### 6. 配置dag文件
	
将各个数据处理环节定义为component后，需要将各个component组成一个lidar数据处理pipeline，如下配置lidar_driver.dag:
		
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

#### 7. 运行lidar驱动并查看点云
完成以步骤后，就可以通过以下命令来启动lidar驱动。

```bash
mainboard -d /path/to/lidar_driver.dag
```
此时通过`cyber_visualizer`选择对应的点云channel，就可以可视化查看点云了。
