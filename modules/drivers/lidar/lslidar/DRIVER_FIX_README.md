# 镭神Apollo修改



### lslidar_component.cpp

~~~c++
void LslidarComponent::ScanQueuePollProcess()

// 更改判断条件，!apollo::cyber::IsShutdown()与 scan_queue_.popWait(scan_frame)分开
// 一起判断会导致点云发布信息异常
    
    
void LslidarComponent::HandleScanFrame(const std::shared_ptr<LslidarScan> &scan_frame)

// 增加时间设置
// timestamp_sec 为系统时间
// lidar_timestamp，measurement_time 为每帧最后一个点时间
~~~



### driver.cc

~~~c++
void LslidarDriver::Init()

// 增大LSLIDAR_LS128S2 packets_rate值，避免LS系列雷达实际包数比npackets大导致点云帧率翻倍
// 应该放弃scan->firing_pkts_size() < config_.npackets()判断，仅用包头或角度判断一帧起始
~~~



### lslidarCXV4_parser.cc

~~~c++
void LslidarCXV4Parser::Unpack(const LslidarPacket &pkt,std::shared_ptr<PointCloud> pc, int packet_number)
    
// 增加当前包的时间  current_packet_time = pkt.stamp(); 旧代码中没有给当包赋时间，导致机械式4.0雷达点的时间均为0
// 修改无效点，由于机械式是有序点云，所有线号点数需一致，无效点均赋值为nan。现在将机械式点云修改为无序点云，无效点直接continue过滤，不在添加进点云。

    
void LslidarCXV4Parser::Order(std::shared_ptr<PointCloud> cloud)
 
// 注释点云排序功能，无序点云不需要排序
~~~



### lslidarLS128S2_parser.cc

~~~c++
LslidarLS128S2Parser::LslidarLS128S2Parser(const Config &config)

// 更新LS系列雷达摆镜角度   旧: {0, -2, -1, -3}
// double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5}; // 新摆镜角度 
~~~





机械式点云改为无序点云，height为1，width为该帧点数。

增加多台雷达运行示例 lslidar_four.dag lslidar_four.launch

注释的AERROR日志可以删除，仅用于调试



后续可更新:

- 兼容新款雷达
- 增加LS系列雷达角度畸变矫正解析
- 点云旋转平移功能





