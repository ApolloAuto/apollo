# PnC模块
## 输入
* traffic_light
* routing
* pad
* relative_map
* story_telling
## 输出
* 给底盘的控制指令（油门/刹车，转向）
# Planning模块
## Planning中采用的为消息触发，消息触发的上游消息，定义在Component中Proc()函数的入参
1. Reader:
traffic_light
routing
pad
relative_map
story_telling
2. Writer
rerouting
adc_trajectory
planning_learning_data

## 数据结构
### LocalView结构体
包含了Plannning的所有上游输入数据：包含了预测障碍物及其轨迹、底盘信息、定位信息、交通灯信息、导航信息、相对地图等

planning/common/local_view.h
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */

struct LocalView {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
  std::shared_ptr<perception::TrafficLightDetection> traffic_light;
  std::shared_ptr<routing::RoutingResponse> routing;
  std::shared_ptr<relative_map::MapMsg> relative_map;
  std::shared_ptr<PadMessage> pad_msg;
  std::shared_ptr<storytelling::Stories> stories;
};

### ReferenceLineInfo类
包含了ReferenceLine类，为车辆行驶的参考线；
包含了Blocking_obstacles_, 即在该条参考线上阻塞车道的障碍物；
还有优化出的Path_data、Speed_data以及最终融合后的轨迹Discretized_trajectory。

planning/common/reference_line_info.h
private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;

  PathDecision path_decision_;

  Obstacle* blocking_obstacle_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  RSSInfo rss_info_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;


### ReferenceLine类
ReferenceLine类即为规划的参考线，其中包含了地图给出的原始中心线Map_path，以及后续优化的基准参考点，一系列的Reference_points。

planning/reference_line/reference_line.h
 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
  uint32_t priority_ = 0;
};

### Frame类
包含了Planning一次循环中所用到的所有信息，可以说这个数据结构贯穿了planning的主逻辑
LocalView
地图信息hd_map
规划起始点planning_start_point_
车辆状态信息vehicle_state_
ReferenceLineInfo信息
障碍物信息obstacles_
交通灯信息traffic_lights_

planning/common/frame.h
private:
  static DrivingAction pad_msg_driving_action_;
  uint32_t sequence_num_ = 0;
  LocalView local_view_;                                                                  
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;
  common::VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;

  std::unordered_map<std::string, const perception::TrafficLight *>
      traffic_lights_;

  // current frame published trajectory
  ADCTrajectory current_frame_planned_trajectory_;

  // current frame path for future possible speed fallback
  DiscretizedPath current_frame_planned_path_;

  const ReferenceLineProvider *reference_line_provider_ = nullptr;

  OpenSpaceInfo open_space_info_;

  std::vector<routing::LaneWaypoint> future_route_waypoints_;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 public:
  FrameHistory();
};

### PnC_map
Pnc_map即规划控制地图，在Plannning中属于独立的一块功能，其主要目的在于将Routing给到的原始地图车道信息转换成规划可以用的ReferenceLine信息。实现Pnc_map的功能分为两步：
1. 处理Routing结果，将routing数据处理并存储在map相关的数据结构中，供后续使用
2. 根据routing结果及当前车辆位置，计算可行驶的passage信息，并转换成RouteSegments结构

拿到Pnc_map的结果之后，下一步就要对原始的地图中心线Map_Path信息进行平滑

## control模块
## 输入

* 规划轨迹
* 车辆状态
* 定位
* Dreamview自动模式更改请求
## 输出

* 给底盘的控制指令（转向，节流，刹车）。

