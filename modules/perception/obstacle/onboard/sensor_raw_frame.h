#ifndef MODEULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_
#define MODEULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_

#include "Eigen/Core"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"

namespace apollo {
namespace perception {

class SensorRawFrame {
 public:
  SensorRawFrame()
      : sensor_type_(UNKNOWN_SENSOR_TYPE),
        timestamp_(0.0),
        pose_(Eigen::Matrix4d::Identity()) {}
  virtual ~SensorRawFrame() {}

 public:
  SensorType sensor_type_;
  double timestamp_;
  Eigen::Matrix4d pose_;
};

class VelodyneRawFrame : public SensorRawFrame {
 public:
  VelodyneRawFrame() {}
  ~VelodyneRawFrame() {}

 public:
  pcl_util::PointCloudPtr cloud_;
};

class RadarRawFrame : public SensorRawFrame {
 public:
  RadarRawFrame() {}
  ~RadarRawFrame() {}

 public:
  ContiRadar raw_obstacles_;
  Eigen::Vector3f car_linear_speed_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_OBSTACLE_ONBOARD_SENSOR_RAW_FRAME_H_
