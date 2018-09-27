// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui (huiyujiang@baidu.com)
// @file: base_radar_obstacle_perception.h
// @brief: radar radar obstacle  interface.
#ifndef RADAR_LIB_INTERFACE_BASE_RADAR_OBSTACLE_PERCEPTION_H_
#define RADAR_LIB_INTERFACE_BASE_RADAR_OBSTACLE_PERCEPTION_H_
// SAMPLE CODE:
//
// class DefaultRadarObstaclePerception : public BaseRadarObstaclePerception {
//  public:
//   DefaultRadarObstaclePerception() : BaseRadarObstaclePerception() {}
//   virtual ~DefaultRadarObstaclePerception() {}
//
//   virtual bool Init() override {
//     // Do something.
//     return true;
//   }
//
//   virtual bool Perceive(
//                const ContiRadar& corrected_obstacles,
//                const RadarPerceptionOptions& options,
//                std::vector<base::ObjectPtr>* objects) override {
//      // Do something.
//      return true;
//    }
//
//    virtual std::string Name() const override {
//        return "DefaultRadarObstaclePerception";
//    }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(DefaultRadarObstaclePerception);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseRadarObstaclePerception* radar_perception =
//    BaseRadarObstaclePerceptionRegisterer::GetInstanceByName("DefaultRadarObstaclePerception");
// using radar_perception to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>
#include "modules/perception/radar/lib/interface/base_detector.h"
#include "modules/perception/radar/lib/interface/base_preprocessor.h"
#include "modules/perception/radar/lib/interface/base_roi_filter.h"
#include "modules/perception/radar/lib/interface/base_tracker.h"

namespace apollo {
namespace perception {
namespace radar {
struct RadarPerceptionOptions {
  DetectorOptions detector_options;
  RoiFilterOptions roi_filter_options;
  TrackerOptions track_options;
  std::string sensor_name;
};
class BaseRadarObstaclePerception {
 public:
  BaseRadarObstaclePerception() = default;
  virtual ~BaseRadarObstaclePerception() = default;
  virtual bool Init(const std::string pipeline_name) = 0;
  virtual bool Perceive(const ContiRadar& corrected_obstacles,
                        const RadarPerceptionOptions& options,
                        std::vector<base::ObjectPtr>* objects) = 0;
  virtual std::string Name() const = 0;
};

PERCEPTION_REGISTER_REGISTERER(BaseRadarObstaclePerception);
#define PERCEPTION_REGISTER_RADAR_OBSTACLE_PERCEPTION(name) \
  PERCEPTION_REGISTER_CLASS(BaseRadarObstaclePerception, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_INTERFACE_BASE_RADAR_OBSTACLE_PERCEPTION_H_
