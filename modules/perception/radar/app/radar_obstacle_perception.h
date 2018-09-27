// Copyright 2018 Baidu Inc. All Rights Reserved.
#ifndef RADAR_APP_RADAR_OBSTACLE_PERCEPTION_H_
#define RADAR_APP_RADAR_OBSTACLE_PERCEPTION_H_

#include <string>
#include <vector>
#include <memory>

#include "modules/perception/radar/lib/interface/base_radar_obstacle_perception.h"

namespace apollo {
namespace perception {
namespace radar {
class RadarObstaclePerception : public BaseRadarObstaclePerception{
 public:
  RadarObstaclePerception() {}
  virtual ~RadarObstaclePerception() {}

  bool Init(const std::string pipeline_name) override;

  bool Perceive(const ContiRadar& corrected_obstacles,
                const RadarPerceptionOptions& options,
                std::vector<base::ObjectPtr>* objects) override;

  std::string Name() const override;

 private:
  std::shared_ptr<BaseDetector> detector_;
  std::shared_ptr<BaseRoiFilter> roi_filter_;
  std::shared_ptr<BaseTracker> tracker_;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
#endif  // RADAR_APP_RADAR_OBSTACLE_PERCEPTION_H_
