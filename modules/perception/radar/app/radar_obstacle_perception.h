/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/radar/lib/interface/base_radar_obstacle_perception.h"

namespace apollo {
namespace perception {
namespace radar {
class RadarObstaclePerception : public BaseRadarObstaclePerception {
 public:
  RadarObstaclePerception() {}
  virtual ~RadarObstaclePerception() {}

  bool Init(const std::string& pipeline_name) override;

  bool Perceive(const drivers::ContiRadar& corrected_obstacles,
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
