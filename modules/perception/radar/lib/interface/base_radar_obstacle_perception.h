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
//                const drivers::ContiRadar& corrected_obstacles,
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

#pragma once

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
  virtual bool Init(const std::string& pipeline_name) = 0;
  virtual bool Perceive(const drivers::ContiRadar& corrected_obstacles,
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
