/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_

#include <string>
#include <vector>

#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/modest/radar_track_manager.h"
#include "modules/perception/obstacle/radar/modest/object_builder.h"

namespace apollo {
namespace perception {

class ModestRadarDetector : public BaseRadarDetector {
 public:
  ModestRadarDetector() : BaseRadarDetector() {}
  ~ModestRadarDetector() = default;

  bool Init() override;

  bool Detect(const ContiRadar &raw_obstacles,
              const std::vector<PolygonDType> &map_polygons,
              const RadarDetectorOptions &options,
              std::vector<ObjectPtr> *objects) override;

  bool collect_radar_result(std::vector<ObjectPtr> *objects);

  std::string name() const override {
    return "ModestRadarDetector";
  }

 private:
  void roi_filter(const std::vector<PolygonDType> &map_polygons,
    std::vector<ObjectPtr>& filter_objects);

  // for unit test
  bool result_init_ = true;
  bool result_detect_ = true;

  bool use_had_map_;
  double max_theta_;
  bool use_fp_filter_;
  int delay_frames_;
  ContiParams conti_params_;
  ObjectBuilder object_builder_;
  boost::shared_ptr<RadarTrackManager> radar_tracker_;
  FRIEND_TEST(ModestRadarDetectorTest, modest_radar_detector_test);
  DISALLOW_COPY_AND_ASSIGN(ModestRadarDetector);
};

REGISTER_RADARDETECTOR(ModestRadarDetector);
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_MODEST_RADAR_DETECTOR_H_
