/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/eval/lidar_option.h"
#include <string>
#include "modules/perception/tool/benchmark/lidar/base/point_cloud_frame.h"
#include "modules/perception/tool/benchmark/lidar/eval/frame_statistics.h"
#include "modules/perception/tool/benchmark/lidar/eval/position_metric.h"

namespace apollo {
namespace perception {
namespace benchmark {

bool LidarOption::set_options() const {
  auto iter = _options.find("JACCARD");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple JACCARD values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      FrameStatistics::set_jaccard_index_threshold(value);
      std::cerr << "Set jaccard index threshold: " << value << std::endl;
    }
  }
  iter = _options.find("JACCARD_PERCENTILE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr
          << "Confused by multiple JACCARD_PERCENTILE values, so use default"
          << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      FrameStatistics::set_jaccard_index_percentile(value);
      std::cerr << "Set jaccard index percentile: " << value << std::endl;
    }
  }
  iter = _options.find("RANGE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple RANGE values, so use default"
                << std::endl;
    } else {
      if (*iter->second.begin() == "view") {
        MetaStatistics::set_range_type(VIEW);
        std::cerr << "Set range division method: view" << std::endl;
      } else if (*iter->second.begin() == "box") {
        MetaStatistics::set_range_type(BOX);
        std::cerr << "Set range division method: box" << std::endl;
      } else if (*iter->second.begin() == "roi") {
        MetaStatistics::set_range_type(ROI);
        std::cerr << "Set range division method: roi" << std::endl;
      } else {
        MetaStatistics::set_range_type(DISTANCE);
        std::cerr << "Set range division method: distance" << std::endl;
      }
    }
  }
  iter = _options.find("IGNORE_OUTSIDE_ROI");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr
          << "Confused by multiple IGNORE_OUTSIDE_ROI values, so use default"
          << std::endl;
    } else {
      if (*iter->second.begin() == "true") {
        RoiDistanceBasedRangeInterface::set_ignore_roi_outside(true);
        std::cerr << "Set ignore roi outside: true" << std::endl;
      } else {
        RoiDistanceBasedRangeInterface::set_ignore_roi_outside(false);
        std::cerr << "Set ignore roi outside: false" << std::endl;
      }
    }
  }
  iter = _options.find("LABEL_BLACK_LIST");
  if (iter != _options.end()) {
    Frame::set_black_list(iter->second);
    std::cerr << "Set label black list: ";
    for (auto& value : iter->second) {
      std::cerr << value << " ";
    }
    std::cerr << std::endl;
  }
  iter = _options.find("OVERALL_DISTANCE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple overall distance, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      DistanceBasedRangeInterface::set_distance(value);
      std::cerr << "Set overall distance threshold: " << value << std::endl;
    }
  }
  iter = _options.find("FRONT_ANGLE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple front angle values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      ViewBasedRangeInterface::set_front_view_angle(value);
      std::cerr << "Set view front angle threshold: " << value << std::endl;
    }
  }
  iter = _options.find("REAR_ANGLE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple rear angle values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      ViewBasedRangeInterface::set_rear_view_angle(value);
      std::cerr << "Set view rear angle threshold: " << value << std::endl;
    }
  }
  iter = _options.find("FRONT_DISTANCE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple front distance values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      ViewBasedRangeInterface::set_front_view_distance(value);
      std::cerr << "Set view front distance threshold: " << value << std::endl;
      BoxBasedRangeInterface::set_front_box_distance(value);
      std::cerr << "Set box front distance threshold: " << value << std::endl;
    }
  }
  iter = _options.find("REAR_DISTANCE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple rear distance values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      ViewBasedRangeInterface::set_rear_view_distance(value);
      std::cerr << "Set view rear distance threshold: " << value << std::endl;
      BoxBasedRangeInterface::set_rear_box_distance(value);
      std::cerr << "Set box rear distance threshold: " << value << std::endl;
    }
  }
  iter = _options.find("SIDE_DISTANCE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple side distance values, so use default"
                << std::endl;
    } else {
      double value = std::stof(*iter->second.begin());
      BoxBasedRangeInterface::set_side_box_distance(value);
      std::cerr << "Set box side distance threshold: " << value << std::endl;
    }
  }
  iter = _options.find("CLOUD_TYPE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple cloud type values, so use default"
                << std::endl;
    } else {
      std::string type = *iter->second.begin();
      PointCloudFrame::set_cloud_type(type);
      std::cerr << "Set cloud type: " << type << std::endl;
    }
  }
  iter = _options.find("PENALIZE_PI");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple penalize pi flag, so use default"
                << std::endl;
    } else {
      std::string is_penalize_pi = *iter->second.begin();
      if (is_penalize_pi == "true") {
        OrientationSimilarityMetric::penalize_pi = true;
      } else if (is_penalize_pi == "false") {
        OrientationSimilarityMetric::penalize_pi = false;
      }
      std::cerr << "Set penalize pi flag: " << is_penalize_pi << std::endl;
    }
  }
  iter = _options.find("RECALL_DIM");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple recall dim values, so use default"
                << std::endl;
    } else {
      unsigned int value = std::stoi(*iter->second.begin());
      MetaStatistics::set_recall_dim(value);
      std::cerr << "Set recall dim: " << value << std::endl;
    }
  }
  iter = _options.find("VISIBLE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple visible values, so use default"
                << std::endl;
    } else {
      float value = std::stof(*iter->second.begin());
      Frame::set_visible_threshold(value);
      std::cerr << "Set visible threshold: " << value << std::endl;
    }
  }
  iter = _options.find("CONFIDENCE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple confidence values, so use default"
                << std::endl;
    } else {
      float value = std::stof(*iter->second.begin());
      Frame::set_min_confidence(value);
      std::cerr << "Set min confidence: " << value << std::endl;
    }
  }
  iter = _options.find("ROI_TYPE");
  if (iter != _options.end()) {
    if (iter->second.size() > 1) {
      std::cerr << "Confused by multiple roi type values, so use default"
                << std::endl;
    } else {
      bool value = (*iter->second.begin() == "LANE");
      FrameStatistics::set_roi_is_main_lanes(value);
      std::cerr << "Set roi_is_main_lanes: " << value << std::endl;
    }
  }

  return true;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
