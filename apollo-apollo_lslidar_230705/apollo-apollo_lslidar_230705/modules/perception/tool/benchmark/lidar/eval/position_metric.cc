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

#include "modules/perception/tool/benchmark/lidar/eval/position_metric.h"

#include <algorithm>
#include <vector>

#include "absl/strings/str_cat.h"

namespace apollo {
namespace perception {
namespace benchmark {

void PositionMetric::cal_position_metric(const ObjectPtr& object,
                                         const PositionMetricOption& option) {
  if (object->cloud->points.empty()) {
    return;
  }
  std::vector<double> distance;
  Eigen::Vector2d center(0.0, 0.0);
  distance.reserve(object->cloud->size());
  for (auto& point : object->cloud->points) {
    distance.push_back(sqrt(point.x * point.x + point.y * point.y));
    center(0) += point.x;
    center(1) += point.y;
  }
  center /= static_cast<int>(object->cloud->size());
  std::sort(distance.begin(), distance.end());
  unsigned int sample1 = 0;
  unsigned int sample2 =
      static_cast<unsigned int>(0.2 * static_cast<int>(distance.size()));

  radial_distance = 0.5 * (distance[sample1] + distance[sample2]);
  horizontal_distance = fabs(center(1));
  vertical_distance = center(0);
  angle =
      acos(center.dot(Eigen::Vector2d(1, 0)) / center.norm()) * 180.0 / M_PI;
  is_in_roi =
      option.roi_is_main_lanes ? object->is_in_main_lanes : object->is_in_roi;
  is_valid = true;
}

double DistanceBasedRangeInterface::_s_distance = 60.0;
double DistanceBasedRangeInterface::_s_half_distance = 30.0;

void DistanceBasedRangeInterface::set_distance(double distance) {
  _s_distance = distance;
  _s_half_distance = 0.5 * distance;
}

unsigned int DistanceBasedRangeInterface::get_index(
    const PositionMetric& position) const {
  // unsigned int index = static_cast<unsigned int>(position.radial_distance /
  // 30.0);
  // index = index > 2 ? 2 : index;
  // return index;
  unsigned int index =
      static_cast<unsigned int>(position.radial_distance / _s_half_distance);
  index = index > 2 ? 2 : index;
  return index;
}

unsigned int DistanceBasedRangeInterface::get_dim() const { return 3; }

std::string DistanceBasedRangeInterface::get_element(unsigned int index) const {
  if (index >= get_dim()) {
    return "Total";
  } else {
    switch (index) {
      case 0:
        return absl::StrCat("0 - ", static_cast<int>(_s_half_distance), "m");
      case 1:
        return absl::StrCat(static_cast<int>(_s_half_distance), " - ",
                            static_cast<int>(_s_distance), "m");
      default:
        return absl::StrCat(static_cast<int>(_s_distance), " - 500m");
    }
  }
}

unsigned int DistanceBasedRangeRadarInterface::get_index(
    const PositionMetric& position) const {
  // unsigned int index = static_cast<unsigned int>(position.radial_distance /
  // 30.0);
  // index = index > 2 ? 2 : index;
  // return index;
  unsigned int distance = static_cast<unsigned int>(position.radial_distance);
  if (distance < 30) {
    return 0;
  } else if (distance < 60) {
    return 1;
  } else if (distance < 120) {
    return 2;
  } else if (distance < 200) {
    return 3;
  } else {
    return 4;
  }
}

unsigned int DistanceBasedRangeRadarInterface::get_dim() const { return 5; }

std::string DistanceBasedRangeRadarInterface::get_element(
    unsigned int index) const {
  if (index >= get_dim()) {
    return "Total";
  } else {
    switch (index) {
      case 0:
        return "  0 - 30m";
      case 1:
        return " 30 - 60m";
      case 2:
        return " 60 - 120m";
      case 3:
        return " 120 - 200m";
      default:
        return "200 - 500m";
    }
  }
}

double ViewBasedRangeInterface::_s_front_view_angle = 60.0;
double ViewBasedRangeInterface::_s_rear_view_angle = 60.0;
double ViewBasedRangeInterface::_s_front_view_distance = 60.0;
double ViewBasedRangeInterface::_s_rear_view_distance = 60.0;

void ViewBasedRangeInterface::set_front_view_angle(double angle) {
  _s_front_view_angle = angle;
}

void ViewBasedRangeInterface::set_rear_view_angle(double angle) {
  _s_rear_view_angle = angle;
}

void ViewBasedRangeInterface::set_front_view_distance(double distance) {
  _s_front_view_distance = distance;
}

void ViewBasedRangeInterface::set_rear_view_distance(double distance) {
  _s_rear_view_distance = distance;
}

unsigned int ViewBasedRangeInterface::get_index(
    const PositionMetric& position) const {
  if (position.radial_distance >= 60.0) {
    return 3;
  }
  // if (position.angle <= _s_front_view_angle * 0.5) {
  //    return 0;
  //} else if (position.angle >= 180.0 - _s_rear_view_angle * 0.5) {
  //    return 1;
  //} else if (position.horizontal_distance <= 30.0) {
  //    return 2;
  //} else {
  //    return 3;
  //}
  if (position.angle <= _s_front_view_angle * 0.5 &&
      position.radial_distance <= _s_front_view_distance) {
    return 0;
  } else if (position.angle >= 180.0 - _s_rear_view_angle * 0.5 &&
             position.radial_distance <= _s_rear_view_distance) {
    return 1;
  } else if (position.horizontal_distance <= 30 &&
             position.radial_distance <= 60) {
    return 2;
  }
  return 3;
}

unsigned int ViewBasedRangeInterface::get_dim() const { return 4; }

std::string ViewBasedRangeInterface::get_element(unsigned int index) const {
  if (index >= get_dim()) {
    return "Total";
  } else {
    switch (index) {
      case 0:
        return absl::StrCat("Front(", static_cast<int>(_s_front_view_angle),
                            "d", static_cast<int>(_s_front_view_distance),
                            "m)");
      case 1:
        return absl::StrCat("Rear(", static_cast<int>(_s_rear_view_angle), "d",
                            static_cast<int>(_s_rear_view_distance), "m)");
      case 2:
        return "Side(30m)";
      default:
        return "Outside  ";
    }
  }
}

double BoxBasedRangeInterface::_s_front_box_distance = 120.0;
double BoxBasedRangeInterface::_s_rear_box_distance = 45.0;
double BoxBasedRangeInterface::_s_side_box_distance = 10.0;

void BoxBasedRangeInterface::set_front_box_distance(double distance) {
  _s_front_box_distance = distance;
}

void BoxBasedRangeInterface::set_rear_box_distance(double distance) {
  _s_rear_box_distance = distance;
}

void BoxBasedRangeInterface::set_side_box_distance(double distance) {
  _s_side_box_distance = distance;
}

unsigned int BoxBasedRangeInterface::get_index(
    const PositionMetric& position) const {
  if (position.vertical_distance <= _s_front_box_distance &&
      position.vertical_distance >= -_s_rear_box_distance &&
      position.horizontal_distance <= _s_side_box_distance) {
    return 0;
  } else {
    return 1;
  }
}

unsigned int BoxBasedRangeInterface::get_dim() const { return 2; }

std::string BoxBasedRangeInterface::get_element(unsigned int index) const {
  if (index >= get_dim()) {
    return "Total";
  } else {
    switch (index) {
      case 0:
        return absl::StrCat("Box(F", static_cast<int>(_s_front_box_distance),
                            "R", static_cast<int>(_s_rear_box_distance), "S",
                            static_cast<int>(_s_side_box_distance), ")");
      case 1:
      default:
        return "Outside-box";
    }
  }
}

bool RoiDistanceBasedRangeInterface::_s_ignore_roi_outside = false;

void RoiDistanceBasedRangeInterface::set_ignore_roi_outside(bool ignore) {
  _s_ignore_roi_outside = ignore;
}

unsigned int RoiDistanceBasedRangeInterface::get_index(
    const PositionMetric& position) const {
  unsigned int index =
      static_cast<unsigned int>(position.radial_distance / _s_half_distance);
  if (_s_ignore_roi_outside) {
    index = (index > 2 || !position.is_in_roi) ? 2 : index;
  } else {
    index = index > 1 ? 4 : index;
    if (index != 4) {
      index = position.is_in_roi ? index : index + 2;
    }
  }
  return index;
}

unsigned int RoiDistanceBasedRangeInterface::get_dim() const {
  return _s_ignore_roi_outside ? 3 : 5;
}

std::string RoiDistanceBasedRangeInterface::get_element(
    unsigned int index) const {
  if (index >= get_dim()) {
    return "Total";
  }
  if (_s_ignore_roi_outside) {
    switch (index) {
      case 0:
        return absl::StrCat("I.0-", static_cast<int>(_s_half_distance), "m");
      case 1:
        return absl::StrCat("I.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      default:
        return "Outside";
    }
  } else {
    switch (index) {
      case 0:
        return absl::StrCat("I.0-", static_cast<int>(_s_half_distance), "m");
      case 1:
        return absl::StrCat("I.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      case 2:
        return absl::StrCat("O.0-", static_cast<int>(_s_half_distance), "m");
      case 3:
        return absl::StrCat("O.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      default:
        return absl::StrCat(static_cast<int>(_s_distance), "-500m");
    }
  }
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
