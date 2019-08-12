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
#pragma once
#include <string>
#include "modules/perception/tool/benchmark/lidar/util/object.h"

namespace apollo {
namespace perception {
namespace benchmark {

struct PositionMetricOption {
  // TODO(...): add hdmap, pose, ...
  bool roi_is_main_lanes = false;
};

struct PositionMetric {
  void cal_position_metric(const ObjectPtr& object,
                           const PositionMetricOption& option);

  double radial_distance = 0.0;
  double horizontal_distance = 0.0;
  double vertical_distance = 0.0;
  double angle = 0.0;
  bool is_in_roi = false;

  bool is_valid = false;
};

class BaseRangeInterface {
 public:
  virtual unsigned int get_index(const PositionMetric& position) const = 0;
  virtual unsigned int get_dim() const = 0;
  virtual std::string get_element(unsigned int index) const = 0;
};

class DistanceBasedRangeInterface : public BaseRangeInterface {
 public:
  unsigned int get_index(const PositionMetric& position) const override;
  unsigned int get_dim() const override;
  std::string get_element(unsigned int index) const override;

 public:
  static void set_distance(double distance);

 protected:
  static double _s_distance;
  static double _s_half_distance;
};

// 0~30, 30~60, 60~120, 120~200, 200~
class DistanceBasedRangeRadarInterface : public BaseRangeInterface {
 public:
  unsigned int get_index(const PositionMetric& position) const override;
  unsigned int get_dim() const override;
  std::string get_element(unsigned int index) const override;
};

class ViewBasedRangeInterface : public BaseRangeInterface {
 public:
  unsigned int get_index(const PositionMetric& position) const override;
  unsigned int get_dim() const override;
  std::string get_element(unsigned int index) const override;

 public:
  static void set_front_view_angle(double angle);
  static void set_rear_view_angle(double angle);
  static void set_front_view_distance(double distance);
  static void set_rear_view_distance(double distance);

 private:
  static double _s_front_view_angle;
  static double _s_rear_view_angle;
  static double _s_front_view_distance;
  static double _s_rear_view_distance;
};

class BoxBasedRangeInterface : public BaseRangeInterface {
 public:
  unsigned int get_index(const PositionMetric& position) const override;
  unsigned int get_dim() const override;
  std::string get_element(unsigned int index) const override;

 public:
  static void set_front_box_distance(double distance);
  static void set_rear_box_distance(double distance);
  static void set_side_box_distance(double distance);

 private:
  static double _s_front_box_distance;
  static double _s_rear_box_distance;
  static double _s_side_box_distance;
};

class RoiDistanceBasedRangeInterface : public DistanceBasedRangeInterface {
 public:
  unsigned int get_index(const PositionMetric& position) const override;
  unsigned int get_dim() const override;
  std::string get_element(unsigned int index) const override;

 public:
  static void set_ignore_roi_outside(bool ignore);

 protected:
  static bool _s_ignore_roi_outside;
};

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
