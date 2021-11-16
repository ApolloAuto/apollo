/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *implied. See the License for the specific language governing
 *permissions and limitations under the License.
 *****************************************************************************/

#pragma once

#include <deque>
#include <map>
#include <vector>
#include <string>

#include "modules/prediction/proto/vector_net.pb.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

using FeatureVector = std::vector<std::vector<std::vector<double>>>;
using PidVector = std::vector<std::vector<double>>;

enum ATTRIBUTE_TYPE {
  ROAD,
  LANE_UNKOWN,
  LANE_DOTTED_YELLOW,
  LANE_DOTTED_WHITE,
  LANE_SOLID_YELLOW,
  LANE_SOLID_WHITE,
  LANE_DOUBLE_YELLOW,
  LANE_CURB,
  JUNCTION,
  CROSSWALK,
};

enum BOUNDARY_TYPE {
  UNKNOW,
  NORMAL,
  LEFT_BOUNDARY,
  RIGHT_BOUNDARY,
};

class VectorNet {
 public:
  VectorNet() { apollo::hdmap::HDMapUtil::ReloadMaps(); }

  ~VectorNet() = default;

  bool query(const common::PointENU& center_point, const double obstacle_phi,
             FeatureVector* const feature_ptr, PidVector* const p_id_ptr);

  bool offline_query(const double obstacle_x, const double obstacle_y,
                     const double obstacle_phi);

  bool offline_query(const double obstacle_x, const double obstacle_y,
                     const double obstacle_phi, const std::string file_name);

 private:
  // TODO(Yiqun): 1.Left/Right boundary 2.Ordinal Encoding
  const std::map<ATTRIBUTE_TYPE, double> attribute_map{
      {ROAD, 0.0},
      {LANE_UNKOWN, 1.0},
      {LANE_DOTTED_YELLOW, 2.0},
      {LANE_DOTTED_WHITE, 3.0},
      {LANE_SOLID_YELLOW, 4.0},
      {LANE_SOLID_WHITE, 5.0},
      {LANE_DOUBLE_YELLOW, 6.0},
      {LANE_CURB, 7.0},
      {JUNCTION, 8.0},
      {CROSSWALK, 9.0},
  };

  const std::map<BOUNDARY_TYPE, double> boundary_map{
      {UNKNOW, 0.0}, {NORMAL, 1.0}, {LEFT_BOUNDARY, 2.0}, {RIGHT_BOUNDARY, 3.0},
  };

  const std::map<hdmap::LaneBoundaryType::Type, ATTRIBUTE_TYPE> lane_attr_map{
      {hdmap::LaneBoundaryType::UNKNOWN, LANE_UNKOWN},
      {hdmap::LaneBoundaryType::DOTTED_YELLOW, LANE_DOTTED_YELLOW},
      {hdmap::LaneBoundaryType::DOTTED_WHITE, LANE_DOTTED_WHITE},
      {hdmap::LaneBoundaryType::SOLID_YELLOW, LANE_SOLID_YELLOW},
      {hdmap::LaneBoundaryType::SOLID_WHITE, LANE_SOLID_WHITE},
      {hdmap::LaneBoundaryType::DOUBLE_YELLOW, LANE_DOUBLE_YELLOW},
      {hdmap::LaneBoundaryType::CURB, LANE_CURB},
  };

  template <typename Points>
  void GetOnePolyline(const Points& points, double* start_length,
                      const common::PointENU& center_point,
                      const double obstacle_phi, ATTRIBUTE_TYPE attr_type,
                      BOUNDARY_TYPE bound_type, const int count,
                      std::vector<std::vector<double>>* const one_polyline,
                      std::vector<double>* const one_p_id);

  void GetRoads(const common::PointENU& center_point, const double obstacle_phi,
                FeatureVector* const feature_ptr, PidVector* const p_id_ptr);

  void GetLaneQueue(
      const std::vector<hdmap::LaneInfoConstPtr>& lanes,
      std::vector<std::deque<hdmap::LaneInfoConstPtr>>* const lane_deque_ptr);

  void GetLanes(const common::PointENU& center_point, const double obstacle_phi,
                FeatureVector* const feature_ptr, PidVector* const p_id_ptr);
  void GetJunctions(const common::PointENU& center_point,
                    const double obstacle_phi, FeatureVector* const feature_ptr,
                    PidVector* const p_id_ptr);
  void GetCrosswalks(const common::PointENU& center_point,
                     const double obstacle_phi,
                     FeatureVector* const feature_ptr,
                     PidVector* const p_id_ptr);
  int count_ = 0;
};

}  // namespace prediction
}  // namespace apollo
