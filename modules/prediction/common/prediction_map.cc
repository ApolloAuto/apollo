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

#include <string>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <memory>

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using apollo::hdmap::Id;
using apollo::hdmap::MapPathPoint;

PredictionMap::PredictionMap() : hdmap_(nullptr) {
  LoadMap();
}

PredictionMap::~PredictionMap() { Clear(); }

void PredictionMap::LoadMap() {
  hdmap_.reset(new apollo::hdmap::HDMap());
  CHECK(hdmap_ != nullptr);
  hdmap_->load_map_from_file(FLAGS_map_file);
  AINFO << "Succeeded in loading map file: " << FLAGS_map_file << ".";
}

void PredictionMap::Clear() { hdmap_.reset(); }

Id PredictionMap::id(const std::string& str_id) {
  Id id;
  id.set_id(str_id);
  return id;
}

Eigen::Vector2d PredictionMap::PositionOnLane(const LaneInfo* lane_info,
                                              const double s) {
  apollo::hdmap::Point point = lane_info->get_smooth_point(s);
  return {point.x(), point.y()};
}

double PredictionMap::HeadingOnLane(const LaneInfo* lane_info, const double s) {
  const std::vector<double>& headings = lane_info->headings();
  const std::vector<double>& accumulated_s = lane_info->accumulate_s();
  CHECK(headings.size() == accumulated_s.size());
  size_t size = headings.size();
  if (size == 0) {
    return 0.0;
  }
  if (size == 1) {
    return headings[0];
  }
  const auto low_itr =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  CHECK(low_itr != accumulated_s.end());
  size_t index = low_itr - accumulated_s.begin();
  if (index == size - 1) {
    return headings.back();
  }
  return apollo::common::math::slerp(
             headings[index], accumulated_s[index],
             headings[index + 1], accumulated_s[index + 1], s);
}

double PredictionMap::LaneTotalWidth(
    const apollo::hdmap::LaneInfo* lane_info_ptr, const double s) {
  double left = 0.0;
  double right = 0.0;
  lane_info_ptr->get_width(s, &left, &right);
  return left + right;
}

const LaneInfo* PredictionMap::LaneById(const Id& id) {
  return hdmap_->get_lane_by_id(id).get();
}

const LaneInfo* PredictionMap::LaneById(const std::string& str_id) {
  Id id;
  id.set_id(str_id);
  return LaneById(id);
}

void PredictionMap::GetProjection(const Eigen::Vector2d& position,
                                  const LaneInfo* lane_info_ptr, double* s,
                                  double* l) {
  if (lane_info_ptr == nullptr) {
    return;
  }
  apollo::common::math::Vec2d pos(position[0], position[1]);
  lane_info_ptr->get_projection(pos, s, l);
}

bool PredictionMap::ProjectionFromLane(const LaneInfo* lane_info_ptr,
                                       double s, MapPathPoint* path_point) {
  if (lane_info_ptr == nullptr) {
    return false;
  }
  apollo::hdmap::Point point = lane_info_ptr->get_smooth_point(s);
  double heading = HeadingOnLane(lane_info_ptr, s);
  path_point->set_x(point.x());
  path_point->set_y(point.y());
  path_point->set_heading(heading);
  return true;
}

void PredictionMap::OnLane(const std::vector<const LaneInfo*>& prev_lanes,
                           const Eigen::Vector2d& point, const double heading,
                           const double radius,
                           std::vector<const LaneInfo*>* lanes) {
  std::vector<std::shared_ptr<const LaneInfo>> candidate_lanes;
  // TODO(kechxu) clean the messy code of this function
  apollo::hdmap::Point hdmap_point;
  hdmap_point.set_x(point[0]);
  hdmap_point.set_y(point[1]);
  apollo::common::math::Vec2d vec_point;
  vec_point.set_x(point[0]);
  vec_point.set_y(point[1]);
  if (hdmap_->get_lanes_with_heading(
          hdmap_point, radius, heading, M_PI, &candidate_lanes) != 0) {
    return;
  }
  for (auto candidate_lane : candidate_lanes) {
    if (candidate_lane == nullptr) {
      continue;
    } else if (!candidate_lane->is_on_lane(vec_point)) {
      continue;
    } else if (!IsIdenticalLane(candidate_lane.get(), prev_lanes) &&
               !IsSuccessorLane(candidate_lane.get(), prev_lanes) &&
               !IsLeftNeighborLane(candidate_lane.get(), prev_lanes) &&
               !IsRightNeighborLane(candidate_lane.get(), prev_lanes)) {
      continue;
    } else {
      apollo::hdmap::Point nearest_point =
          candidate_lane->get_nearest_point(vec_point);
      double nearest_point_heading =
          PathHeading(candidate_lane.get(), nearest_point);
      double diff = std::fabs(
          apollo::common::math::AngleDiff(heading, nearest_point_heading));
      if (diff <= FLAGS_max_lane_angle_diff) {
        AINFO << "insert candidate lane";
        lanes->emplace_back(candidate_lane.get());
      }
    }
  }
}

double PredictionMap::PathHeading(const LaneInfo* lane_info_ptr,
                               const apollo::hdmap::Point& point) {
  apollo::common::math::Vec2d vec_point;
  vec_point.set_x(point.x());
  vec_point.set_y(point.y());
  double s = -1.0;
  double l = 0.0;
  lane_info_ptr->get_projection(vec_point, &s, &l);
  return HeadingOnLane(lane_info_ptr, s);
}

int PredictionMap::SmoothPointFromLane(
    const apollo::hdmap::Id& id, const double s, const double l,
    Eigen::Vector2d* point, double* heading) {
  // TODO(kechxu) Double check this implement
  if (point == nullptr || heading == nullptr) {
    return -1;
  }
  const LaneInfo* lane = LaneById(id);
  apollo::hdmap::Point hdmap_point = lane->get_smooth_point(s);
  *heading = PathHeading(lane, hdmap_point);
  point->operator[](0) = hdmap_point.x() - std::sin(*heading) * l;
  point->operator[](1) = hdmap_point.y() + std::cos(*heading) * l;
  return 0;
}

void PredictionMap::NearbyLanesByCurrentLanes(
    const Eigen::Vector2d& point,
    const std::vector<const apollo::hdmap::LaneInfo*>& lanes,
    std::vector<const apollo::hdmap::LaneInfo*>* nearby_lanes) {
  std::unordered_set<std::string> lane_ids;
  for (auto& lane_ptr : lanes) {
    if (lane_ptr == nullptr) {
      continue;
    }
    for (auto& lane_id : lane_ptr->lane().left_neighbor_forward_lane_id()) {
      if (lane_ids.find(lane_id.id()) != lane_ids.end()) {
        continue;
      }
      lane_ids.insert(lane_id.id());
      const LaneInfo* nearby_lane = LaneById(lane_id);
      nearby_lanes->push_back(nearby_lane);
    }
    for (auto& lane_id : lane_ptr->lane().right_neighbor_forward_lane_id()) {
      if (lane_ids.find(lane_id.id()) != lane_ids.end()) {
        continue;
      }
      lane_ids.insert(lane_id.id());
      const LaneInfo* nearby_lane = LaneById(lane_id);
      nearby_lanes->push_back(nearby_lane);
    }
  }
}

bool PredictionMap::IsLeftNeighborLane(const LaneInfo* left_lane,
                                       const LaneInfo* curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (left_lane == nullptr) {
    return false;
  }
  for (auto& left_lane_id : curr_lane->lane().left_neighbor_forward_lane_id()) {
    if (id_string(left_lane) == left_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsLeftNeighborLane(
    const LaneInfo* left_lane, const std::vector<const LaneInfo*>& lanes) {
  if (lanes.size() == 0) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsLeftNeighborLane(left_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsRightNeighborLane(const LaneInfo* right_lane,
                                        const LaneInfo* curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (right_lane == nullptr) {
    return false;
  }
  for (auto& right_lane_id :
       curr_lane->lane().right_neighbor_forward_lane_id()) {
    if (id_string(right_lane) == right_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsRightNeighborLane(
    const LaneInfo* right_lane, const std::vector<const LaneInfo*>& lanes) {
  if (lanes.size() == 0) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsRightNeighborLane(right_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsSuccessorLane(const LaneInfo* succ_lane,
                                    const LaneInfo* curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (succ_lane == nullptr) {
    return false;
  }
  for (auto& successor_lane_id : curr_lane->lane().successor_id()) {
    if (id_string(succ_lane) == successor_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsSuccessorLane(const LaneInfo* succ_lane,
                                    const std::vector<const LaneInfo*>& lanes) {
  if (lanes.size() == 0) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsSuccessorLane(succ_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsPredecessorLane(const LaneInfo* pred_lane,
                                      const LaneInfo* curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (pred_lane == nullptr) {
    return false;
  }
  for (auto& predecessor_lane_id : curr_lane->lane().predecessor_id()) {
    if (id_string(pred_lane) == predecessor_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsPredecessorLane(
    const LaneInfo* pred_lane, const std::vector<const LaneInfo*>& lanes) {
  if (lanes.size() == 0) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsPredecessorLane(pred_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsIdenticalLane(const LaneInfo* other_lane,
                                    const LaneInfo* curr_lane) {
  if (curr_lane == nullptr || other_lane == nullptr) {
    return false;
  }
  return id_string(other_lane) == id_string(curr_lane);
}

bool PredictionMap::IsIdenticalLane(const LaneInfo* other_lane,
                                    const std::vector<const LaneInfo*>& lanes) {
  if (lanes.size() == 0) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsIdenticalLane(other_lane, lane)) {
      return true;
    }
  }
  return false;
}

int PredictionMap::LaneTurnType(const Id& id) {
  const LaneInfo* lane = LaneById(id);
  if (lane != nullptr) {
    return static_cast<int>(lane->lane().turn());
  }
  return 1;
}

int PredictionMap::LaneTurnType(const std::string& lane_id) {
  return LaneTurnType(id(lane_id));
}

}  // namespace prediction
}  // namespace apollo
