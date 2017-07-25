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

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/proto/map_id.pb.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::hdmap::LaneInfo;
using apollo::hdmap::Id;
using apollo::hdmap::MapPathPoint;

PredictionMap::PredictionMap() : hdmap_(nullptr) { LoadMap(); }

PredictionMap::~PredictionMap() { Clear(); }

void PredictionMap::LoadMap() {
  hdmap_.reset(new apollo::hdmap::HDMap());
  CHECK(hdmap_ != nullptr);
  hdmap_->load_map_from_file(FLAGS_map_file);
  ADEBUG << "Load map file: " << FLAGS_map_file;
}

void PredictionMap::Clear() { hdmap_.reset(); }

Id PredictionMap::id(const std::string& str_id) {
  Id id;
  id.set_id(str_id);
  return id;
}

Eigen::Vector2d PredictionMap::PositionOnLane(const LaneInfo* lane_info,
                                              const double s) {
  // TODO(kechxu) implement
  return Eigen::Vector2d(0.0, 0.0);
}

double PredictionMap::HeadingOnLane(const LaneInfo* lane_info, const double s) {
  // TODO(kechxu) implement
  return 0.0;
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
  return hdmap_->get_lane_by_id(id).get();
}

void PredictionMap::GetProjection(const Eigen::Vector2d& position,
                                  const LaneInfo* lane_info_ptr, double* s,
                                  double* l) {
  // TODO(kechxu) implement
}

bool PredictionMap::ProjectionFromLane(const LaneInfo* lane_info_ptr,
                                       MapPathPoint* path_point, double* s) {
  // TODO(kechxu) implement
  return true;
}

void PredictionMap::OnLane(const std::vector<const LaneInfo*>& prev_lanes,
                           const Eigen::Vector2d& point, const double heading,
                           const double radius,
                           std::vector<const LaneInfo*>* lanes) {
  std::vector<const LaneInfo*> candidate_lanes;
  // if (hdmap_->get_lanes_with_heading(
  //         point, radius, heading, M_PI, &candidate_lanes) != 0) {
  //   return;
  // }
  for (auto& candidate_lane : candidate_lanes) {
    if (candidate_lane == nullptr) {
      continue;  // TODO(kechxu) else if point is on candidate_lane
    } else if (!IsIdenticalLane(candidate_lane, prev_lanes) &&
               !IsSuccessorLane(candidate_lane, prev_lanes) &&
               !IsLeftNeighborLane(candidate_lane, prev_lanes) &&
               !IsRightNeighborLane(candidate_lane, prev_lanes)) {
      continue;
    } else {
      // MapPathPoint nearest_point = candidate_lane->get_nearest_point(point);
      // double diff = fabs(math_util::angle_diff(
      //                       heading, nearest_point.heading()));
      // if (diff <= FLAGS_max_lane_angle_diff) {
      //   lanes->emplace_back(candidate_lane);
      // }
    }
  }
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
