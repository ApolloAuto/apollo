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

#include "modules/prediction/common/prediction_map.h"

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <utility>

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfo;
using apollo::hdmap::LaneInfo;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::OverlapInfo;
using apollo::hdmap::PNCJunctionInfo;

bool PredictionMap::Ready() { return HDMapUtil::BaseMapPtr() != nullptr; }

Eigen::Vector2d PredictionMap::PositionOnLane(
    const std::shared_ptr<const LaneInfo> lane_info, const double s) {
  common::PointENU point = lane_info->GetSmoothPoint(s);
  return {point.x(), point.y()};
}

double PredictionMap::HeadingOnLane(
    const std::shared_ptr<const LaneInfo> lane_info, const double s) {
  return lane_info->Heading(s);
}

double PredictionMap::CurvatureOnLane(const std::string& lane_id,
                                      const double s) {
  std::shared_ptr<const hdmap::LaneInfo> lane_info = LaneById(lane_id);
  if (lane_info == nullptr) {
    AERROR << "Null lane_info ptr found";
    return 0.0;
  }
  return lane_info->Curvature(s);
}

double PredictionMap::LaneTotalWidth(
    const std::shared_ptr<const hdmap::LaneInfo> lane_info, const double s) {
  double left = 0.0;
  double right = 0.0;
  lane_info->GetWidth(s, &left, &right);
  return left + right;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneById(
    const std::string& str_id) {
  return HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(str_id));
}

std::shared_ptr<const JunctionInfo> PredictionMap::JunctionById(
    const std::string& str_id) {
  return HDMapUtil::BaseMap().GetJunctionById(hdmap::MakeMapId(str_id));
}

std::shared_ptr<const OverlapInfo> PredictionMap::OverlapById(
    const std::string& str_id) {
  return HDMapUtil::BaseMap().GetOverlapById(hdmap::MakeMapId(str_id));
}

bool PredictionMap::GetProjection(
    const Eigen::Vector2d& pos, const std::shared_ptr<const LaneInfo> lane_info,
    double* s, double* l) {
  if (lane_info == nullptr) {
    return false;
  }
  return lane_info->GetProjection({pos.x(), pos.y()}, s, l);
}

bool PredictionMap::HasNearbyLane(const double x, const double y,
                                  const double radius) {
  common::PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  HDMapUtil::BaseMap().GetLanes(point_enu, radius, &lanes);
  return (!lanes.empty());
}

bool PredictionMap::ProjectionFromLane(
    std::shared_ptr<const LaneInfo> lane_info, const double s,
    MapPathPoint* path_point) {
  if (lane_info == nullptr) {
    return false;
  }
  const common::PointENU point = lane_info->GetSmoothPoint(s);
  const double heading = HeadingOnLane(lane_info, s);
  path_point->set_x(point.x());
  path_point->set_y(point.y());
  path_point->set_heading(heading);
  return true;
}

bool PredictionMap::IsVirtualLane(const std::string& lane_id) {
  std::shared_ptr<const LaneInfo> lane_info =
      HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(lane_id));
  if (lane_info == nullptr) {
    return false;
  }
  const hdmap::Lane& lane = lane_info->lane();
  bool left_virtual = lane.has_left_boundary() &&
                      lane.left_boundary().has_virtual_() &&
                      lane.left_boundary().virtual_();
  bool right_virtual = lane.has_right_boundary() &&
                       lane.right_boundary().has_virtual_() &&
                       lane.right_boundary().virtual_();
  return left_virtual && right_virtual;
}

bool PredictionMap::OnVirtualLane(const Eigen::Vector2d& point,
                                  const double radius) {
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  common::PointENU hdmap_point;
  hdmap_point.set_x(point[0]);
  hdmap_point.set_y(point[1]);
  HDMapUtil::BaseMap().GetLanes(hdmap_point, radius, &lanes);
  for (const auto& lane : lanes) {
    if (IsVirtualLane(lane->id().id())) {
      return true;
    }
  }
  return false;
}

void PredictionMap::OnLane(
    const std::vector<std::shared_ptr<const LaneInfo>>& prev_lanes,
    const Eigen::Vector2d& point, const double heading, const double radius,
    const bool on_lane, const int max_num_lane,
    const double max_lane_angle_diff,
    std::vector<std::shared_ptr<const LaneInfo>>* lanes) {
  std::vector<std::shared_ptr<const LaneInfo>> candidate_lanes;

  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  if (HDMapUtil::BaseMap().GetLanesWithHeading(hdmap_point, radius, heading,
                                               max_lane_angle_diff,
                                               &candidate_lanes) != 0) {
    return;
  }

  std::vector<std::pair<std::shared_ptr<const LaneInfo>, double>> lane_pairs;
  for (const auto& candidate_lane : candidate_lanes) {
    if (candidate_lane == nullptr) {
      continue;
    }
    if (on_lane && !candidate_lane->IsOnLane({point.x(), point.y()})) {
      continue;
    }
    if (!FLAGS_use_navigation_mode &&
        !IsIdenticalLane(candidate_lane, prev_lanes) &&
        !IsSuccessorLane(candidate_lane, prev_lanes) &&
        !IsLeftNeighborLane(candidate_lane, prev_lanes) &&
        !IsRightNeighborLane(candidate_lane, prev_lanes)) {
      continue;
    }
    double distance = 0.0;
    common::PointENU nearest_point =
        candidate_lane->GetNearestPoint({point.x(), point.y()}, &distance);
    double nearest_point_heading = PathHeading(candidate_lane, nearest_point);
    double diff =
        std::fabs(common::math::AngleDiff(heading, nearest_point_heading));
    if (diff <= max_lane_angle_diff) {
      lane_pairs.emplace_back(candidate_lane, diff);
    }
  }
  if (lane_pairs.empty()) {
    return;
  }
  std::sort(lane_pairs.begin(), lane_pairs.end(),
            [](const std::pair<std::shared_ptr<const LaneInfo>, double>& p1,
               const std::pair<std::shared_ptr<const LaneInfo>, double>& p2) {
              return p1.second < p2.second;
            });

  int count = 0;
  for (const auto& lane_pair : lane_pairs) {
    lanes->push_back(lane_pair.first);
    ++count;
    if (count >= max_num_lane) {
      break;
    }
  }
}

std::shared_ptr<const LaneInfo> PredictionMap::GetMostLikelyCurrentLane(
    const common::PointENU& position, const double radius, const double heading,
    const double angle_diff_threshold) {
  std::vector<std::shared_ptr<const LaneInfo>> candidate_lanes;
  if (HDMapUtil::BaseMap().GetLanesWithHeading(position, radius, heading,
                                               angle_diff_threshold,
                                               &candidate_lanes) != 0) {
    return nullptr;
  }
  double min_angle_diff = 2.0 * M_PI;
  std::shared_ptr<const LaneInfo> curr_lane_ptr = nullptr;
  for (auto candidate_lane : candidate_lanes) {
    if (!candidate_lane->IsOnLane({position.x(), position.y()})) {
      continue;
    }
    double distance = 0.0;
    common::PointENU nearest_point = candidate_lane->GetNearestPoint(
        {position.x(), position.y()}, &distance);
    double nearest_point_heading = PathHeading(candidate_lane, nearest_point);
    double angle_diff =
        std::fabs(common::math::AngleDiff(heading, nearest_point_heading));
    if (angle_diff < min_angle_diff) {
      min_angle_diff = angle_diff;
      curr_lane_ptr = candidate_lane;
    }
  }
  return curr_lane_ptr;
}

bool PredictionMap::IsProjectionApproximateWithinLane(
    const Eigen::Vector2d& ego_position, const std::string& lane_id) {
  auto ptr_lane = LaneById(lane_id);
  const auto& lane_points = ptr_lane->points();
  if (lane_points.size() < 2) {
    return false;
  }

  const auto& start_point = lane_points.front();
  const auto& end_point = lane_points.back();

  auto lane_vec = end_point - start_point;

  auto approx_lane_length = lane_vec.Length();
  if (approx_lane_length < 1.0e-3) {
    return false;
  }

  auto dist_vec =
      common::math::Vec2d(ego_position.x(), ego_position.y()) - start_point;

  auto projection_length = dist_vec.InnerProd(lane_vec) / approx_lane_length;

  if (projection_length < 0.0 || projection_length > approx_lane_length) {
    return false;
  }
  return true;
}

bool PredictionMap::NearJunction(const Eigen::Vector2d& point,
                                 const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const JunctionInfo>> junctions;
  HDMapUtil::BaseMap().GetJunctions(hdmap_point, radius, &junctions);
  return junctions.size() > 0;
}

bool PredictionMap::IsPointInJunction(
    const double x, const double y,
    const std::shared_ptr<const JunctionInfo> junction_info_ptr) {
  if (junction_info_ptr == nullptr) {
    return false;
  }
  const Polygon2d& polygon = junction_info_ptr->polygon();
  return polygon.IsPointIn({x, y});
}

std::vector<std::shared_ptr<const JunctionInfo>> PredictionMap::GetJunctions(
    const Eigen::Vector2d& point, const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const JunctionInfo>> junctions;
  HDMapUtil::BaseMap().GetJunctions(hdmap_point, radius, &junctions);
  return junctions;
}

std::vector<std::shared_ptr<const PNCJunctionInfo>>
PredictionMap::GetPNCJunctions(const Eigen::Vector2d& point,
                               const double radius) {
  common::PointENU hdmap_point;
  hdmap_point.set_x(point.x());
  hdmap_point.set_y(point.y());
  std::vector<std::shared_ptr<const PNCJunctionInfo>> junctions;
  HDMapUtil::BaseMap().GetPNCJunctions(hdmap_point, radius, &junctions);
  return junctions;
}

bool PredictionMap::InJunction(const Eigen::Vector2d& point,
                               const double radius) {
  auto junction_infos = GetJunctions(point, radius);
  if (junction_infos.empty()) {
    return false;
  }
  for (const auto junction_info : junction_infos) {
    if (junction_info == nullptr || !junction_info->junction().has_polygon()) {
      continue;
    }
    std::vector<Vec2d> vertices;
    for (const auto& point : junction_info->junction().polygon().point()) {
      vertices.emplace_back(point.x(), point.y());
    }
    if (vertices.size() < 3) {
      continue;
    }
    Polygon2d junction_polygon{vertices};
    if (junction_polygon.IsPointIn({point.x(), point.y()})) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsLaneInJunction(
    const std::shared_ptr<const LaneInfo> lane_info,
    const std::string& junction_id) {
  if (lane_info == nullptr) {
    return false;
  }

  // first, check whether the lane is virtual
  if (!PredictionMap::IsVirtualLane(lane_info->lane().id().id())) {
    return false;
  }

  // second, use junction from lane
  if (lane_info->lane().has_junction_id() &&
      lane_info->lane().junction_id().id() == junction_id) {
    return true;
  }

  // third, use junction from road
  auto ptr_road_info = HDMapUtil::BaseMap().GetRoadById(lane_info->road_id());
  if (ptr_road_info->has_junction_id() &&
      ptr_road_info->junction_id().id() == junction_id) {
    return true;
  }

  return false;
}

double PredictionMap::PathHeading(std::shared_ptr<const LaneInfo> lane_info,
                                  const common::PointENU& point) {
  double s = 0.0;
  double l = 0.0;
  if (lane_info->GetProjection({point.x(), point.y()}, &s, &l)) {
    return HeadingOnLane(lane_info, s);
  } else {
    return M_PI;
  }
}

bool PredictionMap::SmoothPointFromLane(const std::string& id, const double s,
                                        const double l, Eigen::Vector2d* point,
                                        double* heading) {
  if (point == nullptr || heading == nullptr) {
    return false;
  }
  std::shared_ptr<const LaneInfo> lane = LaneById(id);
  common::PointENU hdmap_point = lane->GetSmoothPoint(s);
  *heading = PathHeading(lane, hdmap_point);
  point->x() = hdmap_point.x() - std::sin(*heading) * l;
  point->y() = hdmap_point.y() + std::cos(*heading) * l;
  return true;
}

void PredictionMap::NearbyLanesByCurrentLanes(
    const Eigen::Vector2d& point, const double heading, const double radius,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes,
    const int max_num_lane,
    std::vector<std::shared_ptr<const LaneInfo>>* nearby_lanes) {
  if (lanes.empty()) {
    std::vector<std::shared_ptr<const LaneInfo>> prev_lanes;
    OnLane(prev_lanes, point, heading, radius, false, max_num_lane,
           FLAGS_max_lane_angle_diff, nearby_lanes);
  } else {
    std::unordered_set<std::string> lane_ids;
    for (auto& lane_ptr : lanes) {
      if (lane_ptr == nullptr) {
        continue;
      }
      for (auto& lane_id : lane_ptr->lane().left_neighbor_forward_lane_id()) {
        const std::string& id = lane_id.id();
        if (lane_ids.find(id) != lane_ids.end()) {
          continue;
        }
        std::shared_ptr<const LaneInfo> nearby_lane = LaneById(id);
        double s = -1.0;
        double l = 0.0;
        GetProjection(point, nearby_lane, &s, &l);
        if (s < 0.0 || s >= nearby_lane->total_length() ||
            std::fabs(l) > radius) {
          continue;
        }
        lane_ids.insert(id);
        nearby_lanes->push_back(nearby_lane);
      }
      for (auto& lane_id : lane_ptr->lane().right_neighbor_forward_lane_id()) {
        const std::string& id = lane_id.id();
        if (lane_ids.find(id) != lane_ids.end()) {
          continue;
        }
        std::shared_ptr<const LaneInfo> nearby_lane = LaneById(id);
        double s = -1.0;
        double l = 0.0;
        GetProjection(point, nearby_lane, &s, &l);
        if (s < 0.0 || s >= nearby_lane->total_length() ||
            std::fabs(l) > radius) {
          continue;
        }
        lane_ids.insert(id);
        nearby_lanes->push_back(nearby_lane);
      }
    }
  }
}

std::shared_ptr<const LaneInfo> PredictionMap::GetLeftNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position, const double threshold) {
  std::vector<std::string> neighbor_ids;
  for (const auto& lane_id :
       ptr_ego_lane->lane().left_neighbor_forward_lane_id()) {
    neighbor_ids.push_back(lane_id.id());
  }

  return GetNeighborLane(ptr_ego_lane, ego_position, neighbor_ids, threshold);
}

std::shared_ptr<const LaneInfo> PredictionMap::GetRightNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position, const double threshold) {
  std::vector<std::string> neighbor_ids;
  for (const auto& lane_id :
       ptr_ego_lane->lane().right_neighbor_forward_lane_id()) {
    neighbor_ids.push_back(lane_id.id());
  }

  return GetNeighborLane(ptr_ego_lane, ego_position, neighbor_ids, threshold);
}

std::shared_ptr<const LaneInfo> PredictionMap::GetNeighborLane(
    const std::shared_ptr<const LaneInfo>& ptr_ego_lane,
    const Eigen::Vector2d& ego_position,
    const std::vector<std::string>& neighbor_lane_ids, const double threshold) {
  double ego_s = 0.0;
  double ego_l = 0.0;
  GetProjection(ego_position, ptr_ego_lane, &ego_s, &ego_l);

  double s_diff_min = std::numeric_limits<double>::max();
  std::shared_ptr<const LaneInfo> ptr_lane_min = nullptr;

  for (auto& lane_id : neighbor_lane_ids) {
    std::shared_ptr<const LaneInfo> ptr_lane = LaneById(lane_id);
    double s = -1.0;
    double l = 0.0;
    GetProjection(ego_position, ptr_lane, &s, &l);

    double s_diff = std::fabs(s - ego_s);
    if (s_diff < s_diff_min) {
      s_diff_min = s_diff;
      ptr_lane_min = ptr_lane;
    }
  }

  if (s_diff_min > threshold) {
    return nullptr;
  }
  return ptr_lane_min;
}

std::vector<std::string> PredictionMap::NearbyLaneIds(
    const Eigen::Vector2d& point, const double radius) {
  std::vector<std::string> lane_ids;
  std::vector<std::shared_ptr<const LaneInfo>> lanes;
  common::PointENU hdmap_point;
  hdmap_point.set_x(point[0]);
  hdmap_point.set_y(point[1]);
  HDMapUtil::BaseMap().GetLanes(hdmap_point, radius, &lanes);
  for (const auto& lane : lanes) {
    lane_ids.push_back(lane->id().id());
  }
  return lane_ids;
}

bool PredictionMap::IsLeftNeighborLane(
    std::shared_ptr<const LaneInfo> target_lane,
    std::shared_ptr<const LaneInfo> curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  for (const auto& left_lane_id :
       curr_lane->lane().left_neighbor_forward_lane_id()) {
    if (target_lane->id().id() == left_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsLeftNeighborLane(
    std::shared_ptr<const LaneInfo> target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsLeftNeighborLane(target_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsRightNeighborLane(
    std::shared_ptr<const LaneInfo> target_lane,
    std::shared_ptr<const LaneInfo> curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  for (auto& right_lane_id :
       curr_lane->lane().right_neighbor_forward_lane_id()) {
    if (target_lane->id().id() == right_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsRightNeighborLane(
    std::shared_ptr<const LaneInfo> target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  for (const auto& lane : lanes) {
    if (IsRightNeighborLane(target_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsSuccessorLane(std::shared_ptr<const LaneInfo> target_lane,
                                    std::shared_ptr<const LaneInfo> curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  for (const auto& successor_lane_id : curr_lane->lane().successor_id()) {
    if (target_lane->id().id() == successor_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsSuccessorLane(
    std::shared_ptr<const LaneInfo> target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsSuccessorLane(target_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsPredecessorLane(
    std::shared_ptr<const LaneInfo> target_lane,
    std::shared_ptr<const LaneInfo> curr_lane) {
  if (curr_lane == nullptr) {
    return true;
  }
  if (target_lane == nullptr) {
    return false;
  }
  for (const auto& predecessor_lane_id : curr_lane->lane().predecessor_id()) {
    if (target_lane->id().id() == predecessor_lane_id.id()) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsPredecessorLane(
    std::shared_ptr<const LaneInfo> target_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsPredecessorLane(target_lane, lane)) {
      return true;
    }
  }
  return false;
}

bool PredictionMap::IsIdenticalLane(std::shared_ptr<const LaneInfo> other_lane,
                                    std::shared_ptr<const LaneInfo> curr_lane) {
  if (curr_lane == nullptr || other_lane == nullptr) {
    return true;
  }
  return other_lane->id().id() == curr_lane->id().id();
}

bool PredictionMap::IsIdenticalLane(
    std::shared_ptr<const LaneInfo> other_lane,
    const std::vector<std::shared_ptr<const LaneInfo>>& lanes) {
  if (lanes.empty()) {
    return true;
  }
  for (auto& lane : lanes) {
    if (IsIdenticalLane(other_lane, lane)) {
      return true;
    }
  }
  return false;
}

int PredictionMap::LaneTurnType(const std::string& lane_id) {
  std::shared_ptr<const LaneInfo> lane = LaneById(lane_id);
  if (lane != nullptr) {
    return static_cast<int>(lane->lane().turn());
  }
  return 1;
}

std::vector<std::shared_ptr<const LaneInfo>> PredictionMap::GetNearbyLanes(
    const common::PointENU& position, const double nearby_radius) {
  CHECK(position.has_x() && position.has_y() && position.has_z());
  CHECK(nearby_radius > 0.0);

  std::vector<std::shared_ptr<const LaneInfo>> nearby_lanes;

  HDMapUtil::BaseMap().GetLanes(position, nearby_radius, &nearby_lanes);
  return nearby_lanes;
}

std::shared_ptr<const LaneInfo> PredictionMap::LaneWithSmallestAverageCurvature(
    const std::vector<std::shared_ptr<const LaneInfo>>& lane_infos) {
  CHECK(!lane_infos.empty());
  size_t sample_size = FLAGS_sample_size_for_average_lane_curvature;
  std::shared_ptr<const hdmap::LaneInfo> selected_lane_info = lane_infos[0];
  if (selected_lane_info == nullptr) {
    AERROR << "Lane Vector first element: selected_lane_info is nullptr.";
    return nullptr;
  }
  double smallest_curvature =
      AverageCurvature(selected_lane_info->id().id(), sample_size);
  for (size_t i = 1; i < lane_infos.size(); ++i) {
    std::shared_ptr<const hdmap::LaneInfo> lane_info = lane_infos[i];
    if (lane_info == nullptr) {
      AWARN << "Lane vector element: one lane_info is nullptr.";
      continue;
    }
    double curvature = AverageCurvature(lane_info->id().id(), sample_size);
    if (curvature < smallest_curvature) {
      smallest_curvature = curvature;
      selected_lane_info = lane_info;
    }
  }
  return selected_lane_info;
}

double PredictionMap::AverageCurvature(const std::string& lane_id,
                                       const size_t sample_size) {
  CHECK_GT(sample_size, 0);
  std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr =
      PredictionMap::LaneById(lane_id);
  if (lane_info_ptr == nullptr) {
    return 0.0;
  }
  double lane_length = lane_info_ptr->total_length();
  double s_gap = lane_length / static_cast<double>(sample_size);
  double curvature_sum = 0.0;
  for (size_t i = 0; i < sample_size; ++i) {
    double s = s_gap * static_cast<double>(i);
    curvature_sum += std::abs(PredictionMap::CurvatureOnLane(lane_id, s));
  }
  return curvature_sum / static_cast<double>(sample_size);
}

}  // namespace prediction
}  // namespace apollo
