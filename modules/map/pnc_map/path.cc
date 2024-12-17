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

#include "modules/map/pnc_map/path.h"

#include <algorithm>
#include <limits>
#include <unordered_map>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace hdmap {

using apollo::common::math::Box2d;
using apollo::common::math::kMathEpsilon;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Sqr;
using apollo::common::math::Vec2d;
using apollo::common::util::DebugStringFormatter;
using std::placeholders::_1;

namespace {

const double kSampleDistance = 0.25;

bool FindLaneSegment(const MapPathPoint& p1, const MapPathPoint& p2,
                     LaneSegment* const lane_segment) {
  for (const auto& wp1 : p1.lane_waypoints()) {
    if (nullptr == wp1.lane) {
      continue;
    }
    for (const auto& wp2 : p2.lane_waypoints()) {
      if (nullptr == wp2.lane) {
        continue;
      }
      if (wp1.lane->id().id() == wp2.lane->id().id() && wp1.s < wp2.s) {
        *lane_segment = LaneSegment(wp1.lane, wp1.s, wp2.s);
        return true;
      }
    }
  }
  return false;
}

}  // namespace

std::string LaneWaypoint::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return absl::StrCat("id = ", lane->id().id(), "  s = ", s);
}

LaneBoundaryType::Type LeftBoundaryType(const LaneWaypoint& waypoint) {
  if (!waypoint.lane) {
    return LaneBoundaryType::UNKNOWN;
  }
  for (const auto& type :
       waypoint.lane->lane().left_boundary().boundary_type()) {
    if (type.s() <= waypoint.s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type RightBoundaryType(const LaneWaypoint& waypoint) {
  if (!waypoint.lane) {
    return LaneBoundaryType::UNKNOWN;
  }
  for (const auto& type :
       waypoint.lane->lane().right_boundary().boundary_type()) {
    if (type.s() <= waypoint.s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneWaypoint LeftNeighborWaypoint(const LaneWaypoint& waypoint) {
  LaneWaypoint neighbor;
  if (!waypoint.lane) {
    return neighbor;
  }
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  auto map_ptr = HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(map_ptr);
  for (const auto& lane_id :
       waypoint.lane->lane().left_neighbor_forward_lane_id()) {
    auto lane = map_ptr->GetLaneById(lane_id);
    if (!lane) {
      return neighbor;
    }
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }

    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;
    } else {
      return LaneWaypoint(lane, s);
    }
  }
  return neighbor;
}

void LaneSegment::Join(std::vector<LaneSegment>* segments) {
  static constexpr double kSegmentDelta = 0.5;
  std::size_t k = 0;
  std::size_t i = 0;
  while (i < segments->size()) {
    std::size_t j = i;
    while (j + 1 < segments->size() &&
           segments->at(i).lane == segments->at(j + 1).lane) {
      ++j;
    }
    auto& segment_k = segments->at(k);
    segment_k.lane = segments->at(i).lane;
    segment_k.start_s = segments->at(i).start_s;
    segment_k.end_s = segments->at(j).end_s;
    if (segment_k.start_s < kSegmentDelta) {
      segment_k.start_s = 0.0;
    }
    if (segment_k.end_s + kSegmentDelta >= segment_k.lane->total_length()) {
      segment_k.end_s = segment_k.lane->total_length();
    }
    i = j + 1;
    ++k;
  }
  segments->resize(k);
  segments->shrink_to_fit();  // release memory
}

LaneWaypoint RightNeighborWaypoint(const LaneWaypoint& waypoint) {
  LaneWaypoint neighbor;
  if (!waypoint.lane) {
    return neighbor;
  }
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  auto map_ptr = HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(map_ptr);
  for (const auto& lane_id :
       waypoint.lane->lane().right_neighbor_forward_lane_id()) {
    auto lane = map_ptr->GetLaneById(lane_id);
    if (!lane) {
      return neighbor;
    }
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }
    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;
    } else {
      return LaneWaypoint(lane, s);
    }
  }
  return neighbor;
}

std::string LaneSegment::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return absl::StrCat("id = ", lane->id().id(), "  start_s = ", start_s,
                      "  end_s = ", end_s);
}

std::vector<MapPathPoint> MapPathPoint::GetPointsFromSegment(
    const LaneSegment& segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

std::vector<MapPathPoint> MapPathPoint::GetPointsFromLane(LaneInfoConstPtr lane,
                                                          const double start_s,
                                                          const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto& segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (start_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

void MapPathPoint::RemoveDuplicates(std::vector<MapPathPoint>* points) {
  static constexpr double kDuplicatedPointsEpsilon = 1e-7;
  static constexpr double limit =
      kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  CHECK_NOTNULL(points);
  int count = 0;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

std::string MapPathPoint::DebugString() const {
  return absl::StrCat(
      "x = ", x_, "  y = ", y_, "  heading = ", heading_,
      "  lwp = "
      "{(",
      absl::StrJoin(lane_waypoints_, "), (", DebugStringFormatter()), ")}");
}

std::string Path::DebugString() const {
  return absl::StrCat(
      "num_points = ", num_points_,
      "  points = "
      "{(",
      absl::StrJoin(path_points_, "), (", DebugStringFormatter()),
      ")}  "
      "numlane_segments_ = ",
      lane_segments_.size(),
      "  lane_segments = "
      "{(",
      absl::StrJoin(lane_segments_, "), (", DebugStringFormatter()), ")}");
}

std::string PathOverlap::DebugString() const {
  return absl::StrCat(object_id, " ", start_s, " ", end_s);
}

Path::Path(const std::vector<MapPathPoint>& path_points)
    : path_points_(path_points) {
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points)
    : path_points_(std::move(path_points)) {
  Init();
}

Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
}

Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments,
           const double max_approximation_error)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

Path::Path(const std::vector<LaneSegment>& segments)
    : lane_segments_(segments) {
  for (const auto& segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2U);
  Init();
}

Path::Path(std::vector<LaneSegment>&& segments)
    : lane_segments_(std::move(segments)) {
  for (const auto& segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2U);
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments,
           const double max_approximation_error)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

void Path::Init() {
  InitPoints();
  InitLaneSegments();
  InitPointIndex();
  InitWidth();
  InitOverlaps();
}

void Path::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());
  CHECK_GE(num_points_, 2);

  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  segments_.clear();
  segments_.reserve(num_points_);
  unit_directions_.clear();
  unit_directions_.reserve(num_points_);
  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
      heading.Normalize();
    } else {
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      heading = path_points_[i + 1] - path_points_[i];
      float heading_length = heading.Length();
      // TODO(All): use heading.length when all adjacent lanes are guarantee to
      // be connected.
      s += heading_length;
      // Normalize "heading".
      if (heading_length > 0.0) {
        heading /= heading_length;
      }
    }
    unit_directions_.push_back(heading);
  }
  length_ = s;
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
  num_segments_ = num_points_ - 1;

  CHECK_EQ(accumulated_s_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(unit_directions_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(segments_.size(), static_cast<size_t>(num_segments_));
}

void Path::InitLaneSegments() {
  if (lane_segments_.empty()) {
    for (int i = 0; i + 1 < num_points_; ++i) {
      LaneSegment lane_segment;
      if (FindLaneSegment(path_points_[i], path_points_[i + 1],
                          &lane_segment)) {
        lane_segments_.push_back(lane_segment);
      }
    }
  }
  LaneSegment::Join(&lane_segments_);
  if (lane_segments_.empty()) {
    return;
  }
  lane_accumulated_s_.resize(lane_segments_.size());
  lane_accumulated_s_[0] = lane_segments_[0].Length();
  for (std::size_t i = 1; i < lane_segments_.size(); ++i) {
    lane_accumulated_s_[i] =
        lane_accumulated_s_[i - 1] + lane_segments_[i].Length();
  }

  lane_segments_to_next_point_.clear();
  lane_segments_to_next_point_.reserve(num_points_);
  for (int i = 0; i + 1 < num_points_; ++i) {
    LaneSegment lane_segment;
    if (FindLaneSegment(path_points_[i], path_points_[i + 1], &lane_segment)) {
      lane_segments_to_next_point_.push_back(lane_segment);
    } else {
      lane_segments_to_next_point_.push_back(LaneSegment());
    }
  }
  CHECK_EQ(lane_segments_to_next_point_.size(),
           static_cast<size_t>(num_segments_));
}

void Path::InitWidth() {
  lane_left_width_.clear();
  lane_left_width_.reserve(num_sample_points_);
  lane_right_width_.clear();
  lane_right_width_.reserve(num_sample_points_);

  road_left_width_.clear();
  road_left_width_.reserve(num_sample_points_);
  road_right_width_.clear();
  road_right_width_.reserve(num_sample_points_);

  double sample_s = 0;
  double segment_end_s = -1.0;
  double segment_start_s = -1.0;
  double waypoint_s = 0.0;
  double left_width = 0.0;
  double right_width = 0.0;
  const LaneWaypoint* cur_waypoint = nullptr;
  bool is_reach_to_end = false;
  int path_point_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    // Find the segment at the position of "sample_s".
    while (segment_end_s < sample_s && !is_reach_to_end) {
      const auto& cur_point = path_points_[path_point_index];
      cur_waypoint = &(cur_point.lane_waypoints()[0]);
      CHECK_NOTNULL(cur_waypoint->lane);
      segment_start_s = accumulated_s_[path_point_index];
      segment_end_s = segment_start_s + segments_[path_point_index].length();
      if (++path_point_index >= num_points_) {
        is_reach_to_end = true;
      }
    }
    // Find the width of the way point at the position of "sample_s".
    waypoint_s = cur_waypoint->s + sample_s - segment_start_s;
    cur_waypoint->lane->GetWidth(waypoint_s, &left_width, &right_width);
    lane_left_width_.push_back(left_width - cur_waypoint->l);
    lane_right_width_.push_back(right_width + cur_waypoint->l);
    cur_waypoint->lane->GetRoadWidth(waypoint_s, &left_width, &right_width);
    road_left_width_.push_back(left_width - cur_waypoint->l);
    road_right_width_.push_back(right_width + cur_waypoint->l);
    sample_s += kSampleDistance;
  }
  // Check the width array size.
  auto num_sample_points = static_cast<size_t>(num_sample_points_);
  CHECK_EQ(lane_left_width_.size(), num_sample_points);
  CHECK_EQ(lane_right_width_.size(), num_sample_points);

  CHECK_EQ(road_left_width_.size(), num_sample_points);
  CHECK_EQ(road_right_width_.size(), num_sample_points);
}

void Path::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);
  double s = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    while (last_index + 1 < num_points_ &&
           accumulated_s_[last_index + 1] <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }
  CHECK_EQ(last_point_index_.size(), static_cast<size_t>(num_sample_points_));
}

void Path::GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                          std::vector<PathOverlap>* const overlaps) const {
  if (overlaps == nullptr) {
    return;
  }
  overlaps->clear();
  std::unordered_map<std::string, std::vector<std::pair<double, double>>>
      overlaps_by_id;
  double s = 0.0;
  for (const auto& lane_segment : lane_segments_) {
    if (lane_segment.lane == nullptr) {
      continue;
    }
    for (const auto& overlap : GetOverlaps_from_lane(*(lane_segment.lane))) {
      const auto& overlap_info =
          overlap->GetObjectOverlapInfo(lane_segment.lane->id());
      if (overlap_info == nullptr) {
        continue;
      }

      const auto& lane_overlap_info = overlap_info->lane_overlap_info();
      if (lane_overlap_info.start_s() <= lane_segment.end_s &&
          lane_overlap_info.end_s() >= lane_segment.start_s) {
        const double ref_s = s - lane_segment.start_s;
        const double adjusted_start_s =
            std::max(lane_overlap_info.start_s(), lane_segment.start_s) + ref_s;
        const double adjusted_end_s =
            std::min(lane_overlap_info.end_s(), lane_segment.end_s) + ref_s;
        for (const auto& object : overlap->overlap().object()) {
          if (object.id().id() != lane_segment.lane->id().id()) {
            overlaps_by_id[object.id().id()].emplace_back(adjusted_start_s,
                                                          adjusted_end_s);
          }
        }
      }
    }
    s += lane_segment.end_s - lane_segment.start_s;
  }
  for (auto& overlaps_one_object : overlaps_by_id) {
    const std::string& object_id = overlaps_one_object.first;
    auto& segments = overlaps_one_object.second;
    std::sort(segments.begin(), segments.end());

    const double kMinOverlapDistanceGap = 1.5;  // in meters.
    for (const auto& segment : segments) {
      if (!overlaps->empty() && overlaps->back().object_id == object_id &&
          segment.first - overlaps->back().end_s <= kMinOverlapDistanceGap) {
        overlaps->back().end_s =
            std::max(overlaps->back().end_s, segment.second);
      } else {
        overlaps->emplace_back(object_id, segment.first, segment.second);
      }
    }
  }
  std::sort(overlaps->begin(), overlaps->end(),
            [](const PathOverlap& overlap1, const PathOverlap& overlap2) {
              return overlap1.start_s < overlap2.start_s;
            });
}

const PathOverlap* Path::NextLaneOverlap(double s) const {
  auto next = std::upper_bound(
      lane_overlaps_.begin(), lane_overlaps_.end(), s,
      [](double s, const PathOverlap& o) { return s < o.start_s; });
  if (next == lane_overlaps_.end()) {
    return nullptr;
  } else {
    return &(*next);
  }
}

void Path::InitOverlaps() {
  GetAllOverlaps(std::bind(&LaneInfo::cross_lanes, _1), &lane_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::signals, _1), &signal_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::yield_signs, _1), &yield_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::stop_signs, _1), &stop_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::crosswalks, _1), &crosswalk_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::junctions, _1), &junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::pnc_junctions, _1),
                 &pnc_junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::clear_areas, _1), &clear_area_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::speed_bumps, _1), &speed_bump_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::parking_spaces, _1),
                 &parking_space_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::areas, _1), &area_overlaps_);
}

MapPathPoint Path::GetSmoothPoint(const InterpolatedIndex& index) const {
  CHECK_GE(index.id, 0);
  CHECK_LT(index.id, num_points_);

  const MapPathPoint& ref_point = path_points_[index.id];
  if (std::abs(index.offset) > kMathEpsilon) {
    const Vec2d delta = unit_directions_[index.id] * index.offset;
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()},
                       ref_point.heading());
    if (index.id < num_segments_ && !ref_point.lane_waypoints().empty()) {
      const LaneSegment& lane_segment = lane_segments_to_next_point_[index.id];
      auto ref_lane_waypoint = ref_point.lane_waypoints()[0];
      if (lane_segment.lane != nullptr) {
        for (const auto& lane_waypoint : ref_point.lane_waypoints()) {
          if (lane_waypoint.lane->id().id() == lane_segment.lane->id().id()) {
            ref_lane_waypoint = lane_waypoint;
            break;
          }
        }
        point.add_lane_waypoint(
            LaneWaypoint(lane_segment.lane, lane_segment.start_s + index.offset,
                         ref_lane_waypoint.l));
      }
    }
    if (point.lane_waypoints().empty() && !ref_point.lane_waypoints().empty()) {
      point.add_lane_waypoint(ref_point.lane_waypoints()[0]);
    }
    return point;
  } else {
    return ref_point;
  }
}

MapPathPoint Path::GetSmoothPoint(double s) const {
  return GetSmoothPoint(GetIndexFromS(s));
}

double Path::GetSFromIndex(const InterpolatedIndex& index) const {
  if (index.id < 0) {
    return 0.0;
  }
  if (index.id >= num_points_) {
    return length_;
  }
  return accumulated_s_[index.id] + index.offset;
}

InterpolatedIndex Path::GetIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(num_points_, 0);
  if (s >= length_) {
    return {num_points_ - 1, 0.0};
  }
  const int sample_id = static_cast<int>(s / kSampleDistance);
  if (sample_id >= num_sample_points_) {
    return {num_points_ - 1, 0.0};
  }
  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
  while (low + 1 < high) {
    const int mid = (low + high) >> 1;
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return {low, s - accumulated_s_[low]};
}

InterpolatedIndex Path::GetLaneIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(lane_segments_.size(), 0U);
  if (s >= length_) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }
  auto iter = std::lower_bound(lane_accumulated_s_.begin(),
                               lane_accumulated_s_.end(), s);
  if (iter == lane_accumulated_s_.end()) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }
  int index =
      static_cast<int>(std::distance(lane_accumulated_s_.begin(), iter));
  if (index == 0) {
    return {index, s};
  } else {
    return {index, s - lane_accumulated_s_[index - 1]};
  }
}

std::vector<hdmap::LaneSegment> Path::GetLaneSegments(
    const double start_s, const double end_s) const {
  std::vector<hdmap::LaneSegment> lanes;
  if (start_s + kMathEpsilon > end_s) {
    return lanes;
  }
  auto start_index = GetLaneIndexFromS(start_s);
  if (start_index.offset + kMathEpsilon >=
      lane_segments_[start_index.id].Length()) {
    start_index.id += 1;
    start_index.offset = 0;
  }
  const int num_lanes = static_cast<int>(lane_segments_.size());
  if (start_index.id >= num_lanes) {
    return lanes;
  }
  lanes.emplace_back(lane_segments_[start_index.id].lane, start_index.offset,
                     lane_segments_[start_index.id].Length());
  auto end_index = GetLaneIndexFromS(end_s);
  for (int i = start_index.id; i < end_index.id && i < num_lanes; ++i) {
    lanes.emplace_back(lane_segments_[i]);
  }
  if (end_index.offset >= kMathEpsilon) {
    lanes.emplace_back(lane_segments_[end_index.id].lane, 0, end_index.offset);
  }
  return lanes;
}

bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral) const {
  double distance = 0.0;
  return GetNearestPoint(point, accumulate_s, lateral, &distance);
}

bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral, double* min_distance) const {
  if (!GetProjection(point, accumulate_s, lateral, min_distance)) {
    return false;
  }
  if (*accumulate_s < 0.0) {
    *accumulate_s = 0.0;
    *min_distance = point.DistanceTo(path_points_[0]);
  } else if (*accumulate_s > length_) {
    *accumulate_s = length_;
    *min_distance = point.DistanceTo(path_points_.back());
  }
  return true;
}

bool Path::GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                         double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool Path::GetProjection(const double heading, const common::math::Vec2d& point,
                         double* accumulate_s, double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, heading, accumulate_s, lateral, &distance);
}

bool Path::GetProjectionWithWarmStartS(const common::math::Vec2d& point,
                                       double* accumulate_s,
                                       double* lateral) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr) {
    return false;
  }
  if (*accumulate_s < 0.0) {
    *accumulate_s = 0.0;
  } else if (*accumulate_s > length()) {
    *accumulate_s = length();
  }
  CHECK_GE(num_points_, 2);
  double warm_start_s = *accumulate_s;
  // Find the segment at the position of "accumulate_s".
  int left_index = 0;
  int right_index = num_segments_;
  int mid_index = 0;
  // Find the segment with projection of the given point on it.
  while (right_index > left_index + 1) {
    FindIndex(left_index, right_index, warm_start_s, &mid_index);
    const auto& segment = segments_[mid_index];
    const auto& start_point = segment.start();
    double delta_x = point.x() - start_point.x();
    double delta_y = point.y() - start_point.y();
    const auto& unit_direction = segment.unit_direction();
    double proj = delta_x * unit_direction.x() + delta_y * unit_direction.y();
    *accumulate_s = accumulated_s_[mid_index] + proj;
    *lateral = unit_direction.x() * delta_y - unit_direction.y() * delta_x;
    if (proj > 0.0) {
      if (proj < segment.length()) {
        return true;
      }
      if (mid_index == right_index) {
        *accumulate_s = accumulated_s_[mid_index];
        return true;
      }
      left_index = mid_index + 1;
    } else {
      if (mid_index == left_index) {
        *accumulate_s = accumulated_s_[mid_index];
        return true;
      }
      if (std::abs(proj) < segments_[mid_index - 1].length()) {
        return true;
      }
      right_index = mid_index - 1;
    }
    warm_start_s = segment.length() + proj;
  }
  return true;
}

bool Path::GetProjectionWithHueristicParams(const Vec2d& point,
                                            const double hueristic_start_s,
                                            const double hueristic_end_s,
                                            double* accumulate_s,
                                            double* lateral,
                                            double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();

  int start_interpolation_index = GetIndexFromS(hueristic_start_s).id;
  int end_interpolation_index = static_cast<int>(
      std::fmin(num_segments_, GetIndexFromS(hueristic_end_s).id + 1));
  int min_index = start_interpolation_index;
  for (int i = start_interpolation_index; i <= end_interpolation_index; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool Path::GetProjection(const Vec2d& point, double* accumulate_s,
                         double* lateral, double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments_; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool Path::GetProjection(const Vec2d& point, const double heading,
                         double* accumulate_s, double* lateral,
                         double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments_; ++i) {
    if (abs(common::math::AngleDiff(segments_[i].heading(), heading)) >= M_PI_2)
      continue;
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool Path::GetHeadingAlongPath(const Vec2d& point, double* heading) const {
  if (heading == nullptr) {
    return false;
  }
  double s = 0;
  double l = 0;
  if (GetProjection(point, &s, &l)) {
    *heading = GetSmoothPoint(s).heading();
    return true;
  }
  return false;
}

double Path::GetLaneLeftWidth(const double s) const {
  return GetSample(lane_left_width_, s);
}

double Path::GetLaneRightWidth(const double s) const {
  return GetSample(lane_right_width_, s);
}

bool Path::GetLaneWidth(const double s, double* lane_left_width,
                        double* lane_right_width) const {
  CHECK_NOTNULL(lane_left_width);
  CHECK_NOTNULL(lane_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }
  *lane_left_width = GetSample(lane_left_width_, s);
  *lane_right_width = GetSample(lane_right_width_, s);
  return true;
}

double Path::GetRoadLeftWidth(const double s) const {
  return GetSample(road_left_width_, s);
}

double Path::GetRoadRightWidth(const double s) const {
  return GetSample(road_right_width_, s);
}

bool Path::GetRoadWidth(const double s, double* road_left_width,
                        double* road_right_width) const {
  CHECK_NOTNULL(road_left_width);
  CHECK_NOTNULL(road_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }

  *road_left_width = GetSample(road_left_width_, s);
  *road_right_width = GetSample(road_right_width_, s);
  return true;
}

double Path::GetSample(const std::vector<double>& samples,
                       const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return samples[0];
  }
  const int idx = static_cast<int>(s / kSampleDistance);
  if (idx >= num_sample_points_ - 1) {
    return samples.back();
  }
  const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
  return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
}

bool Path::IsOnPath(const Vec2d& point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!GetLaneWidth(accumulate_s, &lane_left_width, &lane_right_width)) {
    return false;
  }
  if (lateral < lane_left_width && lateral > -lane_right_width) {
    return true;
  }
  return false;
}

bool Path::OverlapWith(const common::math::Box2d& box, double width) const {
  if (use_path_approximation_) {
    return approximation_.OverlapWith(*this, box, width);
  }
  const Vec2d center = box.center();
  const double radius_sqr = Sqr(box.diagonal() / 2.0 + width) + kMathEpsilon;
  for (const auto& segment : segments_) {
    if (segment.DistanceSquareTo(center) > radius_sqr) {
      continue;
    }
    if (box.DistanceTo(segment) <= width + kMathEpsilon) {
      return true;
    }
  }
  return false;
}

double PathApproximation::compute_max_error(const Path& path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return 0.0;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  double max_distance_sqr = 0.0;
  for (int i = s + 1; i < t; ++i) {
    max_distance_sqr =
        std::max(max_distance_sqr, segment.DistanceSquareTo(points[i]));
  }
  return sqrt(max_distance_sqr);
}

bool PathApproximation::is_within_max_error(const Path& path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return true;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  for (int i = s + 1; i < t; ++i) {
    if (segment.DistanceSquareTo(points[i]) > max_sqr_error_) {
      return false;
    }
  }
  return true;
}

void PathApproximation::Init(const Path& path) {
  InitDilute(path);
  InitProjections(path);
}

void PathApproximation::InitDilute(const Path& path) {
  const int num_original_points = path.num_points();
  original_ids_.clear();
  int last_idx = 0;
  while (last_idx < num_original_points - 1) {
    original_ids_.push_back(last_idx);
    int next_idx = last_idx + 1;
    int delta = 2;
    for (; last_idx + delta < num_original_points; delta *= 2) {
      if (!is_within_max_error(path, last_idx, last_idx + delta)) {
        break;
      }
      next_idx = last_idx + delta;
    }
    for (; delta > 0; delta /= 2) {
      if (next_idx + delta < num_original_points &&
          is_within_max_error(path, last_idx, next_idx + delta)) {
        next_idx += delta;
      }
    }
    last_idx = next_idx;
  }
  original_ids_.push_back(last_idx);
  num_points_ = static_cast<int>(original_ids_.size());
  if (num_points_ == 0) {
    return;
  }

  segments_.clear();
  segments_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    segments_.emplace_back(path.path_points()[original_ids_[i]],
                           path.path_points()[original_ids_[i + 1]]);
  }
  max_error_per_segment_.clear();
  max_error_per_segment_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    max_error_per_segment_.push_back(
        compute_max_error(path, original_ids_[i], original_ids_[i + 1]));
  }
}

void PathApproximation::InitProjections(const Path& path) {
  if (num_points_ == 0) {
    return;
  }
  projections_.clear();
  projections_.reserve(segments_.size() + 1);
  double s = 0.0;
  projections_.push_back(0);
  for (const auto& segment : segments_) {
    s += segment.length();
    projections_.push_back(s);
  }
  const auto& original_points = path.path_points();
  const int num_original_points = static_cast<int>(original_points.size());
  original_projections_.clear();
  original_projections_.reserve(num_original_points);
  for (size_t i = 0; i < projections_.size(); ++i) {
    original_projections_.push_back(projections_[i]);
    if (i + 1 < projections_.size()) {
      const auto& segment = segments_[i];
      for (int idx = original_ids_[i] + 1; idx < original_ids_[i + 1]; ++idx) {
        const double proj = segment.ProjectOntoUnit(original_points[idx]);
        original_projections_.push_back(
            projections_[i] + std::max(0.0, std::min(proj, segment.length())));
      }
    }
  }

  // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
  max_original_projections_to_left_.resize(num_original_points);
  double last_projection = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_original_points; ++i) {
    last_projection = std::max(last_projection, original_projections_[i]);
    max_original_projections_to_left_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(max_original_projections_to_left_[i],
             max_original_projections_to_left_[i + 1] + kMathEpsilon);
  }

  // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
  min_original_projections_to_right_.resize(original_projections_.size());
  last_projection = std::numeric_limits<double>::infinity();
  for (int i = num_original_points - 1; i >= 0; --i) {
    last_projection = std::min(last_projection, original_projections_[i]);
    min_original_projections_to_right_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(min_original_projections_to_right_[i],
             min_original_projections_to_right_[i + 1] + kMathEpsilon);
  }

  // Sample max_p_to_left by sample_distance.
  max_projection_ = projections_.back();
  num_projection_samples_ =
      static_cast<int>(max_projection_ / kSampleDistance) + 1;
  sampled_max_original_projections_to_left_.clear();
  sampled_max_original_projections_to_left_.reserve(num_projection_samples_);
  double proj = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_projection_samples_; ++i) {
    while (last_index + 1 < num_original_points &&
           max_original_projections_to_left_[last_index + 1] < proj) {
      ++last_index;
    }
    sampled_max_original_projections_to_left_.push_back(last_index);
    proj += kSampleDistance;
  }
  CHECK_EQ(sampled_max_original_projections_to_left_.size(),
           static_cast<size_t>(num_projection_samples_));
}

bool PathApproximation::GetProjection(const Path& path,
                                      const common::math::Vec2d& point,
                                      double* accumulate_s, double* lateral,
                                      double* min_distance) const {
  if (num_points_ == 0) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  double min_distance_sqr = std::numeric_limits<double>::infinity();
  int estimate_nearest_segment_idx = -1;
  std::vector<double> distance_sqr_to_segments;
  distance_sqr_to_segments.reserve(segments_.size());
  for (size_t i = 0; i < segments_.size(); ++i) {
    const double distance_sqr = segments_[i].DistanceSquareTo(point);
    distance_sqr_to_segments.push_back(distance_sqr);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      estimate_nearest_segment_idx = static_cast<int>(i);
    }
  }
  if (estimate_nearest_segment_idx < 0) {
    return false;
  }
  const auto& original_segments = path.segments();
  const int num_original_segments = static_cast<int>(original_segments.size());
  const auto& original_accumulated_s = path.accumulated_s();
  double min_distance_sqr_with_error =
      Sqr(sqrt(min_distance_sqr) +
          max_error_per_segment_[estimate_nearest_segment_idx] + max_error_);
  *min_distance = std::numeric_limits<double>::infinity();
  int nearest_segment_idx = -1;
  for (size_t i = 0; i < segments_.size(); ++i) {
    if (distance_sqr_to_segments[i] >= min_distance_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(point);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(point));
      if (prod_sqr >= min_distance_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(min_distance_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    bool min_distance_updated = false;
    bool is_within_end_point = false;
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      const double x0 = point.x() - original_segment.start().x();
      const double y0 = point.y() - original_segment.start().y();
      const double ux = original_segment.unit_direction().x();
      const double uy = original_segment.unit_direction().y();
      double proj = x0 * ux + y0 * uy;
      double distance = 0.0;
      if (proj < 0.0) {
        if (is_within_end_point) {
          continue;
        }
        is_within_end_point = true;
        distance = hypot(x0, y0);
      } else if (proj <= original_segment.length()) {
        is_within_end_point = true;
        distance = std::abs(x0 * uy - y0 * ux);
      } else {
        is_within_end_point = false;
        if (idx != last_segment_idx) {
          continue;
        }
        distance = original_segment.end().DistanceTo(point);
      }
      if (distance < *min_distance) {
        min_distance_updated = true;
        *min_distance = distance;
        nearest_segment_idx = idx;
      }
    }
    if (min_distance_updated) {
      min_distance_sqr_with_error = Sqr(*min_distance + max_error_);
    }
  }
  if (nearest_segment_idx >= 0) {
    const auto& segment = original_segments[nearest_segment_idx];
    double proj = segment.ProjectOntoUnit(point);
    const double prod = segment.ProductOntoUnit(point);
    if (nearest_segment_idx > 0) {
      proj = std::max(0.0, proj);
    }
    if (nearest_segment_idx + 1 < num_original_segments) {
      proj = std::min(segment.length(), proj);
    }
    *accumulate_s = original_accumulated_s[nearest_segment_idx] + proj;
    if ((nearest_segment_idx == 0 && proj < 0.0) ||
        (nearest_segment_idx + 1 == num_original_segments &&
         proj > segment.length())) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0 ? (*min_distance) : -(*min_distance));
    }
    return true;
  }
  return false;
}

bool PathApproximation::OverlapWith(const Path& path, const Box2d& box,
                                    double width) const {
  if (num_points_ == 0) {
    return false;
  }
  const Vec2d center = box.center();
  const double radius = box.diagonal() / 2.0 + width;
  const double radius_sqr = Sqr(radius);
  const auto& original_segments = path.segments();
  for (size_t i = 0; i < segments_.size(); ++i) {
    const LineSegment2d& segment = segments_[i];
    const double max_error = max_error_per_segment_[i];
    const double radius_sqr_with_error = Sqr(radius + max_error);
    if (segment.DistanceSquareTo(center) > radius_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(center);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(center));
      if (prod_sqr >= radius_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(radius_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      if (original_segment.DistanceSquareTo(center) > radius_sqr) {
        continue;
      }
      if (box.DistanceTo(original_segment) <= width) {
        return true;
      }
    }
  }
  return false;
}

void Path::FindIndex(int left_index, int right_index, double target_s,
                     int* mid_index) const {
  // Find the segment index with binary search
  while (right_index > left_index + 1) {
    *mid_index = ((left_index + right_index) >> 1);
    if (accumulated_s_[*mid_index] < target_s) {
      left_index = *mid_index;
      continue;
    }
    if (accumulated_s_[*mid_index - 1] > target_s) {
      right_index = *mid_index - 1;
      continue;
    }
    return;
  }
}

}  // namespace hdmap
}  // namespace apollo
