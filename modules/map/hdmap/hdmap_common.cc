/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#include "modules/map/hdmap/hdmap_common.h"

#include <algorithm>
#include <iostream>
#include <limits>

#include "glog/logging.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/map/hdmap/hdmap_util.h"

namespace {

// Minimum error in lane segmentation.
const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;

void remove_duplicates(std::vector<apollo::common::math::Vec2d> *points) {
  CHECK_NOTNULL(points);

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    }
  }
  points->resize(count);
}

void points_from_curve(const apollo::hdmap::Curve &input_curve,
                       std::vector<apollo::common::math::Vec2d> *points) {
  CHECK_NOTNULL(points)->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    } else {
      LOG(FATAL) << "Can not handle curve type.";
    }
  }
  remove_duplicates(points);
}

apollo::common::math::Polygon2d convert_to_polygon2d(
    const apollo::hdmap::Polygon &polygon) {
  std::vector<apollo::common::math::Vec2d> points;
  points.reserve(polygon.point_size());
  for (const auto &point : polygon.point()) {
    points.emplace_back(point.x(), point.y());
  }
  remove_duplicates(&points);
  while (points.size() >= 2 &&
         points[0].DistanceTo(points.back()) <=
             apollo::common::math::kMathEpsilon) {
    points.pop_back();
  }
  return apollo::common::math::Polygon2d(points);
}

void segments_from_curve(
    const apollo::hdmap::Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments) {
  std::vector<apollo::common::math::Vec2d> points;
  points_from_curve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}

apollo::common::PointENU point_from_vec2d(
    const apollo::common::math::Vec2d &point) {
  apollo::common::PointENU pt;
  pt.set_x(point.x());
  pt.set_y(point.y());
  return pt;
}
apollo::common::math::Vec2d vec2d_from_point(
    const apollo::common::PointENU &point) {
  return apollo::common::math::Vec2d(point.x(), point.y());
  ;
}

}  // namespace

namespace apollo {
namespace hdmap {

LaneInfo::LaneInfo(const apollo::hdmap::Lane &lane) : _lane(lane) { init(); }

void LaneInfo::init() {
  points_from_curve(_lane.central_curve(), &_points);
  CHECK_GE(_points.size(), 2);
  _segments.clear();
  _accumulated_s.clear();
  _unit_directions.clear();
  _headings.clear();

  double s = 0;
  for (size_t i = 0; i + 1 < _points.size(); ++i) {
    _segments.emplace_back(_points[i], _points[i + 1]);
    _accumulated_s.push_back(s);
    _unit_directions.push_back(_segments.back().unit_direction());
    s += _segments.back().length();
  }

  _accumulated_s.push_back(s);
  _total_length = s;
  CHECK(!_unit_directions.empty());
  _unit_directions.push_back(_unit_directions.back());
  for (const auto &direction : _unit_directions) {
    _headings.push_back(direction.Angle());
  }
  for (const auto &overlap_id : _lane.overlap_id()) {
    _overlap_ids.emplace_back(overlap_id.id());
  }
  CHECK(!_segments.empty());

  _sampled_left_width.clear();
  _sampled_right_width.clear();
  for (const auto &sample : _lane.left_sample()) {
    _sampled_left_width.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : _lane.right_sample()) {
    _sampled_right_width.emplace_back(sample.s(), sample.width());
  }

  const double kMinHalfWidth = 1.05;
  if (_lane.has_type()) {
    if (_lane.type() == Lane::CITY_DRIVING) {
      for (const auto &p : _sampled_left_width) {
        if (p.second < kMinHalfWidth) {
          AERROR
              << "lane[id = " << _lane.id().DebugString()
              << "]. _sampled_left_width[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << kMinHalfWidth << "].";
        }
      }
      for (const auto &p : _sampled_right_width) {
        if (p.second < kMinHalfWidth) {
          AERROR
              << "lane[id = " << _lane.id().DebugString()
              << "]. _sampled_right_width[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << kMinHalfWidth << "].";
        }
      }
    } else if (_lane.type() == Lane::NONE) {
      AERROR << "_lane[id = " << _lane.id().DebugString() << " type is NONE.";
    }
  } else {
    AERROR << "_lane[id = " << _lane.id().DebugString() << "has NO type.";
  }

  create_kdtree();
}

void LaneInfo::get_width(const double s, double *left_width,
                         double *right_width) const {
  if (left_width != nullptr) {
    *left_width = get_width_from_sample(_sampled_left_width, s);
  }
  if (right_width != nullptr) {
    *right_width = get_width_from_sample(_sampled_right_width, s);
  }
}

double LaneInfo::heading(const double s) const {
  CHECK_GE(s, _accumulated_s.front()) << "s should be >= "
                                      << _accumulated_s.front();
  CHECK_LE(s, _accumulated_s.back()) << "s should be <= "
                                     << _accumulated_s.back();
  auto iter = std::lower_bound(_accumulated_s.begin(), _accumulated_s.end(), s);
  int index = std::distance(_accumulated_s.begin(), iter);
  if (index == 0 || *iter - s <= common::math::kMathEpsilon) {
    return _headings[index];
  } else {
    return ::apollo::common::math::slerp(
        _headings[index - 1], _accumulated_s[index - 1], _headings[index],
        _accumulated_s[index], s);
    // return _headings[index - 1];
  }
}

double LaneInfo::get_width(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  get_width(s, &left_width, &right_width);
  return left_width + right_width;
}

double LaneInfo::get_effective_width(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  get_width(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}

double LaneInfo::get_width_from_sample(
    const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}

bool LaneInfo::is_on_lane(const apollo::common::math::Vec2d &point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!get_projection(point, &accumulate_s, &lateral)) {
    return false;
  }

  if (accumulate_s > total_length() || accumulate_s < 0.0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  get_width(accumulate_s, &left_width, &right_width);
  if (lateral < left_width && lateral > -right_width) {
    return true;
  }
  return false;
}

bool LaneInfo::is_on_lane(const apollo::common::math::Box2d &box) const {
  std::vector<apollo::common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    if (!is_on_lane(corner)) {
      return false;
    }
  }
  return true;
}

apollo::common::PointENU LaneInfo::get_smooth_point(double s) const {
  CHECK_GE(_points.size(), 2);
  if (s <= 0.0) {
    return point_from_vec2d(_points[0]);
  }

  if (s >= total_length()) {
    return point_from_vec2d(_points.back());
  }

  const auto low_itr =
      std::lower_bound(_accumulated_s.begin(), _accumulated_s.end(), s);
  CHECK(low_itr != _accumulated_s.end());
  size_t index = low_itr - _accumulated_s.begin();
  double delta_s = *low_itr - s;
  if (delta_s < apollo::common::math::kMathEpsilon) {
    return point_from_vec2d(_points[index]);
  }

  auto smooth_point = _points[index] - _unit_directions[index - 1] * delta_s;

  return point_from_vec2d(smooth_point);
}

double LaneInfo::distance_to(const apollo::common::math::Vec2d &point) const {
  const auto segment_box = _lane_segment_kdtree->GetNearestObject(point);
  return segment_box->DistanceTo(point);
}

double LaneInfo::distance_to(const apollo::common::math::Vec2d &point,
                             apollo::common::math::Vec2d *map_point,
                             double *s_offset, int *s_offset_index) const {
  const auto segment_box = _lane_segment_kdtree->GetNearestObject(point);
  int index = segment_box->id();
  double distance = _segments[index].DistanceTo(point, map_point);
  *s_offset_index = index;
  *s_offset =
      _accumulated_s[index] + _segments[index].start().DistanceTo(*map_point);
  return distance;
}

apollo::common::PointENU LaneInfo::get_nearest_point(
    const apollo::common::math::Vec2d &point, double *distance) const {
  const auto segment_box = _lane_segment_kdtree->GetNearestObject(point);
  int index = segment_box->id();
  apollo::common::math::Vec2d nearest_point;
  *distance = _segments[index].DistanceTo(point, &nearest_point);

  return point_from_vec2d(nearest_point);
}

bool LaneInfo::get_projection(const apollo::common::math::Vec2d &point,
                              double *accumulate_s, double *lateral) const {
  CHECK_NOTNULL(accumulate_s);
  CHECK_NOTNULL(lateral);

  if (_segments.empty()) {
    return false;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  std::size_t min_index = 0;
  double min_proj = 0.0;
  std::size_t num_segments = _segments.size();
  for (std::size_t i = 0; i < num_segments; ++i) {
    const auto &segment = _segments[i];
    const double distance = segment.DistanceTo(point);
    if (distance < min_distance) {
      const double proj = segment.ProjectOntoUnit(point);
      if (proj < 0.0 && i > 0) {
        continue;
      }
      if (proj > segment.length() && i + 1 < num_segments) {
        const auto &next_segment = _segments[i + 1];
        if ((point - next_segment.start())
                .InnerProd(next_segment.unit_direction()) >= 0.0) {
          continue;
        }
      }
      min_distance = distance;
      min_index = i;
      min_proj = proj;
    }
  }

  const auto &segment = _segments[min_index];
  if (min_index + 1 >= num_segments) {
    *accumulate_s = _accumulated_s[min_index] + min_proj;
  } else {
    *accumulate_s =
        _accumulated_s[min_index] + std::min(min_proj, segment.length());
  }
  const double prod = segment.ProductOntoUnit(point);
  if ((min_index == 0 && min_proj < 0.0) ||
      (min_index + 1 == num_segments && min_proj > segment.length())) {
    *lateral = prod;
  } else {
    *lateral = (prod > 0.0 ? min_distance : -min_distance);
  }

  return true;
}

void LaneInfo::post_process(const HDMapImpl &map_instance) {
  update_overlaps(map_instance);
}

void LaneInfo::update_overlaps(const HDMapImpl &map_instance) {
  for (const auto &overlap_id : _overlap_ids) {
    const auto &overlap_ptr =
        map_instance.GetOverlapById(apollo::hdmap::MakeMapId(overlap_id));
    if (overlap_ptr == nullptr) {
      continue;
    }
    _overlaps.emplace_back(overlap_ptr);
    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == _lane.id().id()) {
        continue;
      }
      const auto &object_map_id = apollo::hdmap::MakeMapId(object_id);
      if (map_instance.GetLaneById(object_map_id) != nullptr) {
        _cross_lanes.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSignalById(object_map_id) != nullptr) {
        _signals.emplace_back(overlap_ptr);
      }
      if (map_instance.GetYieldSignById(object_map_id) != nullptr) {
        _yield_signs.emplace_back(overlap_ptr);
      }
      if (map_instance.GetStopSignById(object_map_id) != nullptr) {
        _stop_signs.emplace_back(overlap_ptr);
      }
      if (map_instance.GetCrosswalkById(object_map_id) != nullptr) {
        _crosswalks.emplace_back(overlap_ptr);
      }
      if (map_instance.GetJunctionById(object_map_id) != nullptr) {
        _junctions.emplace_back(overlap_ptr);
      }

      // TODO: support parking and speed bump
      /*
      if (map_instance.get_parking_space_by_id(object_map_id) != nullptr) {
        _parking_spaces.emplace_back(overlap_ptr);
      }
      if (map_instance.get_speed_bump_by_id(object_map_id) != nullptr) {
        _speed_bumps.emplace_back(overlap_ptr);
      }
      */
    }
  }
}

void LaneInfo::create_kdtree() {
  apollo::common::math::AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;

  _segment_box_list.clear();
  for (size_t id = 0; id < _segments.size(); ++id) {
    const auto &segment = _segments[id];
    _segment_box_list.emplace_back(
        apollo::common::math::AABox2d(segment.start(), segment.end()), this,
        &segment, id);
  }
  _lane_segment_kdtree.reset(new LaneSegmentKDTree(_segment_box_list, params));
}

JunctionInfo::JunctionInfo(const apollo::hdmap::Junction &junction)
    : _junction(junction) {
  init();
}

void JunctionInfo::init() {
  _polygon = convert_to_polygon2d(_junction.polygon());
  CHECK_GT(_polygon.num_points(), 2);
}

SignalInfo::SignalInfo(const apollo::hdmap::Signal &signal) : _signal(signal) {
  init();
}

void SignalInfo::init() {
  for (const auto &stop_line : _signal.stop_line()) {
    segments_from_curve(stop_line, &_segments);
  }
  CHECK(!_segments.empty());
  std::vector<apollo::common::math::Vec2d> points;
  for (const auto &segment : _segments) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  CHECK_GT(points.size(), 0);
}

CrosswalkInfo::CrosswalkInfo(const apollo::hdmap::Crosswalk &crosswalk)
    : _crosswalk(crosswalk) {
  init();
}

void CrosswalkInfo::init() {
  _polygon = convert_to_polygon2d(_crosswalk.polygon());
  CHECK_GT(_polygon.num_points(), 2);
}

StopSignInfo::StopSignInfo(const apollo::hdmap::StopSign &stop_sign)
    : _stop_sign(stop_sign) {
  init();
}

void StopSignInfo::init() {
  for (const auto &stop_line : _stop_sign.stop_line()) {
    segments_from_curve(stop_line, &_segments);
  }
  CHECK(!_segments.empty());
}

YieldSignInfo::YieldSignInfo(const apollo::hdmap::YieldSign &yield_sign)
    : _yield_sign(yield_sign) {
  init();
}

void YieldSignInfo::init() {
  for (const auto &stop_line : _yield_sign.stop_line()) {
    segments_from_curve(stop_line, &_segments);
  }
  // segments_from_curve(_yield_sign.stop_line(), &_segments);
  CHECK(!_segments.empty());
}

OverlapInfo::OverlapInfo(const apollo::hdmap::Overlap &overlap)
    : _overlap(overlap) {}
const ObjectOverlapInfo *OverlapInfo::get_object_overlap_info(
    const apollo::hdmap::Id &id) const {
  for (const auto &object : _overlap.object()) {
    if (object.id().id() == id.id()) {
      return &object;
    }
  }
  return nullptr;
}

RoadInfo::RoadInfo(const apollo::hdmap::Road &road) : _road(road) {
  for (int i = 0; i < _road.section_size(); ++i) {
    _sections.push_back(_road.section(i));
    _road_boundaries.push_back(_road.section(i).boundary());
  }
}

const std::vector<apollo::hdmap::RoadBoundary> &RoadInfo::get_boundaries()
    const {
  return _road_boundaries;
}

}  // namespace hdmap
}  // namespace apollo
