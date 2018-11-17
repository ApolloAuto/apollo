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

#define private public

#include <vector>
#include "modules/map/hdmap/hdmap_impl.h"
#include "modules/tools/fuzz/map/proto/hdmap_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

namespace apollo {
namespace hdmap {

using apollo::hdmap::Curve;
using apollo::common::math::Vec2d;
using apollo::common::math::LineSegment2d;

const double kDuplicatedPointsEpsilon = 1e-7;

// class HDmapFuzz {
//   public:
//     void Init();
//     void Fuzz(HDmapFuzzMessage hdmap_fuzz_message);
//   private:
//     HDMapImpl hdmap_impl_;
// } hdmap_fuzzer;

void RemoveDuplicates(std::vector<Vec2d> *points) {
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

void PointsFromCurve(const Curve &input_curve, std::vector<Vec2d> *points) {
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    }
  }
  RemoveDuplicates(points);
}

void SegmentsFromCurve(
    const Curve &curve,
    std::vector<apollo::common::math::LineSegment2d> *segments) {
  std::vector<Vec2d> points;
  PointsFromCurve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}

bool check_lane(apollo::hdmap::Lane lane) {
  std::vector<Vec2d> points;
  PointsFromCurve(lane.central_curve(), &points);
  if (points.size() < 2) {
    return false;
  }
  return true;
}

bool check_crosswalk(apollo::hdmap::Crosswalk crosswalk) {
  std::vector<Vec2d> points;
  for (const auto& point : crosswalk.polygon().point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  if (points.size() < 3) {
    return false;
  }
  return true;
}

bool check_signal(apollo::hdmap::Signal signal) {
  std::vector<LineSegment2d> segments;
  for (const auto& stop_line : signal.stop_line()) {
    SegmentsFromCurve(stop_line, &segments);
  }
  if (segments.empty()) {
    return false;
  }
  std::vector<Vec2d> points;
  for (const auto &segment : segments) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  if (points.size() <= 0) {
    return false;
  }
  return true;
}

bool check_yield(apollo::hdmap::YieldSign yield) {
  std::vector<LineSegment2d> segments;
  for (const auto &stop_line : yield.stop_line()) {
    SegmentsFromCurve(stop_line, &segments);
  }
  if (segments.empty()) {
    return false;
  }
  return true;
}

bool check_clear_area(apollo::hdmap::ClearArea clear_area) {
  std::vector<Vec2d> points;
  for (const auto& point : clear_area.polygon().point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  if (points.size() < 3) {
    return false;
  }
  return true;
}

bool check_speed_bump(apollo::hdmap::SpeedBump speed_bump) {
  std::vector<LineSegment2d> segments;
  for (const auto &stop_line : speed_bump.position()) {
    SegmentsFromCurve(stop_line, &segments);
  }
  if (segments.empty()) {
    return false;
  }
  return true;
}

bool check_junction(apollo::hdmap::Junction junction) {
  std::vector<Vec2d> points;
  for (const auto& point : junction.polygon().point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  if (points.size() < 3) {
    return false;
  }
  return true;
}

bool check_stop_sign(apollo::hdmap::StopSign stop_sign) {
  std::vector<LineSegment2d> segments;
  for (const auto &stop_line : stop_sign.stop_line()) {
    SegmentsFromCurve(stop_line, &segments);
  }
  if (segments.empty()) {
    return false;
  }
  return true;
}

bool check_parking_space(apollo::hdmap::ParkingSpace parking_space) {
  std::vector<Vec2d> points;
  for (const auto& point : parking_space.polygon().point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  if (points.size() < 3) {
    return false;
  }
  return true;
}

// need to replace "AFATAL << "Unknown lane id: " << lane_id.id();"
// to "continue;" in hdmap_impl.cc
DEFINE_PROTO_FUZZER(
    const apollo::tools::fuzz::hdmap::HDmapFuzzMessage& hdmap_fuzz_message) {
  for (const auto& lane : hdmap_fuzz_message.map().lane()) {
    if (!check_lane(lane)) {
      return 0;
    }
  }
  for (const auto& crosswalk : hdmap_fuzz_message.map().crosswalk()) {
    if (!check_crosswalk(crosswalk)) {
      return 0;
    }
  }
  for (const auto& signal : hdmap_fuzz_message.map().signal()) {
    if (!check_signal(signal)) {
      return 0;
    }
  }
  for (const auto& yield : hdmap_fuzz_message.map().yield()) {
    if (!check_yield(yield)) {
      return 0;
    }
  }
  for (const auto& clear_area : hdmap_fuzz_message.map().clear_area()) {
    if (!check_clear_area(clear_area)) {
      return 0;
    }
  }
  for (const auto& speed_bump : hdmap_fuzz_message.map().speed_bump()) {
    if (!check_speed_bump(speed_bump)) {
      return 0;
    }
  }
  for (const auto& junction : hdmap_fuzz_message.map().junction()) {
    if (!check_junction(junction)) {
      return 0;
    }
  }
  for (const auto& stop_sign : hdmap_fuzz_message.map().stop_sign()) {
    if (!check_stop_sign(stop_sign)) {
      return 0;
    }
  }
  for (const auto& parking_space : hdmap_fuzz_message.map().parking_space()) {
    if (!check_parking_space(parking_space)) {
      return 0;
    }
  }
  HDMapImpl hdmap_impl;
  hdmap_impl.LoadMapFromProto(hdmap_fuzz_message.map());
}

}  // namespace hdmap
}  // namespace apollo
