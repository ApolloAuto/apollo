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

#include <iostream>
#include <algorithm>

#include "modules/map/hdmap/hdmap_common.h"
#include "glog/logging.h"

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
        points[0].DistanceTo(points.back())
                            <= apollo::common::math::kMathEpsilon) {
        points.pop_back();
    }
    return apollo::common::math::Polygon2d(points);
}

void segments_from_curve(const apollo::hdmap::Curve &curve,
                    std::vector<apollo::common::math::LineSegment2d> *segments) {
    std::vector<apollo::common::math::Vec2d> points;
    points_from_curve(curve, &points);
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        segments->emplace_back(points[i], points[i + 1]);
  }
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
    CHECK(!_segments.empty());

    _sampled_left_width.clear();
    _sampled_right_width.clear();
    for (const auto &sample : _lane.left_sample()) {
        _sampled_left_width.emplace_back(sample.s(), sample.width());
    }
    for (const auto &sample : _lane.right_sample()) {
        _sampled_right_width.emplace_back(sample.s(), sample.width());
    }
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
                        const std::vector<LaneInfo::SampledWidth>& samples,
                        const double s) const {
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
    const LaneInfo::SampledWidth& sample1 = samples[low];
    const LaneInfo::SampledWidth& sample2 = samples[high];
    const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
    return sample1.second * ratio + sample2.second * (1.0 - ratio);
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
    for (const auto& segment : _segments) {
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
    // for (const auto &stop_line : _stop_sign.stop_line()) {
    //     segments_from_curve(stop_line, &_segments);
    // }
    segments_from_curve(_stop_sign.stop_line(), &_segments);
    CHECK(!_segments.empty());
}

YieldSignInfo::YieldSignInfo(const apollo::hdmap::YieldSign &yield_sign)
    : _yield_sign(yield_sign) {
    init();
}

void YieldSignInfo::init() {
    // for (const auto &stop_line : _yield_sign.stop_line()) {
    //     segments_from_curve(stop_line, &_segments);
    // }
    segments_from_curve(_yield_sign.stop_line(), &_segments);
    CHECK(!_segments.empty());
}

OverlapInfo::OverlapInfo(const apollo::hdmap::Overlap &overlap)
    : _overlap(overlap) {}

const ObjectOverlapInfo * OverlapInfo::get_object_overlap_info(
                            const apollo::hdmap::Id &id) const {
    for (const auto &object : _overlap.object()) {
        if (object.id().id() == id.id()) {
        return &object;
        }
    }
    return nullptr;
}

}  // namespace hdmap
}  // namespace apollo
