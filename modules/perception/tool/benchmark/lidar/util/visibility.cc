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
#include "modules/perception/tool/benchmark/lidar/util/visibility.h"
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace apollo {
namespace perception {
namespace benchmark {

std::map<int, std::vector<size_t>> Visibility::s_lut_{
    {-4, {1, 2, 3}}, {-3, {2, 3}},   {-2, {2, 3, 0}}, {-1, {1, 2}},   {0, {}},
    {1, {3, 0}},     {2, {0, 1, 2}}, {3, {0, 1}},     {4, {3, 0, 1}},
};

void Visibility::set_car_pos(const Eigen::Vector3d& car_pos) {
  car_pos_ = car_pos;
}

void Visibility::fill_objects(std::vector<ObjectPtr>* objs, float thresh) {
  auto get_interval = [](float num, float thresh) -> int {
    if (num < -thresh) {
      return -1;
    } else if (num > thresh) {
      return 1;
    } else {
      return 0;
    }
  };

  auto point_in_rectangle = [this](const VisPoint& point) -> bool {
    const float x = point.x();
    const float y = point.y();
    return x < half_length_ && x > -half_length_ && y < half_width_ &&
           y > -half_width_;
  };

  reset_state();
  add_region();
  full_visual_angle_.reserve(objs->size());
  for (size_t i = 0; i < objs->size(); ++i) {
    const ObjectPtr& obj = objs->at(i);
    float theta =
        static_cast<float>(atan2(obj->direction(1), obj->direction(0)));
    float cos_v = static_cast<float>(cos(theta));
    float sin_v = static_cast<float>(sin(theta));
    Eigen::Matrix2f rotation_mat;
    rotation_mat << cos_v, -sin_v, sin_v, cos_v;
    Eigen::Vector3f transition_vec;
    transition_vec << static_cast<float>(obj->center.x() - car_pos_(0)),
        static_cast<float>(obj->center.y() - car_pos_(1)), 1;

    Eigen::Matrix3f trans_mat = Eigen::Matrix3f::Zero();
    trans_mat.topLeftCorner(2, 2) = rotation_mat;
    trans_mat.block<3, 1>(0, 2) = transition_vec;

    float half_length = static_cast<float>(obj->length / 2);
    float half_width = static_cast<float>(obj->width / 2);
    Eigen::Vector2f origin_in_obj_cord =
        (trans_mat.inverse() * Eigen::Vector3f(0, 0, 1)).head(2);
    int pos = get_interval(origin_in_obj_cord(0), half_length) * 3 +
              get_interval(origin_in_obj_cord(1), half_width);
    const auto& point_pos = s_lut_[pos];

    std::vector<VisPoint> points_vec;
    VisPoint p0(
        (trans_mat * Eigen::Vector3f(half_length, half_width, 1)).head(2));
    VisPoint p1(
        (trans_mat * Eigen::Vector3f(half_length, -half_width, 1)).head(2));
    VisPoint p2(
        (trans_mat * Eigen::Vector3f(-half_length, -half_width, 1)).head(2));
    VisPoint p3(
        (trans_mat * Eigen::Vector3f(-half_length, half_width, 1)).head(2));
    points_vec.push_back(p0), points_vec.push_back(p1);
    points_vec.push_back(p2), points_vec.push_back(p3);

    bool should_do = true;
    for (size_t i = 0; i < points_vec.size(); ++i) {
      if (point_pos.empty() || !point_in_rectangle(points_vec[i])) {
        should_do = false;
        break;
      }
    }
    if (!should_do) {
      full_visual_angle_.push_back(0);
      continue;
    }

    for (size_t j = 1; j < point_pos.size(); ++j) {
      add_segment(points_vec[point_pos[j - 1]], points_vec[point_pos[j]],
                  static_cast<int>(i));
    }

    full_visual_angle_.push_back(calculate_visual_angle(
        points_vec[*point_pos.begin()], points_vec[*point_pos.rbegin()]));
  }
  visual_angle_.assign(objs->size(), 0);
  calculate(objs, thresh);
}

float Visibility::calculate(std::vector<ObjectPtr>* objs, float thresh) {
  if (points_.empty()) {
    return 0.0;
  }

  latest_query_segments_cache_.reserve(points_.size());

  VisPoint pending_p, last_p;
  Segment pending_seg;

  bool init = false;
  for (auto it = points_.begin(); it != points_.end(); ++it) {
    VisPoint current_p = it->first;

    if (!init) {
      update_candidate_segment(current_p, UpdateOperation::add);
      if (candidate_segment_.empty()) {
        continue;
      }
      pending_p = current_p;
      pending_seg = *candidate_segment_.begin();
      last_p = current_p;
      init = true;
      continue;
    }

    if (current_p == last_p) {
      continue;
    }

    update_candidate_segment(current_p, UpdateOperation::add);
    update_candidate_segment(current_p, UpdateOperation::remove);

    Segment current_seg = *candidate_segment_.begin();
    if (current_p == pending_seg.end) {
      area_sum_ += calculate_area(pending_p, current_p);
      if (!(pending_seg.idx < 0)) {
        visual_angle_[static_cast<size_t>(pending_seg.idx)] +=
            calculate_visual_angle(pending_p, current_p);
      }

      pending_seg = current_seg;
      if (current_p == current_seg.start) {
        pending_p = current_p;
      } else {
        VisPoint intersection;
        intersects(current_p, current_seg, &intersection);
        pending_p = intersection;
      }
    } else if (pending_seg != current_seg) {
      VisPoint intersection;
      intersects(current_p, pending_seg, &intersection);
      area_sum_ += calculate_area(pending_p, intersection);
      if (!(pending_seg.idx < 0)) {
        visual_angle_[static_cast<size_t>(pending_seg.idx)] +=
            calculate_visual_angle(pending_p, intersection);
      }

      pending_p = current_p;
      pending_seg = current_seg;
    }

    last_p = current_p;
  }

  area_sum_ += calculate_area(pending_p, pending_seg.end);
  if (!(pending_seg.idx < 0)) {
    visual_angle_[static_cast<size_t>(pending_seg.idx)] +=
        calculate_visual_angle(pending_p, pending_seg.end);
  }

  float all_area = 4 * half_length_ * half_width_;
  float ret = area_sum_ / all_area;

  for (size_t i = 0; i < objs->size(); ++i) {
    float ratio = visual_angle_[i] / full_visual_angle_[i];
    if (std::isfinite(ratio)) {
      objs->at(i)->visible_ratio = ratio < 1 ? ratio : 1;
    } else {
      objs->at(i)->visible_ratio = 0.0;
    }
    objs->at(i)->visible = objs->at(i)->visible_ratio >= thresh;
  }

  return ret;
}

void Visibility::reset_state() {
  latest_query_point_ = VisPoint(0, 0);
  latest_query_segments_cache_.clear();
  points_.clear();
  candidate_segment_.clear();
  full_visual_angle_.clear();
  visual_angle_.clear();
}

void Visibility::add_region() {
  VisPoint a(half_length_, half_width_);
  VisPoint b(half_length_, -half_width_);
  VisPoint c(-half_length_, -half_width_);
  VisPoint d(-half_length_, half_width_);

  add_segment(a, b, -1);
  add_segment(b, c, -1);
  add_segment(c, d, -1);
  add_segment(d, a, -1);
}

void Visibility::add_segment(const VisPoint& a, const VisPoint& b, int idx) {
  if (compute_orientation(VisPoint(0, 0), a, b) == Orientation::collinear) {
    return;
  }

  if (approx_equal(a.x(), 0.0) || approx_equal(b.x(), 0.0)) {
    VisPoint ta = a, tb = b;
    if (!approx_equal(ta.x(), 0.0)) {
      std::swap(ta, tb);
    }
    Segment seg = tb.x() < 0 ? Segment(tb, ta, idx) : Segment(ta, tb, idx);
    points_.insert(std::make_pair(ta, seg));
    points_.insert(std::make_pair(tb, seg));
    return;
  }

  VisPoint intersection;
  bool axis_y_intersect =
      intersects(VisPoint(0, 1), Segment(a, b), &intersection);
  if (axis_y_intersect) {
    Segment seg1 = a.x() < 0 ? Segment(a, intersection, idx)
                             : Segment(intersection, a, idx);
    Segment seg2 = b.x() < 0 ? Segment(b, intersection, idx)
                             : Segment(intersection, b, idx);
    points_.insert(std::make_pair(a, seg1));
    points_.insert(std::make_pair(b, seg2));
    points_.insert(std::make_pair(intersection, seg1));
    points_.insert(std::make_pair(intersection, seg2));
  } else {
    Segment seg = a < b ? Segment(a, b, idx) : Segment(b, a, idx);
    points_.insert(std::make_pair(a, seg));
    points_.insert(std::make_pair(b, seg));
  }
}

void Visibility::query_segments(const VisPoint& p) {
  if (p == latest_query_point_) {
    return;
  }
  latest_query_segments_cache_.clear();
  auto ret = points_.equal_range(p);
  auto it = ret.first;
  while (it != ret.second) {
    latest_query_segments_cache_.push_back(it->second);
    ++it;
  }
  latest_query_point_ = p;
}

void Visibility::update_candidate_segment(const VisPoint& p,
                                          UpdateOperation op) {
  query_segments(p);

  auto func = (op == UpdateOperation::add
                   ? [](const Segment& seg,
                        std::set<Segment>& segments) { segments.insert(seg); }
                   : [](const Segment& seg, std::set<Segment>& segments) {
                       segments.erase(seg);
                     });
  auto update = [this, func](const Segment& seg) {
    return func(seg, candidate_segment_);
  };

  for (const auto& seg : latest_query_segments_cache_) {
    VisPoint p_pending = (op == UpdateOperation::add ? seg.start : seg.end);
    if (p_pending == p) {
      update(seg);
    }
  }
}

float Visibility::calculate_area(const VisPoint& a, const VisPoint& b) {
  float dot = a.dot(b);
  float area = static_cast<float>(
      sqrt(a.length_squared() * b.length_squared() - dot * dot) / 2);
  area = static_cast<float>(std::isfinite(area) ? area : 0.0);
  return area;
}

float Visibility::calculate_visual_angle(const VisPoint& a, const VisPoint& b) {
  float cos_theta = static_cast<float>(
      a.dot(b) / sqrt(a.length_squared() * b.length_squared()));
  float theta =
      static_cast<float>(approx_equal(cos_theta, 1.0) ? 0.0 : acos(cos_theta));
  // theta = theta * 180 / M_PI;
  return theta;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
