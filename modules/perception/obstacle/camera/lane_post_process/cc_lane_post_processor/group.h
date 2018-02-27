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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_GROUP_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_GROUP_H_

#include <algorithm>
#include <vector>

#include "obstacle/camera/lane_post_process/common/type.h"
#include "obstacle/camera/lane_post_process/common/util.h"

namespace apollo {
namespace perception {

struct GroupParam {
  int max_group_prediction_marker_num;
  int orientation_estimation_skip_marker_num;

  GroupParam()
      : max_group_prediction_marker_num(3),
        orientation_estimation_skip_marker_num(1) {}
};

struct Group {
  MarkerShapeType shape_type;
  SpaceType space_type;

  std::vector<int> marker_idx;

  std::vector<int> start_marker_idx;
  std::vector<Vector2D> start_pos_list;
  std::vector<Vector2D> start_orie_list;
  std::vector<ScalarType> start_angle_list;
  Vector2D start_pos;
  Vector2D start_orie;
  ScalarType start_angle;

  std::vector<int> end_marker_idx;
  std::vector<Vector2D> end_pos_list;
  std::vector<Vector2D> end_orie_list;
  std::vector<ScalarType> end_angle_list;
  Vector2D end_pos;
  Vector2D end_orie;
  ScalarType end_angle;

  Group()
      : shape_type(MarkerShapeType::LINE_SEGMENT),
        space_type(SpaceType::IMAGE),
        start_pos(0.0, 0.0),
        start_orie(0.0, -1.0),
        start_angle(static_cast<ScalarType>(-M_PI / 2.0)),
        end_pos(0.0, 0.0),
        end_orie(0.0, -1.0),
        end_angle(static_cast<ScalarType>(-M_PI / 2.0)) {
    start_marker_idx.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    start_pos_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    start_orie_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    start_angle_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    end_marker_idx.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    end_pos_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    end_orie_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
    end_angle_list.reserve(MAX_GROUP_PREDICTION_MARKER_NUM);
  }

  static bool comp(const Group& a, const Group& b) {
    return (a.space_type == SpaceType::IMAGE) ? a.end_pos(1) > b.end_pos(1)
                                              : a.end_pos(1) < b.end_pos(1);
  }

  inline int compute_orientation(const std::vector<Marker>& markers,
                                 const GroupParam& param);

  inline int compute_orientations(const std::vector<Marker>& markers,
                                  const GroupParam& param);

  inline ScalarType compute_distance(const Group& tar_group,
                                     const AssociationParam& param) const;

  inline ScalarType compute_distance(
      const Group& tar_group, const AssociationParam& param,
      int cur_group_end_marker_descend_id,
      int tar_group_start_marker_ascend_id) const;

  inline Bbox bbox(const std::vector<Marker>& markers) const;
};

inline int Group::compute_orientation(const std::vector<Marker>& markers,
                                      const GroupParam& group_param) {
  // find valid start marker number
  int start_marker_count = 0;
  for (; start_marker_count < static_cast<int>(start_marker_idx.size());
       ++start_marker_count) {
    if (start_marker_idx[start_marker_count] == -1) {
      break;
    }
  }
  XLOG(DEBUG) << "start_marker_count = " << start_marker_count;

  // compute orientation for start markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = 0;
      if (start_marker_count > 1) {
        start_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             start_marker_count - 2);
      }
      int end_ind = start_marker_count - 1;
      XLOG_IF(ERROR, start_ind > end_ind) << "start_ind (" << start_ind
                                          << ") is larger than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "start markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;

      start_orie = markers.at(start_marker_idx.at(end_ind)).pos -
                   markers.at(start_marker_idx.at(start_ind)).pos;
      break;
    }
    case MarkerShapeType::LINE_SEGMENT: {
      int start_ind =
          std::min(group_param.orientation_estimation_skip_marker_num,
                   start_marker_count - 1);
      int end_ind = start_marker_count - 1;
      XLOG_IF(ERROR, start_ind > end_ind) << "start_ind (" << start_ind
                                          << ") is larger than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "start markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;

      start_orie = markers.at(start_marker_idx.at(end_ind)).pos -
                   markers.at(start_marker_idx.at(start_ind)).start_pos;
      break;
    }
    case MarkerShapeType::POLYNOMIAL: {
      break;
    }
    default: { XLOG(FATAL) << "unknown marker shape type."; }
  }
  start_angle = std::atan2(start_orie(1), start_orie(0));
  rect_angle(start_angle);

  // find valid end marker number
  int end_marker_count = 0;
  for (; end_marker_count < static_cast<int>(end_marker_idx.size());
       ++end_marker_count) {
    if (end_marker_idx[end_marker_count] == -1) {
      break;
    }
  }
  XLOG(DEBUG) << "end_marker_count = " << end_marker_count;

  // compute orientation for end markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = end_marker_count - 1;
      int end_ind = 0;
      if (end_marker_count > 1) {
        end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                           end_marker_count - 2);
      }
      XLOG_IF(ERROR, start_ind < end_ind) << "start_ind (" << start_ind
                                          << ") is smaller than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "end markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;

      end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                 markers.at(end_marker_idx.at(start_ind)).pos;
      break;
    }
    case MarkerShapeType::LINE_SEGMENT: {
      int start_ind = end_marker_count - 1;
      int end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             end_marker_count - 1);
      XLOG_IF(ERROR, start_ind < end_ind) << "start_ind (" << start_ind
                                          << ") is smaller than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "end markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;

      end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                 markers.at(end_marker_idx.at(start_ind)).start_pos;
      break;
    }
    case MarkerShapeType::POLYNOMIAL: {
      break;
    }
    default: { XLOG(FATAL) << "unknown marker shape type."; }
  }
  end_angle = std::atan2(end_orie(1), end_orie(0));
  rect_angle(end_angle);

  return end_marker_count;
}

inline int Group::compute_orientations(const std::vector<Marker>& markers,
                                       const GroupParam& group_param) {
  // find valid start marker number
  int start_marker_count = 0;
  for (; start_marker_count < static_cast<int>(start_marker_idx.size());
       ++start_marker_count) {
    if (start_marker_idx[start_marker_count] == -1) {
      break;
    }
  }
  XLOG(DEBUG) << "start_marker_count = " << start_marker_count;

  // compute orientation for start markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = 0;
      if (start_marker_count > 1) {
        start_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             start_marker_count - 2);
      }
      int end_ind = start_marker_count - 1;
      XLOG_IF(ERROR, start_ind > end_ind) << "start_ind (" << start_ind
                                          << ") is larger than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "start markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;
      this->start_orie = markers.at(start_marker_idx.at(end_ind)).pos -
                         markers.at(start_marker_idx.at(start_ind)).pos;
      for (start_ind = 0; start_ind <= end_ind; ++start_ind) {
        this->start_pos_list.push_back(
            markers.at(start_marker_idx.at(start_ind)).pos);
        this->start_orie_list.push_back(
            markers.at(start_marker_idx.at(end_ind)).pos -
            markers.at(start_marker_idx.at(start_ind)).pos);
      }
      break;
    }
    case MarkerShapeType::LINE_SEGMENT: {
      int start_ind =
          std::min(group_param.orientation_estimation_skip_marker_num,
                   start_marker_count - 1);
      int end_ind = start_marker_count - 1;
      XLOG_IF(ERROR, start_ind > end_ind) << "start_ind (" << start_ind
                                          << ") is larger than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "start markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;
      this->start_orie = markers.at(start_marker_idx.at(end_ind)).pos -
                         markers.at(start_marker_idx.at(start_ind)).start_pos;
      for (start_ind = 0; start_ind <= end_ind; ++start_ind) {
        this->start_pos_list.push_back(
            markers.at(start_marker_idx.at(start_ind)).start_pos);
        this->start_orie_list.push_back(
            markers.at(start_marker_idx.at(end_ind)).pos -
            markers.at(start_marker_idx.at(start_ind)).start_pos);
      }
      break;
    }
    case MarkerShapeType::POLYNOMIAL: {
      break;
    }
    default: { XLOG(FATAL) << "unknown marker shape type."; }
  }
  this->start_angle = std::atan2(this->start_orie(1), this->start_orie(0));
  rect_angle(this->start_angle);
  for (auto it_start_orie = this->start_orie_list.begin();
       it_start_orie != this->start_orie_list.end(); ++it_start_orie) {
    this->start_angle_list.push_back(
        std::atan2((*it_start_orie)(1), (*it_start_orie)(0)));
    rect_angle(this->start_angle_list.back());
  }

  // find valid end marker number
  int end_marker_count = 0;
  for (; end_marker_count < static_cast<int>(end_marker_idx.size());
       ++end_marker_count) {
    if (end_marker_idx[end_marker_count] == -1) {
      break;
    }
  }
  XLOG(DEBUG) << "end_marker_count = " << end_marker_count;
  CHECK_EQ(start_marker_count, end_marker_count);

  // compute orientation for end markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = end_marker_count - 1;
      int end_ind = 0;
      if (end_marker_count > 1) {
        end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                           end_marker_count - 2);
      }
      XLOG_IF(ERROR, start_ind < end_ind) << "start_ind (" << start_ind
                                          << ") is smaller than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "end markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;
      this->end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                       markers.at(end_marker_idx.at(start_ind)).pos;
      for (end_ind = 0; end_ind <= start_ind; ++end_ind) {
        this->end_pos_list.push_back(
            markers.at(end_marker_idx.at(end_ind)).pos);
        this->end_orie_list.push_back(
            markers.at(end_marker_idx.at(end_ind)).pos -
            markers.at(end_marker_idx.at(start_ind)).pos);
      }
      break;
    }
    case MarkerShapeType::LINE_SEGMENT: {
      int start_ind = end_marker_count - 1;
      int end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             end_marker_count - 1);
      XLOG_IF(ERROR, start_ind < end_ind) << "start_ind (" << start_ind
                                          << ") is smaller than end_ind ("
                                          << end_ind << ")";
      XLOG(DEBUG) << "end markers: start_ind=" << start_ind
                  << ", end_ind=" << end_ind;
      this->end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                       markers.at(end_marker_idx.at(start_ind)).start_pos;
      for (end_ind = 0; end_ind <= start_ind; ++end_ind) {
        this->end_pos_list.push_back(
            markers.at(end_marker_idx.at(end_ind)).pos);
        this->end_orie_list.push_back(
            markers.at(end_marker_idx.at(end_ind)).pos -
            markers.at(end_marker_idx.at(start_ind)).start_pos);
      }
      break;
    }
    case MarkerShapeType::POLYNOMIAL: {
      break;
    }
    default: { XLOG(FATAL) << "unknown marker shape type."; }
  }
  this->end_angle = std::atan2(this->end_orie(1), this->end_orie(0));
  rect_angle(this->end_angle);
  for (auto it_end_orie = this->end_orie_list.begin();
       it_end_orie != this->end_orie_list.end(); ++it_end_orie) {
    this->end_angle_list.push_back(
        std::atan2((*it_end_orie)(1), (*it_end_orie)(0)));
    rect_angle(this->end_angle_list.back());
  }

  return end_marker_count;
}

inline ScalarType Group::compute_distance(const Group& tar_group,
                                          const AssociationParam& param) const {
  XLOG(DEBUG) << "source point = (" << this->end_pos(0) << ", "
              << this->end_pos(1) << ")";
  XLOG(DEBUG) << "target point = (" << tar_group.start_pos(0) << ", "
              << tar_group.start_pos(1) << ")";
  Vector2D displacement = tar_group.start_pos - this->end_pos;
  ScalarType norm_v = this->end_orie.norm();
  if (norm_v < kEpsilon) {
    XLOG(DEBUG)
        << "norm of orientation vector for reference group is too small: "
        << norm_v;
  }

  // (1) compute the projection distance of the target marker w.r.t. model
  ScalarType projection_dist = displacement.dot(this->end_orie);
  if (norm_v >= kEpsilon) {
    projection_dist /= norm_v;
  }
  XLOG(DEBUG) << "(1) projection_dist = " << std::to_string(projection_dist)
              << " [" << std::to_string(param.min_distance) << ", "
              << std::to_string(param.max_distance) << "]";
  if (projection_dist < param.min_distance ||
      projection_dist > param.max_distance) {
    return -1;
  }

  // (2) compute the departure distance to target marker
  // line equation:
  // A * x + B * y + C = 0
  // A = _v.y(), B = -_v.x(), C = _v.x() * _p.y() - _v.y() * _p.x();
  // the normal distance from a point (x2, y2) to the line:
  // d = |A * x2 + B * y2 + C| / sqrt(A^2 + B^2)
  ScalarType departure_dist =
      std::abs(this->end_orie.y() * tar_group.start_pos.x() -
               this->end_orie.x() * tar_group.start_pos.y() +
               this->end_orie.x() * this->end_pos.y() -
               this->end_orie.y() * this->end_pos.x());
  if (norm_v >= kEpsilon) {
    departure_dist /= norm_v;
  }
  XLOG(DEBUG) << "(2) departure_dist = " << std::to_string(departure_dist)
              << " (" << std::to_string(param.max_departure_distance) << ")";
  if (departure_dist > param.max_departure_distance) {
    return -2;
  }

  // (3) compute the deviation angle from reference marker to the target one
  // orientation angle of end markers on reference group
  ScalarType beta = this->end_angle;
  XLOG(DEBUG) << "beta = " << std::to_string(beta / M_PI * 180.0);

  // angle from reference marker to the target one
  ScalarType gamma = std::atan2(displacement(1), displacement(0));
  XLOG_IF(ERROR, gamma < -static_cast<ScalarType>(M_PI) ||
                     gamma > static_cast<ScalarType>(M_PI))
      << "gamma is out range of [-pi, pi]: " << gamma;
  if (gamma < 0) {
    gamma += 2 * static_cast<ScalarType>(M_PI);
  }
  XLOG(DEBUG) << "gamma = " << std::to_string(gamma / M_PI * 180.0);

  ScalarType deviation_angle_dist = std::abs(beta - gamma);
  XLOG_IF(ERROR, deviation_angle_dist < 0 ||
                     deviation_angle_dist > 2 * static_cast<ScalarType>(M_PI))
      << "deviation_angle_dist is out range of [0, 2*pi]: "
      << deviation_angle_dist;
  if (deviation_angle_dist > static_cast<ScalarType>(M_PI)) {
    deviation_angle_dist =
        2 * static_cast<ScalarType>(M_PI) - deviation_angle_dist;
  }
  XLOG_IF(ERROR, deviation_angle_dist < 0 ||
                     deviation_angle_dist > static_cast<ScalarType>(M_PI))
      << "deviation_angle_dist is out range of [0, pi]: "
      << deviation_angle_dist;
  XLOG(DEBUG) << "(3) deviation_angle_dist = "
              << std::to_string(deviation_angle_dist / M_PI * 180.0);
  if (deviation_angle_dist > param.max_deviation_angle) {
    return -3;
  }

  // (4) relative orientation angle
  ScalarType tar_group_start_len = tar_group.start_orie.norm();
  ScalarType orie_dist = 0;
  XLOG(DEBUG) << "(4a) tar_group_start_len = "
              << std::to_string(tar_group_start_len) << " ("
              << std::to_string(param.min_orientation_estimation_size) << ")";
  if (tar_group_start_len > param.min_orientation_estimation_size) {
    // orientation angle of start markers on target group
    ScalarType alpha = tar_group.start_angle;
    XLOG(DEBUG) << "alpha = " << std::to_string(alpha / M_PI * 180.0);

    orie_dist = std::abs(alpha - beta);
    XLOG_IF(ERROR,
            orie_dist < 0 || orie_dist > 2 * static_cast<ScalarType>(M_PI))
        << "orie_dist is out range of [0, 2*pi]: " << orie_dist;
    if (orie_dist > static_cast<ScalarType>(M_PI)) {
      orie_dist = 2 * static_cast<ScalarType>(M_PI) - orie_dist;
    }
    XLOG_IF(ERROR, orie_dist < 0 || orie_dist > static_cast<ScalarType>(M_PI))
        << "orie_dist is out range of [0, pi]: " << orie_dist;
    XLOG(DEBUG) << "(4b) orie_dist = "
                << std::to_string(orie_dist / M_PI * 180.0) << " ("
                << std::to_string(param.max_relative_orie / M_PI * 180.0)
                << ")";
    if (orie_dist > param.max_relative_orie) {
      return -4;
    }
  }

  //
  ScalarType r =
      std::max(std::abs(param.min_distance), std::abs(param.max_distance));
  if (r > kEpsilon) {
    projection_dist = std::abs(projection_dist) / r;
  }
  if (param.max_departure_distance > kEpsilon) {
    departure_dist /= param.max_departure_distance;
  }
  if (param.max_deviation_angle > kEpsilon) {
    deviation_angle_dist /= param.max_deviation_angle;
  }
  if (param.max_relative_orie > kEpsilon) {
    orie_dist /= param.max_relative_orie;
  }

  ScalarType dist = param.distance_weight * projection_dist +
                    param.departure_distance_weight * departure_dist +
                    param.deviation_angle_weight * deviation_angle_dist +
                    param.relative_orie_weight * orie_dist;

  XLOG_IF(ERROR, !std::isfinite(dist)) << "the distance value is infinite.";
  XLOG_IF(ERROR, dist < 0) << "the distance value is negative: "
                           << std::to_string(dist);

  XLOG(DEBUG) << "dist = " << std::to_string(dist) << "\n";
  return dist;
}

inline ScalarType Group::compute_distance(
    const Group& tar_group, const AssociationParam& param,
    int cur_group_end_marker_descend_id,
    int tar_group_start_marker_ascend_id) const {
  XLOG(DEBUG) << "cur_group_end_marker_descend_id="
              << cur_group_end_marker_descend_id;
  XLOG_IF(ERROR, cur_group_end_marker_descend_id < 0)
      << "cur_group_end_marker_descend_id = "
      << cur_group_end_marker_descend_id;
  XLOG_IF(ERROR, cur_group_end_marker_descend_id >=
                     static_cast<int>(this->end_pos_list.size()))
      << "cur_group_end_marker_descend_id = "
      << cur_group_end_marker_descend_id;

  Vector2D cur_group_end_pos =
      this->end_pos_list.at(cur_group_end_marker_descend_id);
  Vector2D cur_group_end_orie =
      this->end_orie_list.at(cur_group_end_marker_descend_id);

  XLOG(DEBUG) << "tar_group_start_marker_ascend_id="
              << tar_group_start_marker_ascend_id;
  XLOG_IF(ERROR, tar_group_start_marker_ascend_id < 0)
      << "tar_group_start_marker_ascend_id = "
      << tar_group_start_marker_ascend_id;
  XLOG_IF(ERROR, tar_group_start_marker_ascend_id >=
                     static_cast<int>(tar_group.start_pos_list.size()))
      << "tar_group_start_marker_ascend_id = "
      << tar_group_start_marker_ascend_id;

  Vector2D tar_group_start_pos =
      tar_group.start_pos_list.at(tar_group_start_marker_ascend_id);
  Vector2D tar_group_start_orie =
      tar_group.start_orie_list.at(tar_group_start_marker_ascend_id);

  XLOG(DEBUG) << "source point = (" << cur_group_end_pos(0) << ", "
              << cur_group_end_pos(1) << ")";
  XLOG(DEBUG) << "target point = (" << tar_group_start_pos(0) << ", "
              << tar_group_start_pos(1) << ")";
  Vector2D displacement = tar_group_start_pos - cur_group_end_pos;
  ScalarType norm_v = cur_group_end_orie.norm();
  if (norm_v < kEpsilon) {
    XLOG(INFO)
        << "norm of orientation vector for reference group is too small: "
        << norm_v;
  }

  // (1) compute the projection distance of the target marker w.r.t. model
  ScalarType projection_dist = displacement.dot(cur_group_end_orie);
  if (norm_v >= kEpsilon) {
    projection_dist /= norm_v;
  }
  XLOG(DEBUG) << "(1) projection_dist = " << std::to_string(projection_dist)
              << " [" << std::to_string(param.min_distance) << ", "
              << std::to_string(param.max_distance) << "]";
  if (projection_dist < param.min_distance ||
      projection_dist > param.max_distance) {
    return -1;
  }

  // (2) compute the departure distance to target marker
  // line equation:
  // A * x + B * y + C = 0
  // A = _v.y(), B = -_v.x(), C = _v.x() * _p.y() - _v.y() * _p.x();
  // the normal distance from a point (x2, y2) to the line:
  // d = |A * x2 + B * y2 + C| / sqrt(A^2 + B^2)
  ScalarType departure_dist =
      std::abs(cur_group_end_orie.y() * tar_group_start_pos.x() -
               cur_group_end_orie.x() * tar_group_start_pos.y() +
               cur_group_end_orie.x() * cur_group_end_pos.y() -
               cur_group_end_orie.y() * cur_group_end_pos.x());
  if (norm_v >= kEpsilon) {
    departure_dist /= norm_v;
  }
  XLOG(DEBUG) << "(2) departure_dist = " << std::to_string(departure_dist)
              << " (" << std::to_string(param.max_departure_distance) << ")";
  if (departure_dist > param.max_departure_distance) {
    return -2;
  }

  // (3) compute the deviation angle from reference marker to the target one
  // orientation angle of end marker on reference group
  ScalarType beta = this->end_angle_list.at(cur_group_end_marker_descend_id);
  XLOG(DEBUG) << "beta = " << std::to_string(beta / M_PI * 180.0);

  // angle from reference marker to the target one
  ScalarType gamma = std::atan2(displacement(1), displacement(0));
  XLOG_IF(ERROR, gamma < -static_cast<ScalarType>(M_PI) ||
                     gamma > static_cast<ScalarType>(M_PI))
      << "gamma is out range of [-pi, pi]: " << gamma;
  if (gamma < 0) {
    gamma += 2 * static_cast<ScalarType>(M_PI);
  }
  XLOG(DEBUG) << "gamma = " << std::to_string(gamma / M_PI * 180.0);

  ScalarType deviation_angle_dist = std::abs(beta - gamma);
  XLOG_IF(ERROR, deviation_angle_dist < 0 ||
                     deviation_angle_dist > 2 * static_cast<ScalarType>(M_PI))
      << "deviation_angle_dist is out range of [0, 2*pi]: "
      << deviation_angle_dist;
  if (deviation_angle_dist > static_cast<ScalarType>(M_PI)) {
    deviation_angle_dist =
        2 * static_cast<ScalarType>(M_PI) - deviation_angle_dist;
  }
  XLOG_IF(ERROR, deviation_angle_dist < 0 ||
                     deviation_angle_dist > static_cast<ScalarType>(M_PI))
      << "deviation_angle_dist is out range of [0, pi]: "
      << deviation_angle_dist;
  XLOG(DEBUG) << "(3) deviation_angle_dist = "
              << std::to_string(deviation_angle_dist / M_PI * 180.0);
  if (deviation_angle_dist > param.max_deviation_angle) {
    return -3;
  }

  // (4) relative orientation angle
  ScalarType tar_group_start_len = tar_group_start_orie.norm();
  ScalarType orie_dist = 0;
  XLOG(DEBUG) << "(4a) tar_group_start_len = "
              << std::to_string(tar_group_start_len) << " ("
              << std::to_string(param.min_orientation_estimation_size) << ")";
  if (tar_group_start_len > param.min_orientation_estimation_size) {
    // orientation angle of start markers on target group
    ScalarType alpha =
        tar_group.start_angle_list.at(tar_group_start_marker_ascend_id);
    XLOG(DEBUG) << "alpha = " << std::to_string(alpha / M_PI * 180.0);

    orie_dist = std::abs(alpha - beta);
    XLOG_IF(ERROR,
            orie_dist < 0 || orie_dist > 2 * static_cast<ScalarType>(M_PI))
        << "orie_dist is out range of [0, 2*pi]: " << orie_dist;
    if (orie_dist > static_cast<ScalarType>(M_PI)) {
      orie_dist = 2 * static_cast<ScalarType>(M_PI) - orie_dist;
    }
    XLOG_IF(ERROR, orie_dist < 0 || orie_dist > static_cast<ScalarType>(M_PI))
        << "orie_dist is out range of [0, pi]: " << orie_dist;
    XLOG(DEBUG) << "(4b) orie_dist = "
                << std::to_string(orie_dist / M_PI * 180.0) << " ("
                << std::to_string(param.max_relative_orie / M_PI * 180.0)
                << ")";
    if (orie_dist > param.max_relative_orie) {
      return -4;
    }
  }

  //
  ScalarType r =
      std::max(std::abs(param.min_distance), std::abs(param.max_distance));
  if (r > kEpsilon) {
    projection_dist = std::abs(projection_dist) / r;
  }
  if (param.max_departure_distance > kEpsilon) {
    departure_dist /= param.max_departure_distance;
  }
  if (param.max_deviation_angle > kEpsilon) {
    deviation_angle_dist /= param.max_deviation_angle;
  }
  if (param.max_relative_orie > kEpsilon) {
    orie_dist /= param.max_relative_orie;
  }

  ScalarType dist = param.distance_weight * projection_dist +
                    param.departure_distance_weight * departure_dist +
                    param.deviation_angle_weight * deviation_angle_dist +
                    param.relative_orie_weight * orie_dist;

  XLOG_IF(ERROR, !std::isfinite(dist)) << "the distance value is infinite.";
  XLOG_IF(ERROR, dist < 0) << "the distance value is negative: "
                           << std::to_string(dist);

  XLOG(DEBUG) << "dist = " << std::to_string(dist) << "\n";
  return dist;
}

inline Bbox Group::bbox(const std::vector<Marker>& markers) const {
  Bbox box(std::numeric_limits<ScalarType>::max(),    // x_min
           std::numeric_limits<ScalarType>::max(),    // y_min
           -std::numeric_limits<ScalarType>::max(),   // x_max
           -std::numeric_limits<ScalarType>::max());  // y_max

  for (int i : this->marker_idx) {
    assert(i >= 0 && i < static_cast<int>(markers.size()));
    box(0) = std::min(box(0), markers[i].pos.x());
    box(1) = std::min(box(1), markers[i].pos.y());
    box(2) = std::max(box(2), markers[i].pos.x());
    box(3) = std::max(box(3), markers[i].pos.y());

    if (markers[i].shape_type != MarkerShapeType::POINT) {
      box(0) = std::min(box(0), markers[i].start_pos.x());
      box(1) = std::min(box(1), markers[i].start_pos.y());
      box(2) = std::max(box(2), markers[i].start_pos.x());
      box(3) = std::max(box(3), markers[i].start_pos.y());
    }
  }

  return box;
}

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_CC_LANE_POST_PROCESSOR_GROUP_H_
