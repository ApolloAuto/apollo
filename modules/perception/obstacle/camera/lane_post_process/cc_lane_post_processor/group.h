/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <limits>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/util.h"

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

  static bool Comp(const Group& a, const Group& b) {
    return (a.space_type == SpaceType::IMAGE) ? a.end_pos(1) > b.end_pos(1)
                                              : a.end_pos(1) < b.end_pos(1);
  }

  inline int ComputeOrientation(const std::vector<Marker>& markers,
                                const GroupParam& param);

  inline ScalarType ComputeDistance(const Group& tar_group,
                                    const AssociationParam& param) const;

  inline Bbox GetBbox(const std::vector<Marker>& markers) const;
};

inline int Group::ComputeOrientation(const std::vector<Marker>& markers,
                                     const GroupParam& group_param) {
  // find valid start marker number
  int start_marker_count = 0;
  for (; start_marker_count < static_cast<int>(start_marker_idx.size());
       ++start_marker_count) {
    if (start_marker_idx[start_marker_count] == -1) {
      break;
    }
  }
  ADEBUG << "start_marker_count = " << start_marker_count;

  // compute orientation for start markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = 0;
      if (start_marker_count > 1) {
        start_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             start_marker_count - 2);
      }
      int end_ind = start_marker_count - 1;
      ADEBUG << "start markers: start_ind=" << start_ind
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
      ADEBUG << "start markers: start_ind=" << start_ind
             << ", end_ind=" << end_ind;

      start_orie = markers.at(start_marker_idx.at(end_ind)).pos -
                   markers.at(start_marker_idx.at(start_ind)).start_pos;
      break;
    }

    default: { AERROR << "unknown marker shape type."; }
  }
  start_angle = std::atan2(start_orie(1), start_orie(0));
  RectAngle(&start_angle);

  // find valid end marker number
  int end_marker_count = 0;
  for (; end_marker_count < static_cast<int>(end_marker_idx.size());
       ++end_marker_count) {
    if (end_marker_idx[end_marker_count] == -1) {
      break;
    }
  }
  ADEBUG << "end_marker_count = " << end_marker_count;

  // compute orientation for end markers
  switch (shape_type) {
    case MarkerShapeType::POINT: {
      int start_ind = end_marker_count - 1;
      int end_ind = 0;
      if (end_marker_count > 1) {
        end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                           end_marker_count - 2);
      }
      ADEBUG << "end markers: start_ind=" << start_ind
             << ", end_ind=" << end_ind;

      end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                 markers.at(end_marker_idx.at(start_ind)).pos;
      break;
    }

    case MarkerShapeType::LINE_SEGMENT: {
      int start_ind = end_marker_count - 1;
      int end_ind = std::min(group_param.orientation_estimation_skip_marker_num,
                             end_marker_count - 1);
      ADEBUG << "end markers: start_ind=" << start_ind
             << ", end_ind=" << end_ind;

      end_orie = markers.at(end_marker_idx.at(end_ind)).pos -
                 markers.at(end_marker_idx.at(start_ind)).start_pos;
      break;
    }

    default: { AERROR << "unknown marker shape type."; }
  }
  end_angle = std::atan2(end_orie(1), end_orie(0));
  RectAngle(&end_angle);

  return end_marker_count;
}

inline ScalarType Group::ComputeDistance(const Group& tar_group,
                                         const AssociationParam& param) const {
  ADEBUG << "source point = (" << this->end_pos(0) << ", " << this->end_pos(1)
         << ")";
  ADEBUG << "target point = (" << tar_group.start_pos(0) << ", "
         << tar_group.start_pos(1) << ")";
  Vector2D displacement = tar_group.start_pos - this->end_pos;
  ScalarType norm_v = this->end_orie.norm();
  if (norm_v < kEpsilon) {
    ADEBUG << "norm of orientation vector for reference group is too small: "
           << norm_v;
  }

  // (1) compute the projection distance of the target marker w.r.t. model
  ScalarType projection_dist = displacement.dot(this->end_orie);
  if (norm_v >= kEpsilon) {
    projection_dist /= norm_v;
  }
  ADEBUG << "(1) projection_dist = " << std::to_string(projection_dist) << " ["
         << std::to_string(param.min_distance) << ", "
         << std::to_string(param.max_distance) << "]";
  if (projection_dist < param.min_distance ||
      projection_dist > param.max_distance) {
    return -1;
  }

  // (2) compute the departure distance to target marker
  // line equation:
  // A * x + B * y + C = 0
  // A = v_.y(), B = -v_.x(), C = v_.x() * p_.y() - v_.y() * p_.x();
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
  ADEBUG << "(2) departure_dist = " << std::to_string(departure_dist) << " ("
         << std::to_string(param.max_departure_distance) << ")";
  if (departure_dist > param.max_departure_distance) {
    return -2;
  }

  // (3) compute the deviation angle from reference marker to the target one
  // orientation angle of end markers on reference group
  ScalarType beta = this->end_angle;
  ADEBUG << "beta = " << std::to_string(beta / M_PI * 180.0);

  // angle from reference marker to the target one
  ScalarType gamma = std::atan2(displacement(1), displacement(0));
  if (gamma < 0) {
    gamma += 2 * static_cast<ScalarType>(M_PI);
  }
  ADEBUG << "gamma = " << std::to_string(gamma / M_PI * 180.0);

  ScalarType deviation_angle_dist = std::abs(beta - gamma);
  if (deviation_angle_dist > static_cast<ScalarType>(M_PI)) {
    deviation_angle_dist =
        2 * static_cast<ScalarType>(M_PI) - deviation_angle_dist;
  }
  ADEBUG << "(3) deviation_angle_dist = "
         << std::to_string(deviation_angle_dist / M_PI * 180.0);
  if (deviation_angle_dist > param.max_deviation_angle) {
    return -3;
  }

  // (4) relative orientation angle
  ScalarType tar_group_start_len = tar_group.start_orie.norm();
  ScalarType orie_dist = 0;
  ADEBUG << "(4a) tar_group_start_len = " << std::to_string(tar_group_start_len)
         << " (" << std::to_string(param.min_orientation_estimation_size)
         << ")";
  if (tar_group_start_len > param.min_orientation_estimation_size) {
    // orientation angle of start markers on target group
    ScalarType alpha = tar_group.start_angle;
    ADEBUG << "alpha = " << std::to_string(alpha / M_PI * 180.0);

    orie_dist = std::abs(alpha - beta);
    if (orie_dist > static_cast<ScalarType>(M_PI)) {
      orie_dist = 2 * static_cast<ScalarType>(M_PI) - orie_dist;
    }
    ADEBUG << "(4b) orie_dist = " << std::to_string(orie_dist / M_PI * 180.0)
           << " (" << std::to_string(param.max_relative_orie / M_PI * 180.0)
           << ")";
    if (orie_dist > param.max_relative_orie) {
      return -4;
    }
  }

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

  ADEBUG << "overall distance = " << std::to_string(dist) << "\n";
  return dist;
}

inline Bbox Group::GetBbox(const std::vector<Marker>& markers) const {
  Bbox box(std::numeric_limits<ScalarType>::max(),    // x_min
           std::numeric_limits<ScalarType>::max(),    // y_min
           -std::numeric_limits<ScalarType>::max(),   // x_max
           -std::numeric_limits<ScalarType>::max());  // y_max

  for (int i : this->marker_idx) {
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
