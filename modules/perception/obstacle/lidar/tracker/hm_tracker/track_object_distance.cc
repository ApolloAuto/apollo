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

#include <algorithm>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/obstacle/common/geometry_util.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {

double TrackObjectDistance::s_location_distance_weight_ = 0.6;
double TrackObjectDistance::s_direction_distance_weight_ = 0.2;
double TrackObjectDistance::s_bbox_size_distance_weight_ = 0.1;
double TrackObjectDistance::s_point_num_distance_weight_ = 0.1;
double TrackObjectDistance::s_histogram_distance_weight_ = 0.5;

bool TrackObjectDistance::SetLocationDistanceWeight(
  const float& location_distance_weight) {
  if (location_distance_weight >= 0) {
    s_location_distance_weight_ = location_distance_weight;
    AINFO << "location distance weight of TrackObjectDistance is "
          << s_location_distance_weight_;
    return true;
  }
  AERROR << "invalid location distance weight of TrackeObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetDirectionDistanceWeight(
  const float& direction_distance_weight) {
  if (direction_distance_weight >= 0) {
    s_direction_distance_weight_ = direction_distance_weight;
    AINFO << "direction distance weight of TrackObjectDistance is "
          << s_direction_distance_weight_;
    return true;
  }
  AERROR << "invalid direction distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetBboxSizeDistanceWeight(
  const float& bbox_size_distance_weight) {
  if (bbox_size_distance_weight >= 0) {
    s_bbox_size_distance_weight_ = bbox_size_distance_weight;
    AINFO << "bbox size distance weight of TrackObjectDistance is "
          << s_bbox_size_distance_weight_;
    return true;
  }
  AERROR << "invalid bbox size distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetPointNumDistanceWeight(
  const float& point_num_distance_weight) {
  if (point_num_distance_weight >= 0) {
    s_point_num_distance_weight_ = point_num_distance_weight;
    AINFO << "point num distance weight of TrackObjectDistance is "
          << s_point_num_distance_weight_;
    return true;
  }
  AERROR << "invalid point num distance weight of TrackObjectDistance!";
  return false;
}

bool TrackObjectDistance::SetHistogramDistanceWeight(
  const float& histogram_distance_weight) {
  if (histogram_distance_weight >= 0) {
    s_histogram_distance_weight_ = histogram_distance_weight;
    AINFO << "histogram distance weight of TrackObjectDistance is "
          << s_histogram_distance_weight_;
    return true;
  }
  AERROR << "invalid histogram distance weight of TrackObjectDistance!";
  return false;
}

float TrackObjectDistance::ComputeDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute distance for given trakc & object
  float location_dist = ComputeLocationDistance(track, track_predict,
    new_object, time_diff);
  float direction_dist = ComputeDirectionDistance(track, track_predict,
    new_object, time_diff);
  float bbox_size_dist = ComputeBboxSizeDistance(track, track_predict,
    new_object, time_diff);
  float point_num_dist = ComputePointNumDistance(track, track_predict,
    new_object, time_diff);
  float histogram_dist = ComputeHistogramDistance(track, track_predict,
    new_object, time_diff);

  float result_dist = s_location_distance_weight_ * location_dist +
    s_direction_distance_weight_ * direction_dist +
    s_bbox_size_distance_weight_ * bbox_size_dist +
    s_point_num_distance_weight_ * point_num_dist +
    s_histogram_distance_weight_ * histogram_dist;

  return result_dist;
}

float TrackObjectDistance::ComputeLocationDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute locatin distance for given track & object
  // range from 0 to positive infinity
  const TrackedObjectPtr& last_object = track->current_object_;

  Eigen::Vector3f measured_anchor_point = new_object->anchor_point;
  Eigen::Vector3f predicted_anchor_point = track_predict.head(3);
  Eigen::Vector3f measure_predict_diff = measured_anchor_point -
    predicted_anchor_point;

  float location_dist = sqrt((measure_predict_diff.head(2).cwiseProduct(
    measure_predict_diff.head(2))).sum());

  /* NEED TO NOTICE: All the states would be collected mainly based on states of tracked objects.
   * Thus, update tracked object when you update the state of track !!!!! */
  Eigen::Vector2f ref_dir = last_object->velocity.head(2);
  float speed = ref_dir.norm();
  ref_dir /= speed;

  /* Let location distance generated from a normal distribution with symmetric variance. 
   * Modify its variance when speed greater than a threshold. Penalize variance other than 
   * motion direction. */
  if (speed > 2) {
    Eigen::Vector2f ref_o_dir = Eigen::Vector2f(ref_dir[1], -ref_dir[0]);
    float dx = ref_dir(0) * measure_predict_diff(0) + ref_dir(1) *
      measure_predict_diff(1);
    float dy = ref_o_dir(0) * measure_predict_diff(0) + ref_o_dir(1) *
      measure_predict_diff(1);
    location_dist = sqrt(dx * dx * 0.25 + dy * dy * 4);
  }

  return location_dist;
}

float TrackObjectDistance::ComputeDirectionDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute direction distance for given track & object
  // range from 0 to 2
  const TrackedObjectPtr& last_object = track->current_object_;

  Eigen::Vector3f old_anchor_point = last_object->anchor_point;
  Eigen::Vector3f new_anchor_point = new_object->anchor_point;
  Eigen::Vector3f anchor_point_shift_dir = new_anchor_point - old_anchor_point;
  anchor_point_shift_dir[2] = 0.0;

  Eigen::Vector3f track_motion_dir = track_predict.head(6).tail(3);
  track_motion_dir[2] = 0.0;

  double cos_theta = 0.994;  // average cos
  if (!track_motion_dir.head(2).isZero() &&
    !anchor_point_shift_dir.head(2).isZero()) {
    cos_theta = VectorCosTheta2dXy(track_motion_dir,
      anchor_point_shift_dir);
  }
  float direction_dist = -cos_theta + 1.0;

  return direction_dist;
}

float TrackObjectDistance::ComputeBboxSizeDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute bbox size distance for given track & object
  // range from 0 to 1
  const TrackedObjectPtr& last_object = track->current_object_;

  Eigen::Vector3f old_bbox_dir = last_object->direction;
  Eigen::Vector3f new_bbox_dir = new_object->direction;
  Eigen::Vector3f old_bbox_size = last_object->size;
  Eigen::Vector3f new_bbox_size = new_object->size;

  float size_dist = 0.0;
  double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) +
    old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) -
    old_bbox_dir(1) * new_bbox_dir(0));

  if (dot_val_00 > dot_val_01) {
    float temp_val_0 = fabs(old_bbox_size(0) - new_bbox_size(0)) /
      std::max(old_bbox_size(0), new_bbox_size(0));
    float temp_val_1 = fabs(old_bbox_size(1) - new_bbox_size(1)) /
      std::max(old_bbox_size(1), new_bbox_size(1));
    size_dist = std::min(temp_val_0, temp_val_1);
  } else {
    float temp_val_0 = fabs(old_bbox_size(0) - new_bbox_size(1)) /
      std::max(old_bbox_size(0), new_bbox_size(1));
    float temp_val_1 = fabs(old_bbox_size(1) - new_bbox_size(0)) /
      std::max(old_bbox_size(1), new_bbox_size(0));
    size_dist = std::min(temp_val_0, temp_val_1);
  }

  return size_dist;
}

float TrackObjectDistance::ComputePointNumDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute point num distance for given track & object
  // range from 0 and 1
  const TrackedObjectPtr& last_object = track->current_object_;

  int old_point_number = last_object->object_ptr->cloud->size();
  int new_point_number = new_object->object_ptr->cloud->size();

  float point_num_dist = fabs(old_point_number - new_point_number) * 1.0f /
    std::max(old_point_number, new_point_number);

  return point_num_dist;
}

float TrackObjectDistance::ComputeHistogramDistance(const ObjectTrackPtr& track,
  const Eigen::VectorXf& track_predict,
  const TrackedObjectPtr& new_object,
  const double& time_diff) {
  // Compute histogram distance for given track & object
  // range from 0 to 3
  const TrackedObjectPtr& last_object = track->current_object_;

  std::vector<float>& old_object_shape_features =
    last_object->object_ptr->shape_features;
  std::vector<float>& new_object_shape_features =
    new_object->object_ptr->shape_features;

  if (old_object_shape_features.size() != new_object_shape_features.size()) {
    AERROR << "sizes of compared features not matched. TrackObjectDistance";
    return 100;
  }

  float histogram_dist = 0.0;
  for (size_t i = 0; i < old_object_shape_features.size(); ++i) {
    histogram_dist += std::fabs(old_object_shape_features[i] -
      new_object_shape_features[i]);
  }

  return histogram_dist;
}

}  // namespace perception
}  // namespace apollo
