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
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_object_distance.h"
#include <algorithm>
#include <vector>
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "track_object_distance.h"

namespace apollo {
namespace perception {
namespace lidar {

bool TrackObjectDistance::set_weight_location_dist(float weight_location_dist) {
  // Set weight of location dist for all the track object distance objects
  CHECK_GE(weight_location_dist, 0);
  weight_location_dist_ = weight_location_dist;
  return true;
}

bool TrackObjectDistance::set_weight_direction_dist(
    float weight_direction_dist) {
  // Set weight of direction dist for all the track object distance objects
  CHECK_GE(weight_direction_dist, 0);
  weight_direction_dist_ = weight_direction_dist;
  return true;
}

bool TrackObjectDistance::set_weight_bbox_size_dist(
    float weight_bbox_size_dist) {
  // Set weight of bbox size dist for all the track object distance objects
  CHECK_GE(weight_bbox_size_dist, 0);
  weight_bbox_size_dist_ = weight_bbox_size_dist;
  return true;
}

bool TrackObjectDistance::set_weight_point_num_dist(
    float weight_point_num_dist) {
  // Set weight of point num dist for all the track object distance objects
  CHECK_GE(weight_point_num_dist, 0);
  weight_point_num_dist_ = weight_point_num_dist;
  return true;
}

bool TrackObjectDistance::set_weight_histogram_dist(
    float weight_histogram_dist) {
  // Set weight of histogram dist for all the track objects distance objects
  CHECK_GE(weight_histogram_dist, 0);
  weight_histogram_dist_ = weight_histogram_dist;
  return true;
}

bool TrackObjectDistance::set_weight_centroid_shift_dist(
    float weight_centroid_shift_dist) {
  // Set weight of centroid shift for all the track objects distance objects
  CHECK_GE(weight_centroid_shift_dist, 0);
  weight_centroid_shift_dist_ = weight_centroid_shift_dist;
  return true;
}

bool TrackObjectDistance::set_weight_bbox_iou_dist(float weight_bbox_iou_dist) {
  // Set weight of bbox iou for all the track objects distance objects
  CHECK_GE(weight_bbox_iou_dist, 0);
  weight_bbox_iou_dist_ = weight_bbox_iou_dist;
  return true;
}

bool TrackObjectDistance::set_background_object_match_threshold(
    float background_object_match_threshold) {
  // Set weight of bbox iou for all the track objects distance objects
  CHECK_GE(background_object_match_threshold, 0);
  background_object_match_threshold_ = background_object_match_threshold;
  return true;
}

float TrackObjectDistance::ComputeDistance(const TrackDataPtr &track,
                                           const Eigen::VectorXf &track_predict,
                                           const TrackedObjectPtr &new_object,
                                           const double time_diff) {
  float location_dist = 0.0f;
  float direction_dist = 0.0f;
  float bbox_size_dist = 0.0f;
  float point_num_dist = 0.0f;
  float histogram_dist = 0.0f;
  float centroid_shift_dist = 0.0f;
  float bbox_iou_dist = 0.0f;

  float delta = 1e-10;
  // Compute distance for given trakc & object
  if (weight_location_dist_ > delta) {
    location_dist =
        ComputeLocationDistance(track, track_predict, new_object, time_diff);
  }
  if (weight_direction_dist_ > delta) {
    direction_dist =
        ComputeDirectionDistance(track, track_predict, new_object, time_diff);
  }
  if (weight_bbox_size_dist_ > delta) {
    bbox_size_dist =
        ComputeBboxSizeDistance(track, track_predict, new_object, time_diff);
  }
  if (weight_point_num_dist_ > delta) {
    point_num_dist =
        ComputePointNumDistance(track, track_predict, new_object, time_diff);
  }
  if (weight_histogram_dist_ > delta) {
    histogram_dist =
        ComputeHistogramDistance(track, track_predict, new_object, time_diff);
  }
  if (weight_centroid_shift_dist_ > delta) {
    centroid_shift_dist = ComputeCentroidShiftDistance(track, track_predict,
                                                       new_object, time_diff);
  }
  if (weight_bbox_iou_dist_ > delta) {
    bbox_iou_dist =
        ComputeBboxIouDistance(track, track_predict, new_object, time_diff);
  }

  float result_dist = weight_location_dist_ * location_dist +
                      weight_direction_dist_ * direction_dist +
                      weight_bbox_size_dist_ * bbox_size_dist +
                      weight_point_num_dist_ * point_num_dist +
                      weight_histogram_dist_ * histogram_dist +
                      weight_centroid_shift_dist_ * centroid_shift_dist +
                      weight_bbox_iou_dist_ * bbox_iou_dist;
  return result_dist;
}

float TrackObjectDistance::ComputeLocationDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute locatin distance for given track & object
  // range from 0 to positive infinity
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  const TrackedObjectPtr &last_object = latest_obj_pair.second;

  Eigen::Vector3f measured_anchor_point =
      (new_object->anchor_point).cast<float>();
  Eigen::Vector3f predicted_anchor_point = track_predict.head(3);
  Eigen::Vector3f measure_predict_diff =
      measured_anchor_point - predicted_anchor_point;

  float location_dist = sqrt(
      (measure_predict_diff.head(2).cwiseProduct(measure_predict_diff.head(2)))
          .sum());

  /* NEED TO NOTICE: All the states would be collected mainly based on
   * states of tracked objects. Thus, update tracked object when you
   * update the state of track !!!!! */
  Eigen::Vector2d ref_dir = last_object->output_velocity.head(2);
  double speed = ref_dir.norm();
  ref_dir /= speed;

  /* Let location distance generated from a normal distribution with
   * symmetric variance. Modify its variance when speed greater than
   * a threshold. Penalize variance other than motion direction. */
  if (speed > 2) {
    Eigen::Vector2d ref_o_dir = Eigen::Vector2d(ref_dir[1], -ref_dir[0]);
    double dx = ref_dir(0) * measure_predict_diff(0) +
                ref_dir(1) * measure_predict_diff(1);
    double dy = ref_o_dir(0) * measure_predict_diff(0) +
                ref_o_dir(1) * measure_predict_diff(1);
    location_dist = sqrt(dx * dx * 0.5 + dy * dy * 2);
  }

  return location_dist;
}

float TrackObjectDistance::ComputeDirectionDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute direction distance for given track & object
  // range from 0 to 2
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  const TrackedObjectPtr &last_object = latest_obj_pair.second;

  Eigen::Vector3f old_anchor_point =
      (last_object->belief_anchor_point).cast<float>();
  Eigen::Vector3f new_anchor_point = (new_object->anchor_point).cast<float>();
  Eigen::Vector3f anchor_point_shift_dir = new_anchor_point - old_anchor_point;
  anchor_point_shift_dir[2] = 0.0;

  Eigen::Vector3f track_motion_dir = track_predict.head(6).tail(3);
  track_motion_dir[2] = 0.0;

  double cos_theta = 0.994;  // average cos
  if (!track_motion_dir.head(2).isZero() &&
      !anchor_point_shift_dir.head(2).isZero()) {
    // cos_theta = vector_cos_theta_2d_xy(track_motion_dir,
    //                                   anchor_point_shift_dir);
    cos_theta = common::CalculateCosTheta2DXY<float>(track_motion_dir,
                                                     anchor_point_shift_dir);
  }
  float direction_dist = -cos_theta + 1.0;

  return direction_dist;
}

float TrackObjectDistance::ComputeBboxSizeDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute bbox size distance for given track & object
  // range from 0 to 1
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  const TrackedObjectPtr &last_object = latest_obj_pair.second;

  Eigen::Vector3f old_bbox_dir = last_object->output_direction.cast<float>();
  Eigen::Vector3f new_bbox_dir = new_object->direction.cast<float>();
  Eigen::Vector3f old_bbox_size = last_object->output_size.cast<float>();
  Eigen::Vector3f new_bbox_size = new_object->size.cast<float>();

  float size_dist = 0.0;
  double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) +
                           old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) -
                           old_bbox_dir(1) * new_bbox_dir(0));
  float temp_val_0 = 0.0;
  float temp_val_1 = 0.0;
  if (dot_val_00 > dot_val_01) {
    temp_val_0 = fabs(old_bbox_size(0) - new_bbox_size(0)) /
                 std::max(old_bbox_size(0), new_bbox_size(0));
    temp_val_1 = fabs(old_bbox_size(1) - new_bbox_size(1)) /
                 std::max(old_bbox_size(1), new_bbox_size(1));
    size_dist = std::min(temp_val_0, temp_val_1);
  } else {
    temp_val_0 = fabs(old_bbox_size(0) - new_bbox_size(1)) /
                 std::max(old_bbox_size(0), new_bbox_size(1));
    temp_val_1 = fabs(old_bbox_size(1) - new_bbox_size(0)) /
                 std::max(old_bbox_size(1), new_bbox_size(0));
    size_dist = std::min(temp_val_0, temp_val_1);
  }
  // LOG_INFO << "size distance value "  << old_bbox_size(0) << " " <<
  // new_bbox_size(0)
  //                                   << " " << dot_val_00 << " " << dot_val_01
  //                                   << " " << temp_val_0 << " " << temp_val_1
  //                                   << " " << size_dist;

  return size_dist;
}

float TrackObjectDistance::ComputePointNumDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute point num distance for given track & object
  // range from 0 and 1
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  const TrackedObjectPtr &last_object = latest_obj_pair.second;

  // int old_point_number = last_object->object_ptr->cloud->size();
  int old_point_number =
      (last_object->object_ptr->lidar_supplement).cloud_world.size();
  int new_point_number =
      (new_object->object_ptr->lidar_supplement).cloud_world.size();

  float point_num_dist = fabs(old_point_number - new_point_number) * 1.0f /
                         std::max(old_point_number, new_point_number);

  return point_num_dist;
}

float TrackObjectDistance::ComputeHistogramDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Compute histogram distance for given track & object
  // range from 0 to 3
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  const TrackedObjectPtr &last_object = latest_obj_pair.second;

  std::vector<float> &old_object_shape_features = last_object->shape_features;
  std::vector<float> &new_object_shape_features = new_object->shape_features;

  if (old_object_shape_features.size() != new_object_shape_features.size()) {
    LOG_ERROR << "sizes of compared features not matched. TrackObjectDistance";
    return 100;
  }

  float histogram_dist = 0.0;
  for (int i = 0; i < old_object_shape_features.size(); ++i) {
    histogram_dist +=
        std::fabs(old_object_shape_features[i] - new_object_shape_features[i]);
  }

  return histogram_dist;
}

float TrackObjectDistance::ComputeCentroidShiftDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  TrackedObjectPtr last_obj = latest_obj_pair.second;
  float dist =
      (last_obj->barycenter.head(2) - new_object->barycenter.head(2)).norm();
  return dist;
}

float TrackObjectDistance::ComputeBboxIouDistance(
    const TrackDataPtr &track, const Eigen::VectorXf &track_predict,
    const TrackedObjectPtr &new_object, const double time_diff) {
  // Step 1: unify bbox direction, change the one with less pts,
  // for efficiency.
  std::pair<double, TrackedObjectPtr> latest_obj_pair =
      track->GetLatestObject();
  TrackedObjectPtr last_obj = latest_obj_pair.second;
  Eigen::Vector3f old_dir = last_obj->output_direction.cast<float>();
  Eigen::Vector3f old_size = last_obj->output_size.cast<float>();
  Eigen::Vector3d old_center = last_obj->output_center;
  Eigen::Vector3f new_dir = new_object->direction.cast<float>();
  Eigen::Vector3f new_size = new_object->size.cast<float>();
  Eigen::Vector3d new_center = new_object->center;
  old_dir.normalize();
  new_dir.normalize();
  // handle randomness
  old_size(0) = old_size(0) > 0.3 ? old_size(0) : 0.3;
  old_size(1) = old_size(1) > 0.3 ? old_size(1) : 0.3;
  new_size(0) = new_size(0) > 0.3 ? new_size(0) : 0.3;
  new_size(1) = new_size(1) > 0.3 ? new_size(1) : 0.3;
  int last_obj_num_pts =
      (last_obj->object_ptr->lidar_supplement).cloud_world.size();
  int cur_obj_num_pts =
      (new_object->object_ptr->lidar_supplement).cloud_world.size();
  bool change_cur_obj_bbox = last_obj_num_pts > cur_obj_num_pts ? true : false;
  float minimum_edge_length = 0.01;
  base::PointDCloud &cloud =
      (new_object->object_ptr->lidar_supplement).cloud_world;
  if (change_cur_obj_bbox) {
    new_dir = old_dir;
    //    compute_bbox_size_center_xy<pcl_util::Point>(cur_obj->object_ptr->cloud,
    //                                                 new_dir,
    //                                                 new_size,
    //                                                 new_center,
    //                                                 minimum_edge_length);
    common::CalculateBBoxSizeCenter2DXY(cloud, new_dir, &new_size, &new_center,
                                        minimum_edge_length);
  } else {
    old_dir = new_dir;
    //    compute_bbox_size_center_xy<pcl_util::Point>(last_obj->object_ptr->cloud,
    //                                                 old_dir,
    //                                                 old_size,
    //                                                 old_center,
    //                                                 minimum_edge_length);
    common::CalculateBBoxSizeCenter2DXY(cloud, old_dir, &old_size, &old_center,
                                        minimum_edge_length);
  }
  // Step 2: compute 2d iou bbox to bbox
  Eigen::Matrix2d trans_mat;
  trans_mat(0, 0) = (old_dir.cast<double>())(0);
  trans_mat(0, 1) = (old_dir.cast<double>())(1);
  trans_mat(1, 0) = -(old_dir.cast<double>())(1);
  trans_mat(1, 1) = (old_dir.cast<double>())(0);
  Eigen::Vector2d old_center_transformed_2d = trans_mat * old_center.head(2);
  Eigen::Vector2d new_center_transformed_2d = trans_mat * new_center.head(2);
  old_center(0) = old_center_transformed_2d(0);
  old_center(1) = old_center_transformed_2d(1);
  new_center(0) = new_center_transformed_2d(0);
  new_center(1) = new_center_transformed_2d(1);
  //  double iou = compute_2d_iou_bbox_to_bbox(old_center,
  //  //                                           old_size,
  //  //                                           new_center,
  //  //                                           new_size);
  Eigen::Vector3d old_size_tmp = old_size.cast<double>();
  Eigen::Vector3d new_size_tmp = new_size.cast<double>();
  double iou = common::CalculateIou2DXY<double>(old_center, old_size_tmp,
                                                new_center, new_size_tmp);
  // Step 4: compute dist
  double dist = (1 - iou) * background_object_match_threshold_;
  return dist;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
