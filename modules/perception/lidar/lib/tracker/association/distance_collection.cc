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
#include <algorithm>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lidar/lib/tracker/association/distance_collection.h"

namespace apollo {
namespace perception {
namespace lidar {

float LocationDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff) {
  // Compute locatin distance for last object and new object
  // range from 0 to positive infinity

  Eigen::Vector3f measured_anchor_point =
      (new_object->anchor_point).cast<float>();
  Eigen::Vector3f predicted_anchor_point = track_predict.head(3);
  Eigen::Vector3f measure_predict_diff =
      measured_anchor_point - predicted_anchor_point;

  float location_dist = static_cast<float>(sqrt(
      (measure_predict_diff.head(2).cwiseProduct(measure_predict_diff.head(2)))
          .sum()));

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
    location_dist = static_cast<float>(sqrt(dx * dx * 0.5 + dy * dy * 2));
  }

  return location_dist;
}

float DirectionDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff) {
  // Compute direction distance for last object and new object
  // range from 0 to 2

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
  float direction_dist = static_cast<float>(-cos_theta) + 1.0f;

  return direction_dist;
}

float BboxSizeDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff) {
  // Compute bbox size distance for last object and new object
  // range from 0 to 1

  Eigen::Vector3f old_bbox_dir = last_object->output_direction.cast<float>();
  Eigen::Vector3f new_bbox_dir = new_object->direction.cast<float>();
  Eigen::Vector3f old_bbox_size = last_object->output_size.cast<float>();
  Eigen::Vector3f new_bbox_size = new_object->size.cast<float>();

  float size_dist = 0.0f;
  double dot_val_00 = fabs(old_bbox_dir(0) * new_bbox_dir(0) +
                           old_bbox_dir(1) * new_bbox_dir(1));
  double dot_val_01 = fabs(old_bbox_dir(0) * new_bbox_dir(1) -
                           old_bbox_dir(1) * new_bbox_dir(0));
  float temp_val_0 = 0.0f;
  float temp_val_1 = 0.0f;
  if (dot_val_00 > dot_val_01) {
    temp_val_0 = static_cast<float>(fabs(old_bbox_size(0) - new_bbox_size(0))) /
                 std::max(old_bbox_size(0), new_bbox_size(0));
    temp_val_1 = static_cast<float>(fabs(old_bbox_size(1) - new_bbox_size(1))) /
                 std::max(old_bbox_size(1), new_bbox_size(1));
    size_dist = std::min(temp_val_0, temp_val_1);
  } else {
    temp_val_0 = static_cast<float>(fabs(old_bbox_size(0) - new_bbox_size(1))) /
                 std::max(old_bbox_size(0), new_bbox_size(1));
    temp_val_1 = static_cast<float>(fabs(old_bbox_size(1) - new_bbox_size(0))) /
                 std::max(old_bbox_size(1), new_bbox_size(0));
    size_dist = std::min(temp_val_0, temp_val_1);
  }

  return size_dist;
}

float PointNumDistance(const TrackedObjectConstPtr& last_object,
                       const Eigen::VectorXf& track_predict,
                       const TrackedObjectConstPtr& new_object,
                       const double time_diff) {
  // Compute point num distance for last object and new object
  // range from 0 and 1

  int old_point_number = static_cast<int>(
      (last_object->object_ptr->lidar_supplement).cloud_world.size());
  int new_point_number = static_cast<int>(
      (new_object->object_ptr->lidar_supplement).cloud_world.size());

  float point_num_dist =
      static_cast<float>(fabs(old_point_number - new_point_number)) * 1.0f /
      static_cast<float>(std::max(old_point_number, new_point_number));

  return point_num_dist;
}

float HistogramDistance(const TrackedObjectConstPtr& last_object,
                        const Eigen::VectorXf& track_predict,
                        const TrackedObjectConstPtr& new_object,
                        const double time_diff) {
  // Compute histogram distance for last object and new object
  // range from 0 to 3

  const std::vector<float>& old_object_shape_features =
      last_object->shape_features;
  const std::vector<float>& new_object_shape_features =
      new_object->shape_features;

  if (old_object_shape_features.size() != new_object_shape_features.size()) {
    AINFO << "sizes of compared features not matched. TrackObjectDistance";
    return 100;
  }

  float histogram_dist = 0.0f;
  for (size_t i = 0; i < old_object_shape_features.size(); ++i) {
    histogram_dist +=
        std::fabs(old_object_shape_features[i] - new_object_shape_features[i]);
  }

  return histogram_dist;
}

float CentroidShiftDistance(const TrackedObjectConstPtr& last_object,
                            const Eigen::VectorXf& track_predict,
                            const TrackedObjectConstPtr& new_object,
                            const double time_diff) {
  float dist = static_cast<float>(
      (last_object->barycenter.head(2) - new_object->barycenter.head(2))
          .norm());
  return dist;
}

float BboxIouDistance(const TrackedObjectConstPtr& last_object,
                      const Eigen::VectorXf& track_predict,
                      const TrackedObjectConstPtr& new_object,
                      const double time_diff, double match_threshold) {
  // Step 1: unify bbox direction, change the one with less pts,
  // for efficiency.
  Eigen::Vector3f old_dir = last_object->output_direction.cast<float>();
  Eigen::Vector3f old_size = last_object->output_size.cast<float>();
  Eigen::Vector3d old_center = last_object->output_center;
  Eigen::Vector3f new_dir = new_object->direction.cast<float>();
  Eigen::Vector3f new_size = new_object->size.cast<float>();
  Eigen::Vector3d new_center = new_object->center;
  old_dir.normalize();
  new_dir.normalize();
  // handle randomness
  old_size(0) = old_size(0) > 0.3f ? old_size(0) : 0.3f;
  old_size(1) = old_size(1) > 0.3f ? old_size(1) : 0.3f;
  new_size(0) = new_size(0) > 0.3f ? new_size(0) : 0.3f;
  new_size(1) = new_size(1) > 0.3f ? new_size(1) : 0.3f;
  int last_object_num_pts = static_cast<int>(
      (last_object->object_ptr->lidar_supplement).cloud_world.size());
  int cur_obj_num_pts = static_cast<int>(
      (new_object->object_ptr->lidar_supplement).cloud_world.size());
  bool change_cur_obj_bbox = last_object_num_pts > cur_obj_num_pts;
  float minimum_edge_length = 0.01f;
  base::PointDCloud& cloud =
      (new_object->object_ptr->lidar_supplement).cloud_world;
  if (change_cur_obj_bbox) {
    new_dir = old_dir;
    common::CalculateBBoxSizeCenter2DXY(cloud, new_dir, &new_size, &new_center,
                                        minimum_edge_length);
  } else {
    old_dir = new_dir;
    common::CalculateBBoxSizeCenter2DXY(cloud, old_dir, &old_size, &old_center,
                                        minimum_edge_length);
  }
  // Step 2: compute 2d iou bbox to bbox
  Eigen::Matrix2d trans_mat;
  trans_mat(0, 0) = (old_dir.cast<double>())(0);
  trans_mat(0, 1) = (old_dir.cast<double>())(1);
  trans_mat(1, 0) = -(old_dir.cast<double>())(1);
  trans_mat(1, 1) = (old_dir.cast<double>())(0);
  Eigen::Vector2d old_center_transformed_2d =
      static_cast<Eigen::Matrix<double, 2, 1, 0, 2, 1>>(trans_mat *
                                                        old_center.head(2));
  Eigen::Vector2d new_center_transformed_2d =
      static_cast<Eigen::Matrix<double, 2, 1, 0, 2, 1>>(trans_mat *
                                                        new_center.head(2));
  old_center(0) = old_center_transformed_2d(0);
  old_center(1) = old_center_transformed_2d(1);
  new_center(0) = new_center_transformed_2d(0);
  new_center(1) = new_center_transformed_2d(1);
  Eigen::Vector3d old_size_tmp = old_size.cast<double>();
  Eigen::Vector3d new_size_tmp = new_size.cast<double>();
  double iou = common::CalculateIou2DXY<double>(old_center, old_size_tmp,
                                                new_center, new_size_tmp);
  // Step 4: compute dist
  double dist = (1 - iou) * match_threshold;
  return static_cast<float>(dist);
}

float SemanticMapDistance(const MlfTrackData& track_dat,
                          const TrackedObjectConstPtr& cur_obj) {
  if (track_dat.feature_ &&
      track_dat.feature_->predicted_trajectory_size() > 0) {
    const apollo::prediction::Trajectory& traj =
        track_dat.feature_->predicted_trajectory(0);
    apollo::common::math::Vec2d pt(
        cur_obj->center[0] - cur_obj->global_local_offset[0],
        cur_obj->center[1] - cur_obj->global_local_offset[1]);
    std::vector<float> dist_vec;
    for (int i = 0; i < traj.trajectory_point_size() - 1; ++i) {
      const apollo::common::TrajectoryPoint& pt_st = traj.trajectory_point(i);
      const apollo::common::TrajectoryPoint& pt_et =
          traj.trajectory_point(i + 1);
      apollo::common::math::Vec2d start(pt_st.path_point().x(),
                                        pt_st.path_point().y());
      apollo::common::math::Vec2d end(pt_et.path_point().x(),
                                      pt_et.path_point().y());
      apollo::common::math::LineSegment2d seg(start, end);
      double dist = seg.DistanceTo(pt);
      dist_vec.push_back(static_cast<float>(dist));
    }
    float min_dist = *(std::min_element(dist_vec.begin(), dist_vec.end()));
    ADEBUG << "min_dist calculated for track id " << track_dat.track_id_
          << " object id " << cur_obj->track_id << " distance " << min_dist;
    return min_dist;
  }

  return 0.0f;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
