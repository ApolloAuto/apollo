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
#include <utility>

#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/measurement_computer.h"

namespace apollo {
namespace perception {
namespace lidar {

void MeasurementComputer::ComputeMeasurment(const TrackDataPtr &track_data,
                                            TrackedObjectPtr new_object,
                                            const double &new_time_stamp) {
  // use a history window to expand measure compute timediff
  int stable_velocity_start_history_size = 5;
  // window size 1, use latest object as old object, keep same as master
  // other window size need to evaluate
  int velocity_compute_window_size = 0;
  // if motion_state is TRUSTED_MOVE, use single frame measurement
  // to avoid lag of speed variation (especially for vehicle turning)
  if (track_data->motion_state_ == MotionState::TRUSTED_MOVE) {
    velocity_compute_window_size = 1;
  } else {
    velocity_compute_window_size = 5;
  }
  int history_size = track_data->history_objects_.size();
  std::pair<double, TrackedObjectPtr> old_obj_pair;

  if (history_size <= stable_velocity_start_history_size) {
    old_obj_pair = track_data->GetLatestObject();
  } else {
    int old_obj_pos = 0;
    if (history_size >=
        stable_velocity_start_history_size + velocity_compute_window_size) {
      // when velocity_compute_window_size = 5
      // history  -9   -8   -7   -6   -5   -4   -3   -2   -1   0(latest)
      // unstable  *    *    *    *    *
      // window                             *    *    *    *   *
      // old pos                            ^
      // negative means from latest
      old_obj_pos = -(velocity_compute_window_size - 1);
    } else {
      // when velocity_compute_window_size = 5
      // history  -7   -6   -5   -4   -3   -2   -1    0(latest)
      // unstable  *    *    *    *    *
      // window                   *    *    *    *    *
      // old pos                            ^
      // positive means from oldest
      old_obj_pos = stable_velocity_start_history_size + 1;
    }
    old_obj_pair = track_data->GetHistoryObject(old_obj_pos);
  }

  const TrackedObjectPtr &old_object = old_obj_pair.second;
  double time_diff = new_time_stamp - old_obj_pair.first;
  // Compute 2D velocity measurement for filtering
  // Robustly compute measured velocity via observation
  // redundency (evaluate measured velocity
  // from multiple viewpoint, and get the one which is
  // most consistant to motion history)
  // Observation I: anchor point velocity measurement
  ComputeMeasuredAnchorPointVelocity(new_object, old_object, time_diff);
  // Observation II: bbox center velocity measurement
  ComputeMeasuredBboxCenterVelocity(new_object, old_object, time_diff);
  // Observation III: bbox corner velocity measurement
  ComputeMeasuredBboxCornerVelocity(new_object, old_object, time_diff);
}

void MeasurementComputer::ComputeMeasuredAnchorPointVelocity(
    TrackedObjectPtr new_object, const TrackedObjectPtr &old_object,
    const double &time_diff) {
  // Compute 2D anchor point velocity measurement
  Eigen::Vector3d measured_anchor_point_velocity =
      new_object->anchor_point - old_object->belief_anchor_point;
  measured_anchor_point_velocity /= time_diff;
  measured_anchor_point_velocity(2) = 0.0;
  new_object->measured_barycenter_velocity = measured_anchor_point_velocity;
}

void MeasurementComputer::ComputeMeasuredBboxCenterVelocity(
    TrackedObjectPtr new_object, const TrackedObjectPtr &old_object,
    const double &time_diff) {
  // Compute 2D bbox center velocity measurement
  Eigen::Vector3f old_dir_tmp = old_object->output_direction.cast<float>();
  Eigen::Vector3d old_size = old_object->output_size;
  Eigen::Vector3d old_center = old_object->output_center;
  Eigen::Vector3f new_size_tmp;
  Eigen::Vector3d new_center;
  double minimum_edge_length = 0.01;
  base::PointDCloud &cloud =
      (new_object->object_ptr->lidar_supplement).cloud_world;
  common::CalculateBBoxSizeCenter2DXY(cloud, old_dir_tmp, &new_size_tmp,
                                      &new_center, minimum_edge_length);
  Eigen::Vector3d old_dir = old_dir_tmp.cast<double>();
  Eigen::Vector3d new_size = new_size_tmp.cast<double>();
  Eigen::Vector3d measured_bbox_center_velocity_with_old_dir =
      (new_center - old_center);
  measured_bbox_center_velocity_with_old_dir /= time_diff;
  measured_bbox_center_velocity_with_old_dir(2) = 0.0;
  Eigen::Vector3d measured_bbox_center_velocity =
      measured_bbox_center_velocity_with_old_dir;
  Eigen::Vector3d project_dir =
      new_object->anchor_point - old_object->belief_anchor_point;
  if (measured_bbox_center_velocity.dot(project_dir) <= 0) {
    measured_bbox_center_velocity = Eigen::Vector3d::Zero();
  }
  new_object->measured_center_velocity = measured_bbox_center_velocity;
}

void MeasurementComputer::ComputeMeasuredBboxCornerVelocity(
    TrackedObjectPtr new_object, const TrackedObjectPtr &old_object,
    const double &time_diff) {
  // Compute 2D bbxo corner velocity measurement
  Eigen::Vector3f old_dir_tmp = old_object->output_direction.cast<float>();
  Eigen::Vector3d old_size = old_object->output_size;
  Eigen::Vector3d old_center = old_object->output_center;
  Eigen::Vector3f new_size_tmp;
  Eigen::Vector3d new_center;
  double minimum_edge_length = 0.01;
  base::PointDCloud &cloud =
      (new_object->object_ptr->lidar_supplement).cloud_world;
  common::CalculateBBoxSizeCenter2DXY(cloud, old_dir_tmp, &new_size_tmp,
                                      &new_center, minimum_edge_length);
  Eigen::Vector3d old_dir = old_dir_tmp.cast<double>();
  Eigen::Vector3d new_size = new_size_tmp.cast<double>();
  Eigen::Vector3d ortho_old_dir(-old_dir(1), old_dir(0), 0.0);
  Eigen::Vector3d old_bbox_corner_list[4];
  Eigen::Vector3d new_bbox_corner_list[4];
  Eigen::Vector3d old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 +
                                    ortho_old_dir * old_size(1) * 0.5;
  Eigen::Vector3d new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 +
                                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[0] = old_bbox_corner;
  new_bbox_corner_list[0] = new_bbox_corner;
  old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 +
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 +
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[1] = old_bbox_corner;
  new_bbox_corner_list[1] = new_bbox_corner;
  old_bbox_corner = old_center + old_dir * old_size(0) * 0.5 -
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center + old_dir * new_size(0) * 0.5 -
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[2] = old_bbox_corner;
  new_bbox_corner_list[2] = new_bbox_corner;
  old_bbox_corner = old_center - old_dir * old_size(0) * 0.5 -
                    ortho_old_dir * old_size(1) * 0.5;
  new_bbox_corner = new_center - old_dir * new_size(0) * 0.5 -
                    ortho_old_dir * new_size(1) * 0.5;
  old_bbox_corner_list[3] = old_bbox_corner;
  new_bbox_corner_list[3] = new_bbox_corner;

  // calculate the nearest corner
  Eigen::Vector3d ref_location =
      new_object->sensor_to_local_pose.matrix().block<3, 1>(0, 3);
  Eigen::Vector3d nearest_old_bbox_corner = old_bbox_corner_list[0];
  Eigen::Vector3d nearest_new_bbox_corner = new_bbox_corner_list[0];
  double min_center_distance = (ref_location - nearest_new_bbox_corner).norm();
  for (size_t i = 1; i < 4; ++i) {
    double center_distance = (ref_location - new_bbox_corner_list[i]).norm();
    if (center_distance < min_center_distance) {
      min_center_distance = center_distance;
      nearest_old_bbox_corner = old_bbox_corner_list[i];
      nearest_new_bbox_corner = new_bbox_corner_list[i];
    }
  }
  // no projection
  new_object->measured_nearest_corner_velocity =
      (nearest_new_bbox_corner - nearest_old_bbox_corner) / time_diff;

  // select project_dir by change of size
  Eigen::Vector3d direct_old_size = old_object->size;
  Eigen::Vector3d direct_new_size = new_object->size;
  double length_change =
      fabs(direct_old_size(0) - direct_new_size(0)) / direct_old_size(0);
  double width_change =
      fabs(direct_old_size(1) - direct_new_size(1)) / direct_old_size(1);

  const double max_change_thresh = 0.1;
  Eigen::Vector3d project_dir;
  if (length_change < max_change_thresh && width_change < max_change_thresh) {
    project_dir = new_object->center - old_object->center;
  } else {
    project_dir = new_object->anchor_point - old_object->belief_anchor_point;
  }

  for (size_t i = 0; i < 4; ++i) {
    old_bbox_corner = old_bbox_corner_list[i];
    new_bbox_corner = new_bbox_corner_list[i];
    new_object->corners[i] = new_bbox_corner_list[i];
    Eigen::Vector3d bbox_corner_velocity =
        ((new_bbox_corner - old_bbox_corner) / time_diff);
    Eigen::Vector3d bbox_corner_velocity_on_project_dir =
        common::Calculate2DXYProjectVector<double>(bbox_corner_velocity,
                                                   project_dir);
    // set velocity as 0 when conflict
    if (bbox_corner_velocity_on_project_dir.dot(project_dir) <= 0) {
      bbox_corner_velocity_on_project_dir = Eigen::Vector3d::Zero();
    }
    new_object->measured_corners_velocity[i] =
        bbox_corner_velocity_on_project_dir;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
