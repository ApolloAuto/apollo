/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/measurement/measurement_collection.h"

#include "modules/perception/common/algorithm/geometry/basic.h"
#include "modules/perception/common/algorithm/geometry/common.h"

namespace apollo {
namespace perception {
namespace radar4d {

void MeasureAnchorPointVelocity(TrackedObjectPtr new_object,
                                const TrackedObjectConstPtr& old_object,
                                const double& time_diff) {
  // Compute 2D anchor point velocity measurement
  Eigen::Vector3d measured_anchor_point_velocity =
      new_object->anchor_point - old_object->belief_anchor_point;
  measured_anchor_point_velocity /= time_diff;
  measured_anchor_point_velocity(2) = 0.0;
  new_object->measured_barycenter_velocity = measured_anchor_point_velocity;
}

void MeasureBboxCenterVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff) {
  // Compute 2D bbox center velocity measurement
  Eigen::Vector3f old_dir_tmp = old_object->output_direction.cast<float>();
  // Eigen::Vector3d old_size = old_object->output_size;
  Eigen::Vector3d old_center = old_object->output_center;
  Eigen::Vector3f new_size_tmp;
  Eigen::Vector3d new_center;
  float minimum_edge_length = 0.01f;
  base::RadarPointDCloud& cloud =
      (new_object->object_ptr->radar4d_supplement).cloud_world;
  algorithm::CalculateBBoxSizeCenter2DXY(cloud, old_dir_tmp, &new_size_tmp,
                                      &new_center, minimum_edge_length);
  // Eigen::Vector3d old_dir = old_dir_tmp.cast<double>();
  // Eigen::Vector3d new_size = new_size_tmp.cast<double>();
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

void MeasureBboxCornerVelocity(TrackedObjectPtr new_object,
                               const TrackedObjectConstPtr& old_object,
                               const double& time_diff) {
  // Compute 2D bbxo corner velocity measurement
  Eigen::Vector3f old_dir_tmp = old_object->output_direction.cast<float>();
  Eigen::Vector3d old_size = old_object->output_size;
  Eigen::Vector3d old_center = old_object->output_center;
  Eigen::Vector3f new_size_tmp;
  Eigen::Vector3d new_center;
  float minimum_edge_length = 0.01f;
  base::RadarPointDCloud& cloud =
      (new_object->object_ptr->radar4d_supplement).cloud_world;
  algorithm::CalculateBBoxSizeCenter2DXY(cloud, old_dir_tmp, &new_size_tmp,
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
  Eigen::Vector3d ref_location = new_object->sensor_to_local_pose.translation();
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
        algorithm::Calculate2DXYProjectVector<double>(bbox_corner_velocity,
                                                   project_dir);
    // set velocity as 0 when conflict
    if (bbox_corner_velocity_on_project_dir.dot(project_dir) <= 0) {
      bbox_corner_velocity_on_project_dir = Eigen::Vector3d::Zero();
    }
    new_object->measured_corners_velocity[i] =
        bbox_corner_velocity_on_project_dir;
  }
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
