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
#include <string>
#include <vector>
#include <limits>

#include "modules/perception/lidar_tracking/tracker/measurement/measurement_collection.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_motion_measurement.h"

#include "modules/perception/common/algorithm/geometry/basic.h"

namespace apollo {
namespace perception {
namespace lidar {

const float STATIC_VELOCITY = 0.01;

void MlfMotionMeasurement::ComputeMotionMeasurment(
    const MlfTrackDataConstPtr& track_data, TrackedObjectPtr new_object) {
  // prefer to choose objects from the same sensor
  std::string sensor_name = new_object->sensor_info.name;
  TrackedObjectConstPtr latest_object =
      track_data->GetLatestSensorObject(sensor_name).second;
  if (latest_object == nullptr) {
    latest_object = track_data->GetLatestObject().second;
  }
  if (latest_object.get() == nullptr) {
    AERROR << "latest_object is not available";
    return;
  }
  // should we estimate the measurement if the time diff is too small?
  double latest_time = latest_object->object_ptr->latest_tracked_time;
  double current_time = new_object->object_ptr->latest_tracked_time;
  double time_diff = current_time - latest_time;
  if (fabs(time_diff) < EPSILON_TIME) {
    time_diff = DEFAULT_FPS;
  }
  MeasureAnchorPointVelocity(new_object, latest_object, time_diff);
  MeasureBboxCenterVelocity(new_object, latest_object, time_diff);
  MeasureBboxCornerVelocity(new_object, latest_object, time_diff);
  MeasureBboxCenterHistoryVelocity(track_data, new_object);
  bool spec_condition =
      new_object->object_ptr->sub_type == base::ObjectSubType::BIGMOT;
  if (spec_condition) {
      MeasureBboxCornerHistoryVelocity(track_data, new_object);
      MeasureObjectDetectionHistoryVelocity(track_data, new_object);
      MeasureObjectDetectionVelocity(new_object, latest_object, time_diff);
  }
  MeasurementSelection(track_data, latest_object, new_object, spec_condition);
  MeasurementQualityEstimation(latest_object, new_object);
  MeasurementRefine(track_data, latest_object, new_object, time_diff);
}

void MlfMotionMeasurement::MeasurementSelection(
    const MlfTrackDataConstPtr& track_data,
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object,
    bool condition) {
  // Select measured velocity among candidates according motion consistency
  int64_t corner_index = 0;
  float corner_velocity_gain = 0.0f;
  std::vector<float> corner_velocity_gain_norms(4);
  for (int i = 0; i < 4; ++i) {
    corner_velocity_gain_norms[i] =
        static_cast<float>((new_object->measured_corners_velocity[i] -
                            latest_object->belief_velocity)
                               .norm());
  }
  std::vector<float>::iterator corener_min_gain =
      std::min_element(std::begin(corner_velocity_gain_norms),
                       std::end(corner_velocity_gain_norms));
  corner_velocity_gain = *corener_min_gain;
  corner_index = corener_min_gain - corner_velocity_gain_norms.begin();

  std::vector<float> velocity_gain_norms(4);
  velocity_gain_norms[0] = corner_velocity_gain;
  velocity_gain_norms[1] =
      static_cast<float>((new_object->measured_barycenter_velocity -
                          latest_object->belief_velocity)
                             .norm());
  velocity_gain_norms[2] = static_cast<float>(
      (new_object->measured_center_velocity - latest_object->belief_velocity)
          .norm());
  velocity_gain_norms[3] = static_cast<float>(
      (new_object->measured_history_center_velocity -
          latest_object->belief_velocity).norm());
  std::vector<float>::iterator min_gain = std::min_element(
      std::begin(velocity_gain_norms), std::end(velocity_gain_norms));
  int64_t min_gain_pos = min_gain - velocity_gain_norms.begin();
  if (min_gain_pos == 0) {
    new_object->selected_measured_velocity =
        new_object->measured_corners_velocity[corner_index];
  }
  if (min_gain_pos == 1) {
    new_object->selected_measured_velocity =
        new_object->measured_barycenter_velocity;
  }
  if (min_gain_pos == 2) {
    new_object->selected_measured_velocity =
        new_object->measured_center_velocity;
  }
  if (min_gain_pos == 3) {
    new_object->selected_measured_velocity =
        new_object->measured_history_center_velocity;
  }
  if (condition) {
    // object-detection-corner-history-observation
    std::vector<float> condition_velocity_norm(10);
    for (int i = 0; i < 4; ++i) {
        condition_velocity_norm[i] = static_cast<float>(
            (new_object->measured_detection_history_corners_velocity[i] -
                latest_object->belief_velocity).norm());
    }
    // object-detection-center-observation
    condition_velocity_norm[4] = static_cast<float>(
        (new_object->measured_detection_history_center_velocity -
            latest_object->belief_velocity).norm());
    condition_velocity_norm[5] = static_cast<float>(
        (new_object->measured_detection_center_velocity -
            latest_object->belief_velocity).norm());
    // corners-history-observation
    for (int i = 0; i < 4; ++i) {
        condition_velocity_norm[i + 6] = static_cast<float>(
            (new_object->measured_history_corners_velocity[i] -
                latest_object->belief_velocity).norm());
    }
    std::vector<float>::iterator condition_min_diff =
        std::min_element(std::begin(condition_velocity_norm),
            std::end(condition_velocity_norm));
    int min_pos = condition_min_diff - condition_velocity_norm.begin();
    if (*condition_min_diff < *min_gain) {
        if (min_pos <= 3 && min_pos >= 0) {
            new_object->selected_measured_velocity =
              new_object->measured_detection_history_corners_velocity[min_pos];
        }
        if (min_pos == 4) {
            new_object->selected_measured_velocity =
                new_object->measured_detection_history_center_velocity;
        }
        if (min_pos == 5) {
            new_object->selected_measured_velocity =
                new_object->measured_detection_center_velocity;
        }
        if (min_pos <= 9 && min_pos >= 6) {
            new_object->selected_measured_velocity =
                new_object->measured_history_corners_velocity[min_pos - 6];
        }
    }
  }
  ADEBUG << "[Measurement] track_id: " << track_data->track_id_
    << " belief_velocity: " << latest_object->belief_velocity(0)
    << ", " << latest_object->belief_velocity(1)
    << " corner_velocity[0]: " << new_object->measured_corners_velocity[0](0)
    << ", " << new_object->measured_corners_velocity[0](1)
    << " corner_velocity[1]: " << new_object->measured_corners_velocity[1](0)
    << ", " << new_object->measured_corners_velocity[1](1)
    << " corner_velocity[2]: " << new_object->measured_corners_velocity[2](0)
    << ", " << new_object->measured_corners_velocity[2](1)
    << " corner_velocity[3]: " << new_object->measured_corners_velocity[3](0)
    << ", " << new_object->measured_corners_velocity[3](1)
    << " barycenter_velocity: " << new_object->measured_barycenter_velocity(0)
    << ", " << new_object->measured_barycenter_velocity(1)
    << " center_velocity: " << new_object->measured_center_velocity(0)
    << ", " << new_object->measured_center_velocity(1)
    << " history_velocity: "<< new_object->measured_history_center_velocity(0)
    << ", " << new_object->measured_history_center_velocity(1)
    << " detection_history_corner_velocity[0]: "
    << new_object->measured_detection_history_corners_velocity[0](0) << ", "
    << new_object->measured_detection_history_corners_velocity[0](1)
    << " detection_history_corner_velocity[1]: "
    << new_object->measured_detection_history_corners_velocity[1](0) << ", "
    << new_object->measured_detection_history_corners_velocity[1](1)
    << " detection_history_corner_velocity[2]: "
    << new_object->measured_detection_history_corners_velocity[2](0) << ", "
    << new_object->measured_detection_history_corners_velocity[2](1)
    << " detection_history_corner_velocity[3]: "
    << new_object->measured_detection_history_corners_velocity[3](0) << ", "
    << new_object->measured_detection_history_corners_velocity[3](1)
    << " detection_center_velocity: "
    << new_object->measured_detection_center_velocity(0) << ", "
    << new_object->measured_detection_center_velocity(1)
    << " detection_history_center_velocity: "
    << new_object->measured_detection_history_center_velocity(0) << ", "
    << new_object->measured_detection_history_center_velocity(1)
    << " history_corner_velocity[0]: "
    << new_object->measured_history_corners_velocity[0](0) << ", "
    << new_object->measured_history_corners_velocity[0](1)
    << " history_corner_velocity[1]: "
    << new_object->measured_history_corners_velocity[1](0) << ", "
    << new_object->measured_history_corners_velocity[1](1)
    << " history_corner_velocity[2]: "
    << new_object->measured_history_corners_velocity[2](0) << ", "
    << new_object->measured_history_corners_velocity[2](1)
    << " history_corner_velocity[3]: "
    << new_object->measured_history_corners_velocity[3](0) << ", "
    << new_object->measured_history_corners_velocity[3](1)
    << " selected: " << new_object->selected_measured_velocity(0) << ", "
    << new_object->selected_measured_velocity(1);
}

void MlfMotionMeasurement::MeasurementQualityEstimation(
    const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object) {
  // 1. point size diff (only for same sensor)
  int pre_num = static_cast<int>(
      latest_object->object_ptr->lidar_supplement.cloud_world.size());
  int cur_num = static_cast<int>(
      new_object->object_ptr->lidar_supplement.cloud_world.size());
  double quality_based_on_point_diff_ratio =
      (latest_object->sensor_info.name == new_object->sensor_info.name)
          ? (1.0 - fabs(pre_num - cur_num) / std::max(pre_num, cur_num))
          : 1.0;
  // 2. association quality
  double quality_based_on_association_score =
      pow(1.0 - new_object->association_score, 2.0);
  new_object->update_quality = std::min(quality_based_on_association_score,
                                        quality_based_on_point_diff_ratio);
}

void MlfMotionMeasurement::MeasurementRefine(
        const MlfTrackDataConstPtr& track_data,
        const TrackedObjectConstPtr& latest_object,
        TrackedObjectPtr new_object,
        const double& time_diff) {
    const Eigen::Vector3d cur_vel = new_object->selected_measured_velocity;
    const int get_history_size = 4;
    // get hisroty objects
    std::vector<TrackedObjectConstPtr> history_objects;
    track_data->GetLatestKObjects(get_history_size, &history_objects);
    if (!history_objects.size()) {
        AWARN << "No history objects find. Unreasonable";
        return;
    }

    float velocity_threshold = 2.0;
    velocity_threshold =
        latest_object->type == base::ObjectType::VEHICLE ? 3.0 : 2.0;

    float small_velocity_threshold = 2.0;
    small_velocity_threshold =
        latest_object->type == base::ObjectType::PEDESTRIAN ? 0.1 : 0.5;
    if (cur_vel.head<2>().norm() > velocity_threshold) {
        ++new_object->measured_big_velocity_age;
    }
    if (cur_vel.head<2>().norm() < small_velocity_threshold) {
        new_object->measured_big_velocity_age = 0;
    }

    // Condition 1: size < 3 and latest is static and new velocity is big
    if (history_objects.size() < get_history_size) {
        if (latest_object->output_velocity.head<2>().norm() < STATIC_VELOCITY
            && cur_vel.head<2>().norm() > velocity_threshold) {
            // new_object->selected_measured_velocity =
            //   latest_object->selected_measured_velocity;
            new_object->selected_measured_velocity =
                latest_object->output_velocity;
            AINFO << "[Velocity-Measurement] track_id: "
                  << track_data->track_id_
                  << " history small and current BIG. Keep Lastest Output";
            return;
        } else {
            return;
        }
    }

    // Condition 2: latest-three static (including small velocity)
    // and new_velocity is big
    bool keep_small_velocity = true;
    for (auto& obj : history_objects) {
        if (obj->output_velocity.head<2>().norm() > 0.1) {
            keep_small_velocity = false;
        }
    }
    if (new_object->measured_big_velocity_age >= get_history_size) {
        AINFO << "[Velocity-Measurement] Continuous big velocity. track_id: "
              << track_data->track_id_;
        return;
    }
    if (keep_small_velocity && cur_vel.head<2>().norm() > velocity_threshold) {
        // new_object->selected_measured_velocity =
        //   latest_object->selected_measured_velocity;
        new_object->selected_measured_velocity =
            latest_object->output_velocity;
        AINFO << "[Velocity-Measurement] track_id: " << track_data->track_id_
              << " history static and current BIG. Keep Lastest Output";
        return;
    }

    // Condition 3: average and variance change BIG
    size_t ori_size = history_objects.size();
    size_t new_size = ori_size + 1;
    Eigen::Vector3d old_ave = Eigen::Vector3d::Zero();
    Eigen::Vector3d old_var = Eigen::Vector3d::Zero();
    Eigen::Vector3d new_ave = Eigen::Vector3d::Zero();
    Eigen::Vector3d new_var = Eigen::Vector3d::Zero();
    // Condition 3: Small-Velocity and small-velocity-variance and SUDDENLY BIG
    for (auto& obj : history_objects) {
        old_ave += obj->output_velocity;
    }
    old_ave /= ori_size;
    for (auto& obj : history_objects) {
        old_var(0) += ((obj->output_velocity(0) - old_ave(0)) *
            (obj->output_velocity(0) - old_ave(0)));
        old_var(1) += ((obj->output_velocity(1) - old_ave(1)) *
            (obj->output_velocity(1) - old_ave(1)));
    }
    old_var /= ori_size;

    // old and cur to get new
    new_ave = old_ave + (cur_vel - old_ave) * 1.0 / ori_size;
    new_var(0) = (cur_vel(0) - old_ave(0)) * (cur_vel(0) - old_ave(0)) * 1.0 *
        ori_size / (new_size * new_size)
          + old_var(0) * 1.0 * ori_size / new_size;
    new_var(1) = (cur_vel(1) - old_ave(1)) * (cur_vel(1) - old_ave(1)) * 1.0 *
        ori_size / (new_size * new_size)
            + old_var(1) * 1.0 * ori_size / new_size;

    float ave_change = fabs(new_ave.head<2>().norm() -
        old_ave.head<2>().norm());
    float var_change = fabs(new_var.head<2>().norm() -
        old_var.head<2>().norm());
    if (ave_change > 2.0 && var_change > 2.0) {
        // new_object->selected_measured_velocity =
        //     latest_object->selected_measured_velocity;
        new_object->selected_measured_velocity =
            latest_object->output_velocity;
        AINFO << "[Velocity-Measurement] track_id: " << track_data->track_id_
              << " old_ave: " << old_ave(0) << ", " << old_ave(1)
              << " old_var: " << old_var(0) << ", " << old_var(1)
              << " new_ave: " << new_ave(0) << ", " << new_ave(1)
              << " new_var: " << new_var(0) << ", " << new_var(1)
              << " Change Bigger. Keep Lastest Output";
        return;
    }

    // Condition 4: disverge bigger (Vt = V0 + a * delta_T)
    double cur_time = new_object->object_ptr->latest_tracked_time;
    Eigen::Vector2d expected_velocity =
        ComputeExpectedVelocity(history_objects, cur_time);
    bool ratio_flag = cur_vel.head<2>().norm() / expected_velocity.norm() > 20;
    if (expected_velocity.norm() > STATIC_VELOCITY && ratio_flag) {
        // new_object->selected_measured_velocity =
        //    latest_object->selected_measured_velocity;
        new_object->selected_measured_velocity =
            latest_object->output_velocity;
        AINFO << "[Velocity-Measurement] track_id: " << track_data->track_id_
              << " Expected velocity CHANGE big. Keep Lastest Output";
        return;
    }

    // Condition 5: direction diff BIG [Not Use]
    // Eigen::Vector3f cur_dir = new_object->object_ptr->direction;
    // cur_dir.normalize();
    // Eigen::Vector3f pre_dir = latest_object->object_ptr->direction;
    // pre_dir.normalize();
    // float theta_change = algorithm::CalculateTheta2DXY(pre_dir, cur_dir);
    // // if (fabs(theta_change) > M_PI / 4) {
    // //     new_object->selected_measured_velocity =
    // //        latest_object->selected_measured_velocity;
    // //     sstr << " direction CHANGE BIG. KEEP LATEST MEASURE";
    // //     ADEBUG << sstr.str();
    // //     return;
    // // }
}

Eigen::Vector2d MlfMotionMeasurement::ComputeExpectedVelocity(
        std::vector<TrackedObjectConstPtr> history_objects,
        double cur_time) {
    if (history_objects.size() == 1) {
        return Eigen::Vector2d(0, 0);
    }
    size_t valid = 0;
    double velocity_sum_x = 0.0, velocity_sum_y = 0.0;
    for (size_t i = 0; i < history_objects.size(); i++) {
        auto after_obj = history_objects[i];
        for (size_t j = i + 1; j < history_objects.size(); j++) {
            // index protect
            if (i == j || j >= history_objects.size()) {
                continue;
            }
            auto pre_obj = history_objects[j];
            if (after_obj->output_velocity.head<2>().norm() < STATIC_VELOCITY
              || pre_obj->output_velocity.head<2>().norm() < STATIC_VELOCITY) {
                continue;
            }
            valid++;
            double time_ratio = after_obj->object_ptr->latest_tracked_time -
                pre_obj->object_ptr->latest_tracked_time;
            double ratio_x = (after_obj->output_velocity(0) -
                pre_obj->output_velocity(0)) / (time_ratio);
            double ratio_y = (after_obj->output_velocity(1) -
                pre_obj->output_velocity(1)) / (time_ratio);

            double expected_time_diff = cur_time -
                after_obj->object_ptr->latest_tracked_time;
            velocity_sum_x += ratio_x * expected_time_diff +
                after_obj->output_velocity(0);
            velocity_sum_y += ratio_y * expected_time_diff +
                after_obj->output_velocity(1);
        }
    }
    if (valid == 0) {
        return Eigen::Vector2d(0, 0);
    }
    velocity_sum_x /= valid;
    velocity_sum_y /= valid;
    return Eigen::Vector2d(velocity_sum_x, velocity_sum_y);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
