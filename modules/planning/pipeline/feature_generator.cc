/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/pipeline/feature_generator.h"

#include <cmath>
#include <string>

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store instance data");

DEFINE_int32(
    instance_label_sample_interval, 100,
    "total number of localization msgs to generate one instance label data.");

DEFINE_int32(instance_num_per_file, 100,
             "number of instance to write out in one data file.");

DEFINE_int32(
    localization_sample_interval_for_trajectory_point, 10,
    "number of localization msgs to generate one trajectory point in label.");

DEFINE_int32(localization_move_window_step, 5,
             "number of localization msgs to skip after generating one label "
             "trajectory point.");

DEFINE_bool(enable_binary_instance, true,
            "True to generate protobuf binary data file.");

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::localization::LocalizationEstimate;

void FeatureGenerator::Init() { instance_ = instances_.add_instances(); }

void FeatureGenerator::WriteOutInstances(const Instances& instances,
                                         const std::string& file_name) {
  if (FLAGS_enable_binary_instance) {
    cyber::common::SetProtoToBinaryFile(instances, file_name);
  } else {
    cyber::common::SetProtoToASCIIFile(instances, file_name);
  }
}

void FeatureGenerator::Close() {
  const std::string file_name = absl::StrCat(
      FLAGS_planning_data_dir, "/instances.", instance_file_index_, ".bin");
  total_instance_num_ += instances_.instances_size();
  WriteOutInstances(instances_, file_name);
  ++instance_file_index_;
  AINFO << "Total instance number:" << total_instance_num_;
}

void FeatureGenerator::GenerateTrajectoryLabel(
    const std::list<apollo::localization::LocalizationEstimate>&
        localization_for_label,
    Instance* instance) {
  int i = -1;
  for (const auto& le : localization_for_label) {
    ++i;
    if ((i % FLAGS_localization_sample_interval_for_trajectory_point) != 0) {
      continue;
    }
    auto trajectory_point = instance->add_label_trajectory_points();
    auto& pose = le.pose();
    trajectory_point->mutable_path_point()->set_x(pose.position().x());
    trajectory_point->mutable_path_point()->set_y(pose.position().y());
    trajectory_point->mutable_path_point()->set_z(pose.position().z());
    trajectory_point->mutable_path_point()->set_theta(pose.heading());
    auto v = std::sqrt(pose.linear_velocity().x() * pose.linear_velocity().x() +
                       pose.linear_velocity().y() * pose.linear_velocity().y());
    trajectory_point->set_v(v);
    auto a = std::sqrt(
        pose.linear_acceleration().x() * pose.linear_acceleration().x() +
        pose.linear_acceleration().y() * pose.linear_acceleration().y());
    trajectory_point->set_a(a);
  }
}

void FeatureGenerator::OnLocalization(
    const apollo::localization::LocalizationEstimate& le) {
  if (instance_ == nullptr) {
    AERROR << "instance pointer is nullptr";
    return;
  }

  auto features = instance_->mutable_localization_feature();
  const auto& pose = le.pose();
  features->mutable_position()->CopyFrom(pose.position());
  features->set_heading(pose.heading());
  features->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  features->mutable_linear_acceleration()->CopyFrom(pose.linear_acceleration());
  features->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());
  localization_for_label_.push_back(le);

  if (static_cast<int>(localization_for_label_.size()) >=
      FLAGS_instance_label_sample_interval) {
    GenerateTrajectoryLabel(localization_for_label_, instance_);
    instance_ = instances_.add_instances();
    for (int i = 0; i < FLAGS_localization_move_window_step; ++i) {
      localization_for_label_.pop_front();
    }
  }

  if (instances_.instances_size() >= FLAGS_instance_num_per_file) {
    const std::string file_name = absl::StrCat(
        FLAGS_planning_data_dir, "/instances.", instance_file_index_, ".bin");
    WriteOutInstances(instances_, file_name);
    total_instance_num_ += instances_.instances_size();
    instances_.Clear();
    ++instance_file_index_;
    instance_ = instances_.add_instances();
  }
}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {
  if (instance_ == nullptr) {
    AERROR << "instance pointer is nullptr";
    return;
  }
  auto features = instance_->mutable_chassis_feature();
  features->set_speed_mps(chassis.speed_mps());
  features->set_throttle_percentage(chassis.throttle_percentage());
  features->set_brake_percentage(chassis.brake_percentage());
  features->set_steering_percentage(chassis.steering_percentage());
  features->set_gear_location(chassis.gear_location());
}

void FeatureGenerator::ProcessOfflineData(const std::string& record_filename) {
  RecordReader reader(record_filename);
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_filename;
    return;
  }

  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_localization_topic) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name == FLAGS_chassis_topic) {
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
