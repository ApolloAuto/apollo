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
#include "modules/planning/common/planning_gflags.h"

DEFINE_string(planning_data_dir, "/apollo/modules/planning/data/",
              "Prefix of files to store learning_data_frame data");
DEFINE_int32(localization_freq, 100, "frequence of localization message");
DEFINE_int32(planning_freq, 10, "frequence of planning message");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_bool(enable_binary_learning_data, true,
            "True to generate protobuf binary data file.");

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::record::RecordMessage;
using apollo::cyber::record::RecordReader;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingResponse;

void FeatureGenerator::Init() {
  learning_data_frame_ = learning_data_.add_learning_data();
}

void FeatureGenerator::WriteOutLearningData(const LearningData& learning_data,
                                            const std::string& file_name) {
  if (FLAGS_enable_binary_learning_data) {
    cyber::common::SetProtoToBinaryFile(learning_data, file_name);
    cyber::common::SetProtoToASCIIFile(learning_data, file_name + ".txt");
  } else {
    cyber::common::SetProtoToASCIIFile(learning_data, file_name);
  }
  learning_data_.Clear();
  ++learning_data_file_index_;
}

void FeatureGenerator::Close() {
  const std::string file_name = absl::StrCat(
      FLAGS_planning_data_dir, "/learning_data.",
      learning_data_file_index_, ".bin");
  WriteOutLearningData(learning_data_, file_name);
  AINFO << "Total learning_data_frame number:"
        << total_learning_data_frame_num_;
}

void FeatureGenerator::GenerateTrajectoryLabel(
    const std::list<apollo::localization::LocalizationEstimate>&
        localization_for_label,
    LearningDataFrame* learning_data_frame) {
  learning_data_frame->set_timestamp_sec(
      localization_for_label.back().header().timestamp_sec());
  learning_data_frame->set_frame_num(total_learning_data_frame_num_++);

  // add routing
  auto features = learning_data_frame->mutable_routing_response();
  features->Clear();
  for (const auto& lane_id : routing_lane_ids) {
    features->add_lane_id(lane_id);
  }

  int i = -1;
  int cnt = 0;

  const int localization_sample_interval_for_trajectory_point =
      FLAGS_localization_freq / FLAGS_planning_freq;
  for (const auto& le : localization_for_label) {
    ++i;
    if ((i % localization_sample_interval_for_trajectory_point) != 0) {
      continue;
    }
    auto trajectory_point = learning_data_frame->add_trajectory_point();
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

    cnt++;
  }
  // AINFO << "number of trajectory points in one frame: " << cnt;
}

void FeatureGenerator::OnLocalization(
    const apollo::localization::LocalizationEstimate& le) {
  if (learning_data_frame_ == nullptr) {
    AERROR << "learning_data_frame_ pointer is nullptr";
    return;
  }

  auto features = learning_data_frame_->mutable_localization();
  const auto& pose = le.pose();
  features->mutable_position()->CopyFrom(pose.position());
  features->set_heading(pose.heading());
  features->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  features->mutable_linear_acceleration()->CopyFrom(pose.linear_acceleration());
  features->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());
  localization_for_label_.push_back(le);

  const int localization_msg_start_cnt =
      FLAGS_localization_freq * FLAGS_trajectory_time_length;
  if (static_cast<int>(localization_for_label_.size()) <
      localization_msg_start_cnt) {
    return;
  }

  // generate trajectory points for one frame
  GenerateTrajectoryLabel(localization_for_label_, learning_data_frame_);
  const int localization_move_window_step =
      FLAGS_localization_freq / FLAGS_planning_freq;
  for (int i = 0; i < localization_move_window_step; ++i) {
    localization_for_label_.pop_front();
  }

  // write frames into a file
  if (learning_data_.learning_data_size() >=
      FLAGS_learning_data_frame_num_per_file) {
    const std::string file_name = absl::StrCat(
        FLAGS_planning_data_dir, "/learning_data.",
        learning_data_file_index_, ".bin");
    WriteOutLearningData(learning_data_, file_name);
  }
  learning_data_frame_ = learning_data_.add_learning_data();
}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {
  if (learning_data_frame_ == nullptr) {
    AERROR << "learning_data_frame_ pointer is nullptr";
    return;
  }
  auto features = learning_data_frame_->mutable_chassis();
  features->set_speed_mps(chassis.speed_mps());
  features->set_throttle_percentage(chassis.throttle_percentage());
  features->set_brake_percentage(chassis.brake_percentage());
  features->set_steering_percentage(chassis.steering_percentage());
  features->set_gear_location(chassis.gear_location());
}

void FeatureGenerator::OnRoutingResponse(
  const apollo::routing::RoutingResponse& routing_response) {
  if (learning_data_frame_ == nullptr) {
    AERROR << "learning_data_frame_ pointer is nullptr";
    return;
  }

  AINFO << "routing_response received at frame["
        << total_learning_data_frame_num_ << "]";

  routing_lane_ids.clear();
  for (int i = 0; i < routing_response.road_size(); ++i) {
    for (int j = 0; j < routing_response.road(i).passage_size(); ++j) {
      for (int k = 0; k < routing_response.road(i).passage(j).segment_size();
          ++k) {
        routing_lane_ids.push_back(
            routing_response.road(i).passage(j).segment(k).id());
      }
    }
  }
}

void FeatureGenerator::ProcessOfflineData(const std::string& record_filename) {
  RecordReader reader(record_filename);
  if (!reader.IsValid()) {
    AERROR << "Fail to open " << record_filename;
    return;
  }

  RecordMessage message;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == FLAGS_chassis_topic) {
      Chassis chassis;
      if (chassis.ParseFromString(message.content)) {
        OnChassis(chassis);
      }
    } else if (message.channel_name == FLAGS_localization_topic) {
      LocalizationEstimate localization;
      if (localization.ParseFromString(message.content)) {
        OnLocalization(localization);
      }
    } else if (message.channel_name == FLAGS_routing_response_topic) {
      RoutingResponse routing_response;
      if (routing_response.ParseFromString(message.content)) {
        OnRoutingResponse(routing_response);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
