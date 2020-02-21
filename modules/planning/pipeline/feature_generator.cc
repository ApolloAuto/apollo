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
using apollo::perception::TrafficLightDetection;
using apollo::routing::RoutingResponse;

void FeatureGenerator::Init() {}

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

void FeatureGenerator::OnLocalization(
    const apollo::localization::LocalizationEstimate& le) {
  localization_for_label_.push_back(le);

  const int localization_msg_start_cnt =
      FLAGS_localization_freq * FLAGS_trajectory_time_length;
  if (static_cast<int>(localization_for_label_.size()) <
      localization_msg_start_cnt) {
    return;
  }

  // generate one frame data
  GenerateLearningDataFrame();

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
}

void FeatureGenerator::OnChassis(const apollo::canbus::Chassis& chassis) {
  chassis_feature_.set_speed_mps(chassis.speed_mps());
  chassis_feature_.set_throttle_percentage(chassis.throttle_percentage());
  chassis_feature_.set_brake_percentage(chassis.brake_percentage());
  chassis_feature_.set_steering_percentage(chassis.steering_percentage());
  chassis_feature_.set_gear_location(chassis.gear_location());
}

void FeatureGenerator::OnTafficLightDetection(
    const TrafficLightDetection& traffic_light_detection) {
  // AINFO << "traffic_light_detection received at frame["
  //      << total_learning_data_frame_num_ << "]";

  traffic_lights_.clear();
  for (int i = 0; i < traffic_light_detection.traffic_light_size(); ++i) {
    const auto& traffic_light_id =
        traffic_light_detection.traffic_light(i).id();
    if (traffic_light_id.empty()) continue;

    // AINFO << "  traffic_light_id[" << traffic_light_id << "] color["
    //       << traffic_light_detection.traffic_light(i).color() << "]";
    traffic_lights_[traffic_light_id] =
        traffic_light_detection.traffic_light(i).color();
  }
}


void FeatureGenerator::OnRoutingResponse(
  const apollo::routing::RoutingResponse& routing_response) {
  AINFO << "routing_response received at frame["
        << total_learning_data_frame_num_ << "]";
  routing_lane_ids_.clear();
  for (int i = 0; i < routing_response.road_size(); ++i) {
    for (int j = 0; j < routing_response.road(i).passage_size(); ++j) {
      for (int k = 0; k < routing_response.road(i).passage(j).segment_size();
          ++k) {
        routing_lane_ids_.push_back(
            routing_response.road(i).passage(j).segment(k).id());
      }
    }
  }
}

void FeatureGenerator::GenerateTrajectoryPoints(
    const std::list<apollo::localization::LocalizationEstimate>&
        localization_for_label,
    LearningDataFrame* learning_data_frame) {
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

void FeatureGenerator::GenerateLearningDataFrame() {
  auto learning_data_frame = learning_data_.add_learning_data();
  // add timestamp_sec & frame_num
  learning_data_frame->set_timestamp_sec(
      localization_for_label_.back().header().timestamp_sec());
  learning_data_frame->set_frame_num(total_learning_data_frame_num_++);

  // add chassis
  auto chassis = learning_data_frame->mutable_chassis();
  chassis->CopyFrom(chassis_feature_);

  // add localization
  auto localization = learning_data_frame->mutable_localization();
  const auto& pose = localization_for_label_.back().pose();
  localization->mutable_position()->CopyFrom(pose.position());
  localization->set_heading(pose.heading());
  localization->mutable_linear_velocity()->CopyFrom(pose.linear_velocity());
  localization->mutable_linear_acceleration()->CopyFrom(
      pose.linear_acceleration());
  localization->mutable_angular_velocity()->CopyFrom(pose.angular_velocity());

  // add traffic_light
  learning_data_frame->clear_traffic_light();
  for (const auto& tl : traffic_lights_) {
    auto traffic_light = learning_data_frame->add_traffic_light();
    traffic_light->set_id(tl.first);
    traffic_light->set_color(tl.second);
  }

  // add routing
  auto routing_response = learning_data_frame->mutable_routing_response();
  routing_response->Clear();
  for (const auto& lane_id : routing_lane_ids_) {
    routing_response->add_lane_id(lane_id);
  }

  // add trajectory_points
  GenerateTrajectoryPoints(localization_for_label_, learning_data_frame);
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
    } else if (message.channel_name == FLAGS_traffic_light_detection_topic) {
      TrafficLightDetection traffic_light_detection;
      if (traffic_light_detection.ParseFromString(message.content)) {
        OnTafficLightDetection(traffic_light_detection);
      }
    }
  }
}

}  // namespace planning
}  // namespace apollo
