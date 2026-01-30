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

#include "modules/external_command/external_command_demo/external_command_wrapper_demo.h"
#include <poll.h>

#include <cctype>
#include <algorithm>

#include "modules/external_command/external_command_demo/proto/sweeper_custom_command.pb.h"

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"

using apollo::external_command::CommandStatus;

ExternalCommandWrapperDemo::ExternalCommandWrapperDemo()
    : command_id_(0), module_name_("demo") {}

bool ExternalCommandWrapperDemo::Init() {
  action_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::ActionCommand, CommandStatus>>(
      node_, "/apollo/external_command/action");
  chassis_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::ChassisCommand, CommandStatus>>(
      node_, "/apollo/external_command/chassis");
  free_space_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::FreeSpaceCommand, CommandStatus>>(
      node_, "/apollo/external_command/free_space");
  zone_cover_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::ZoneCoverCommand, CommandStatus>>(
      node_, "/apollo/external_command/zone_cover");
  lane_follow_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::LaneFollowCommand, CommandStatus>>(
      node_, "/apollo/external_command/lane_follow");
  path_follow_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::PathFollowCommand, CommandStatus>>(
      node_, "/apollo/external_command/path_follow");
  speed_command_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::SpeedCommand, CommandStatus>>(
      node_, "/apollo/external_command/speed");
  valet_parking_command_client_ =
      std::make_shared<apollo::common::ClientWrapper<
          apollo::external_command::ValetParkingCommand, CommandStatus>>(
          node_, "/apollo/external_command/valet_parking");
  status_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::CommandStatusRequest, CommandStatus>>(
      node_, "/apollo/external_command/command_status");
  apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/external_command/external_command_demo/conf/"
      "demo_config.pb.txt",
      &demo_config_);
  return true;
}

bool ExternalCommandWrapperDemo::Proc() {
  int8_t revent = 0;  // short
  struct pollfd fd = {STDIN_FILENO, POLLIN, revent};
  switch (poll(&fd, 1, 100)) {
    case -1:
      std::cout << "Failed to read keyboard" << std::endl;
      return false;
    case 0:
      return true;
    default:
      char data[50];
      std::cin.getline(data, 50);
      std::string input_command_string = data;
      if (input_command_string == "pull_over") {
        // Pull over.
        SendActionCommand(
            apollo::external_command::ActionCommandType::PULL_OVER);
      } else if (input_command_string == "stop") {
        // Stop planning.
        SendActionCommand(apollo::external_command::ActionCommandType::STOP);
      } else if (input_command_string == "start") {
        // Start planning.
        SendActionCommand(apollo::external_command::ActionCommandType::START);
      } else if (input_command_string == "clear") {
        // Start planning.
        SendActionCommand(
            apollo::external_command::ActionCommandType::CLEAR_PLANNING);
      } else if (input_command_string == "manual") {
        // Switch manual mode.
        SendActionCommand(
            apollo::external_command::ActionCommandType::SWITCH_TO_MANUAL);
      } else if (input_command_string == "auto") {
        // Switch auto mode.
        SendActionCommand(
            apollo::external_command::ActionCommandType::SWITCH_TO_AUTO);
      } else if (input_command_string == "vin") {
        // Send vin validation.
        SendActionCommand(apollo::external_command::ActionCommandType::VIN_REQ);
      } else if (input_command_string == "enter_mission") {
        // Enter mission model.
        SendActionCommand(
            apollo::external_command::ActionCommandType::ENTER_MISSION);
      } else if (input_command_string == "exit_mission") {
        // Exit mission model.
        SendActionCommand(
            apollo::external_command::ActionCommandType::EXIT_MISSION);
      } else if (input_command_string == "chassis") {
        SendVehicleSignalCommand();
      } else if (input_command_string == "custom_chassis") {
        SendCustomChassisCommand();
      } else if (input_command_string.find("set_speed") != std::string::npos) {
        auto index = input_command_string.find("set_speed");
        std::string speed_value_string = input_command_string.substr(
            index + std::string("set_speed").length(),
            input_command_string.length());
        if (!speed_value_string.empty()) {
          double speed_value = std::atof(speed_value_string.c_str());
          SendSpeedCommand(speed_value);
        } else {
          AWARN << "Input format is invalid, please input format like: "
                   "set_speed 1.5";
        }
      } else if (input_command_string == "increase_speed") {
        double speed_factor = 1.2;
        SendSpeedFactorCommand(speed_factor);
      } else if (input_command_string == "decrease_speed") {
        double speed_factor = 0.8;
        SendSpeedFactorCommand(speed_factor);
      } else if (input_command_string == "restore_speed") {
        RestoreSpeed();
      } else if (input_command_string == "lane") {
        // Modify way point as needed.
        apollo::external_command::Pose way_point;
        way_point.set_x(0.0);
        way_point.set_y(0.0);
        way_point.set_heading(0.0);
        std::vector<apollo::external_command::Pose> way_points;
        way_points.emplace_back(way_point);
        apollo::external_command::Pose end_pose;
        end_pose.set_x(10.0);
        end_pose.set_y(0.0);
        end_pose.set_heading(0.0);
        SendLaneFollowCommand(way_points, end_pose,
                              demo_config_.target_speed());
      } else if (input_command_string == "path_loc") {
        SendPathFollowCommandWithLocationRecord(
            demo_config_.file_of_path_follow_with_localization_record());
      } else if (input_command_string == "path_path") {
        SendPathFollowCommandWithPathRecord(
            demo_config_.file_of_path_follow_with_planning_record());
      } else if (input_command_string == "valet_parking") {
        std::string parking_spot_id = "451089045";
        SendValetParkingCommand(parking_spot_id, demo_config_.target_speed());
      } else if (input_command_string.find("command_status") !=
                 std::string::npos) {
        // Input command with format: command_status=XX, where XX is the
        // command_id
        const size_t start_pos = std::string("command_status=").length();
        const size_t end_pos = input_command_string.length();
        if (end_pos <= start_pos) {
          std::cout
              << "Please check command status with format: command_status=XX!"
              << std::endl;
          break;
        }
        std::string command_id_string =
            input_command_string.substr(start_pos, end_pos);
        command_id_string.erase(
            std::remove_if(command_id_string.begin(), command_id_string.end(),
                           ::isspace),
            command_id_string.end());
        uint64_t id = std::atoi(command_id_string.c_str());
        CheckCommandStatus(id);
      } else if (input_command_string == "free1") {
        apollo::external_command::Pose end_pose;
        end_pose.set_x(demo_config_.end_pose_x());
        end_pose.set_y(demo_config_.end_pose_y());
        end_pose.set_heading(demo_config_.end_pose_heading());
        std::vector<apollo::external_command::Point> way_points;
        apollo::external_command::Point point1;
        apollo::external_command::Point point2;
        apollo::external_command::Point point3;
        apollo::external_command::Point point4;
        point1.set_x(demo_config_.point1_x());
        point1.set_y(demo_config_.point1_y());
        point2.set_x(demo_config_.point2_x());
        point2.set_y(demo_config_.point2_y());
        point3.set_x(demo_config_.point3_x());
        point3.set_y(demo_config_.point3_y());
        point4.set_x(demo_config_.point4_x());
        point4.set_y(demo_config_.point4_y());
        way_points.emplace_back(point1);
        way_points.emplace_back(point2);
        way_points.emplace_back(point3);
        way_points.emplace_back(point4);

        SendFreespaceCommand(way_points, end_pose);
      } else if (input_command_string == "zone_cover") {
        std::string zone_cover_area_id = demo_config_.zone_cover_area_id();
        SendZoneCoverCommand(zone_cover_area_id);
      } else {
        std::cout << "Invalid input!" << input_command_string << std::endl;
      }
  }
  return true;
}

void ExternalCommandWrapperDemo::SendActionCommand(
    apollo::external_command::ActionCommandType action_command_type) {
  auto command = std::make_shared<apollo::external_command::ActionCommand>();
  FillCommandHeader(command);
  command->set_command(action_command_type);
  std::cout << "Sending action command: " << command->DebugString()
            << std::endl;
  auto response = action_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendVehicleSignalCommand() {
  // Send left turn signal.
  auto command = std::make_shared<apollo::external_command::ChassisCommand>();
  FillCommandHeader(command);
  command->mutable_basic_signal()->set_turn_signal(
      apollo::common::VehicleSignal::TURN_LEFT);
  std::cout << "Sending chassis command: " << command->DebugString()
            << std::endl;
  auto response = chassis_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendCustomChassisCommand() {
  // Send left turn signal.
  auto command = std::make_shared<apollo::external_command::ChassisCommand>();
  FillCommandHeader(command);
  // Set custom command.
  auto custom_operation = command->mutable_custom_operation();
  // Set custom command values.
  apollo::external_command::SweeperCustomCommand sweeper_command;
  sweeper_command.set_is_turn_on_brush(true);
  sweeper_command.set_sweeping_speed(2.0);
  custom_operation->PackFrom(sweeper_command);
  std::cout << "Sending custom chassis command: " << command->DebugString()
            << std::endl;
  auto response = chassis_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendPathFollowCommandWithPathRecord(
    const std::string& record_path) {
  // Read planning data from record file and use the planning path points
  // as the path of PathFollowCommand.
  apollo::planning::ADCTrajectory record_planning_data;
  // Get the path points from record planning data.
  std::shared_ptr<apollo::external_command::PathFollowCommand>
      path_follow_command =
          std::make_shared<apollo::external_command::PathFollowCommand>();
  std::vector<apollo::external_command::Point> left_boundary_points,
      right_boundary_points;

  std::vector<std::string> record_files =
      apollo::cyber::common::ListSubPaths(record_path, DT_REG);
  std::sort(record_files.begin(), record_files.end());
  std::string dir_prefix = record_path + '/';
  for (const auto file_name : record_files) {
    ReadPathFromPathRecord(dir_prefix + file_name,
                           path_follow_command->mutable_way_point(),
                           left_boundary_points, right_boundary_points);
  }
  // Set header and command id of PathFollowCommand.
  FillCommandHeader(path_follow_command);
  // Set path boundary of path.
  if (left_boundary_points.size() == right_boundary_points.size() &&
      right_boundary_points.size() == path_follow_command->way_point().size() &&
      right_boundary_points.size() > 2) {
    std::cout << "Get the left and right boundary Point!\n";
    auto path_boundary = path_follow_command->mutable_path_boundary();
    for (auto& left_boundary_point : left_boundary_points) {
      auto left_boundary = path_boundary->add_left_boundary();
      left_boundary->set_x(left_boundary_point.x());
      left_boundary->set_y(left_boundary_point.y());
    }
    for (auto& right_boundary_point : right_boundary_points) {
      auto right_boundary = path_boundary->add_right_boundary();
      right_boundary->set_x(right_boundary_point.x());
      right_boundary->set_y(right_boundary_point.y());
    }
    std::cout << "left_boundary size: " << path_boundary->left_boundary().size()
              << ", right_boundary size"
              << path_boundary->right_boundary().size()
              << "reference size: " << path_follow_command->way_point().size();

  } else {
    std::cout << "Set the left and right boundary with width of config!\n";
    auto path_boundary = path_follow_command->mutable_boundary_with_width();
    path_boundary->set_left_path_width(demo_config_.left_path_width());
    path_boundary->set_right_path_width(demo_config_.right_path_width());
  }
  // Set target speed.
  path_follow_command->set_target_speed(demo_config_.target_speed());
  auto response = path_follow_command_client_->SendRequest(path_follow_command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendPathFollowCommandWithLocationRecord(
    const std::string& record_dir) {
  // Read planning data from record file and use the planning path points
  // as the path of PathFollowCommand.
  std::shared_ptr<apollo::external_command::PathFollowCommand>
      path_follow_command =
          std::make_shared<apollo::external_command::PathFollowCommand>();
  std::vector<std::string> record_files =
      apollo::cyber::common::ListSubPaths(record_dir, DT_REG);
  std::sort(record_files.begin(), record_files.end());
  std::string dir_prefix = record_dir + '/';
  for (const auto file_name : record_files) {
    ReadPathFromLocationRecord(dir_prefix + file_name,
                               path_follow_command->mutable_way_point());
  }

  // Set header and command id of PathFollowCommand.
  FillCommandHeader(path_follow_command);
  // Set path boundary of path.
  auto path_boundary = path_follow_command->mutable_boundary_with_width();
  path_boundary->set_left_path_width(demo_config_.left_path_width());
  path_boundary->set_right_path_width(demo_config_.right_path_width());
  // Set target speed.
  path_follow_command->set_target_speed(demo_config_.target_speed());
  auto response = path_follow_command_client_->SendRequest(path_follow_command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendSpeedCommand(double speed) {
  auto command = std::make_shared<apollo::external_command::SpeedCommand>();
  FillCommandHeader(command);
  command->set_target_speed(speed);
  std::cout << "Sending speed command: " << command->DebugString() << std::endl;
  auto response = speed_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendSpeedFactorCommand(double speed_factor) {
  auto command = std::make_shared<apollo::external_command::SpeedCommand>();
  FillCommandHeader(command);
  command->set_target_speed_factor(speed_factor);
  std::cout << "Sending speed factor command: " << command->DebugString()
            << std::endl;
  auto response = speed_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::RestoreSpeed() {
  auto command = std::make_shared<apollo::external_command::SpeedCommand>();
  FillCommandHeader(command);
  command->set_is_restore_target_speed(true);
  std::cout << "Restore speed: " << command->DebugString() << std::endl;
  auto response = speed_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendLaneFollowCommand(
    const std::vector<apollo::external_command::Pose>& way_points,
    const apollo::external_command::Pose& end, double target_speed) {
  auto command =
      std::make_shared<apollo::external_command::LaneFollowCommand>();
  FillCommandHeader(command);
  // Copy way_points
  for (const auto& point : way_points) {
    auto way_point = command->add_way_point();
    way_point->CopyFrom(point);
  }
  // Copy end point
  command->mutable_end_pose()->CopyFrom(end);
  if (target_speed > 0.0) {
    command->set_target_speed(target_speed);
  }
  std::cout << "Sending lane follow command: " << command->DebugString()
            << std::endl;
  auto response = lane_follow_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendFreespaceCommand(
    const std::vector<apollo::external_command::Point>& way_points,
    const apollo::external_command::Pose& end) {
  auto command = std::make_shared<apollo::external_command::FreeSpaceCommand>();
  FillCommandHeader(command);
  // Copy way_points
  auto roi_point = command->mutable_drivable_roi();
  for (const auto& point : way_points) {
    roi_point->add_point()->CopyFrom(point);
  }
  // Copy end point
  command->mutable_parking_spot_pose()->CopyFrom(end);
  std::cout << "Sending lane follow command: " << command->DebugString()
            << std::endl;
  auto response = free_space_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::SendValetParkingCommand(
    const std::string& parking_spot_id, double target_speed) {
  auto command =
      std::make_shared<apollo::external_command::ValetParkingCommand>();
  FillCommandHeader(command);
  command->set_parking_spot_id(parking_spot_id);
  if (target_speed > 0.0) {
    command->set_target_speed(target_speed);
  }
  std::cout << "Sending valet parking command: " << command->DebugString()
            << std::endl;
  auto response = valet_parking_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::ReadPathFromPathRecord(
    const std::string& record_file,
    google::protobuf::RepeatedPtrField<apollo::external_command::Point>*
        waypoints,
    std::vector<apollo::external_command::Point>& left_boundary_points,
    std::vector<apollo::external_command::Point>& right_boundary_points) {
  std::cout << "ReadPathFromPathRecord: " << record_file << std::endl;
  apollo::cyber::record::RecordReader reader(record_file);
  if (!reader.IsValid()) {
    std::cout << "Fail to open " << record_file << std::endl;
    return;
  }

  apollo::planning::ADCTrajectory planning_trajectory;
  apollo::cyber::record::RecordMessage message;
  double start_time = apollo::cyber::Clock::NowInSeconds();
  double last_point_x = 0.0;
  double last_point_y = 0.0;
  //   waypoints->Clear();
  //   left_boundary_points.clear();
  //   right_boundary_points.clear();
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == "/apollo/planning") {
      if (planning_trajectory.ParseFromString(message.content)) {
        const auto& location_pose = planning_trajectory.location_pose();
        if ((abs(last_point_x - location_pose.vehice_location().x()) >
                 demo_config_.min_distance_error() ||
             abs(last_point_y - location_pose.vehice_location().y()) >
                 demo_config_.min_distance_error())) {
          if (location_pose.has_left_lane_boundary_point() &&
              location_pose.has_right_lane_boundary_point()) {
            apollo::external_command::Point left_boundary_point,
                right_boundary_point;
            left_boundary_point.set_x(
                location_pose.left_lane_boundary_point().x());
            left_boundary_point.set_y(
                location_pose.left_lane_boundary_point().y());
            right_boundary_point.set_x(
                location_pose.right_lane_boundary_point().x());
            right_boundary_point.set_y(
                location_pose.right_lane_boundary_point().y());
            left_boundary_points.emplace_back(left_boundary_point);
            right_boundary_points.emplace_back(right_boundary_point);
          }

          auto output_point = waypoints->Add();
          output_point->set_x(location_pose.vehice_location().x());
          output_point->set_y(location_pose.vehice_location().y());
          last_point_x = location_pose.vehice_location().x();
          last_point_y = location_pose.vehice_location().y();
        }
      }

    } else {
      std::cout << "Stop Parse message";
    }
  }
  std::cout << "waypoints size: " << waypoints->size() << "\n";
  std::cout << "Parse /apollo/planning message Time =: "
            << (apollo::cyber::Clock::NowInSeconds() - start_time) * 1000
            << "\n";
}

void ExternalCommandWrapperDemo::CheckCommandStatus(const uint64_t command_id) {
  auto command =
      std::make_shared<apollo::external_command::CommandStatusRequest>();
  FillCommandHeader(command);
  command->set_command_id(command_id);
  std::cout << "Sending check command: " << command->DebugString() << std::endl;
  auto response = status_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Check status failed!\n" << std::endl;
  } else {
    std::cout << response->DebugString();
    std::cout << "******Finish checking command.******\n" << std::endl;
  }
}

void ExternalCommandWrapperDemo::ReadPathFromLocationRecord(
    const std::string& record_file,
    google::protobuf::RepeatedPtrField<apollo::external_command::Point>*
        waypoints) const {
  std::cout << "ReadPathFromLocationRecord: " << record_file << std::endl;
  apollo::cyber::record::RecordReader reader(record_file);
  if (!reader.IsValid()) {
    std::cout << "Fail to open " << record_file << std::endl;
    return;
  }
  apollo::localization::LocalizationEstimate localization;
  apollo::cyber::record::RecordMessage message;
  double last_x = 0.0;
  double last_y = 0.0;
  bool is_last_poistion_set = false;
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == "/apollo/localization/pose") {
      if (localization.ParseFromString(message.content)) {
        const auto& position = localization.pose().position();
        if (!is_last_poistion_set) {
          last_x = position.x();
          last_y = position.y();
          is_last_poistion_set = true;
          continue;
        }
        // Save the location to path with min_distance.
        if (abs(last_x - position.x()) > demo_config_.min_distance_error() ||
            abs(last_y - position.y()) > demo_config_.min_distance_error()) {
          auto output_point = waypoints->Add();
          output_point->set_x(position.x());
          output_point->set_y(position.y());
          last_x = position.x();
          last_y = position.y();
        }
      }
    }
  }
}

void ExternalCommandWrapperDemo::SendZoneCoverCommand(
    const std::string zone_cover_area_id) {
  auto command = std::make_shared<apollo::external_command::ZoneCoverCommand>();
  FillCommandHeader(command);
  // Copy way_points
  command->set_zone_cover_area_id(zone_cover_area_id);
  std::cout << "Sending lane follow command: " << command->DebugString()
            << std::endl;
  auto response = zone_cover_command_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
}
