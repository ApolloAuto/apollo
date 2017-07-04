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

#include "modules/hmi/ros_node/ros_node_service.h"

#include <chrono>
#include <thread>

#include "gflags/gflags.h"
#include "grpc++/security/server_credentials.h"
#include "grpc++/server.h"
#include "grpc++/server_builder.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/pad_msg.pb.h"

DEFINE_string(hmi_ros_node_service_address, "127.0.0.1:8897",
              "HMI Ros node service address.");

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::control::DrivingAction;
using apollo::canbus::Chassis;

namespace apollo {
namespace hmi {
namespace {

static constexpr char kHMIRosNodeName[] = "hmi_ros_node_service";

void SendPadMessage(DrivingAction action) {
  apollo::control::PadMessage pb;
  pb.set_action(action);
  AINFO << "Sending PadMessage:\n" << pb.DebugString();
  AdapterManager::FillPadHeader(kHMIRosNodeName, pb.mutable_header());
  AdapterManager::PublishPad(pb);
}

void RunGRPCServer() {
  // Start GRPC service.
  HMIRosNodeImpl service;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(FLAGS_hmi_ros_node_service_address,
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  AINFO << "Server listening on " << FLAGS_hmi_ros_node_service_address;
  server->Wait();
}

}  // namespace

// Init static members.
std::mutex HMIRosNodeImpl::current_driving_mode_mutex_;
Chassis::DrivingMode HMIRosNodeImpl::current_driving_mode_ =
    Chassis::COMPLETE_MANUAL;

void HMIRosNodeImpl::Init() {
  // Setup AdapterManager.
  AdapterManagerConfig config;
  config.set_is_ros(true);
  {
    auto* sub_config = config.add_config();
    sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config->set_type(AdapterConfig::PAD);
  }

  {
    auto* sub_config = config.add_config();
    sub_config->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config->set_type(AdapterConfig::CHASSIS);
  }
  AdapterManager::Init(config);
  AdapterManager::SetChassisCallback(MonitorDrivingMode);
}

grpc::Status HMIRosNodeImpl::ChangeDrivingMode(
    grpc::ServerContext* context, const ChangeDrivingModeRequest* request,
    ChangeDrivingModeResponse* response) {
  AINFO << "received ChangeDrivingModeRequest: " << request->DebugString();
  auto driving_action_to_send = DrivingAction::RESET;
  auto driving_mode_to_wait = Chassis::COMPLETE_MANUAL;
  switch (request->action()) {
    case ChangeDrivingModeRequest::RESET_TO_MANUAL:
      // Default action and mode.
      break;
    case ChangeDrivingModeRequest::START_TO_AUTO:
      driving_action_to_send = DrivingAction::START;
      driving_mode_to_wait = Chassis::COMPLETE_AUTO_DRIVE;
      break;
    default:
      response->set_result(ChangeDrivingModeResponse::UNKNOWN);
      return grpc::Status(grpc::StatusCode::UNKNOWN,
                          "Unknown ChangeDrivingMode action.");
  }

  constexpr int kMaxTries = 5;
  constexpr auto kTryInterval = std::chrono::milliseconds(500);
  auto result = ChangeDrivingModeResponse::FAIL;
  for (int i = 0; i < kMaxTries; ++i) {
    // Send driving action periodically until entering target driving mode.
    SendPadMessage(driving_action_to_send);
    std::this_thread::sleep_for(kTryInterval);

    std::lock_guard<std::mutex> guard(current_driving_mode_mutex_);
    if (current_driving_mode_ == driving_mode_to_wait) {
      result = ChangeDrivingModeResponse::SUCCESS;
      break;
    }
  }
  response->set_result(result);
  AINFO << "ChangeDrivingModeResponse: " << response->DebugString();
  if (result == ChangeDrivingModeResponse::FAIL) {
    AERROR << "Failed to change driving mode to " << request->DebugString();
  }
  return grpc::Status::OK;
}

void HMIRosNodeImpl::MonitorDrivingMode(const apollo::canbus::Chassis& status) {
  auto driving_mode = status.driving_mode();
  std::lock_guard<std::mutex> guard(current_driving_mode_mutex_);
  // Update current_driving_mode_ when it is changed.
  if (driving_mode != current_driving_mode_) {
    AINFO << "Found Chassis DrivingMode changed: "
          << Chassis_DrivingMode_Name(current_driving_mode_) << " -> "
          << Chassis_DrivingMode_Name(driving_mode);
    current_driving_mode_ = driving_mode;
  }
}

}  // namespace hmi
}  // namespace apollo

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Setup ros node.
  ros::init(argc, argv, apollo::hmi::kHMIRosNodeName);
  apollo::hmi::HMIRosNodeImpl::Init();

  // Run GRPC server in background thread.
  std::thread grpc_server_thread(apollo::hmi::RunGRPCServer);

  ros::spin();
  grpc_server_thread.join();
  return 0;
}
