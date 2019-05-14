/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "modules/bridge/bridge_control_component.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace bridge {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

BridgeControlComponent::BridgeControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool BridgeControlComponent::Init() {
  AINFO << "Bridge Control init, starting ...";

  CreateReader(chassis_reader_, FLAGS_chassis_topic,
    FLAGS_chassis_pending_queue_size);
  CreateReader(trajectory_reader_, FLAGS_planning_trajectory_topic,
    FLAGS_planning_pending_queue_size);
  CreateReader(localization_reader_, FLAGS_localization_topic,
    FLAGS_localization_pending_queue_size);
  CreateReader(pad_msg_reader_, FLAGS_pad_topic,
    FLAGS_pad_msg_pending_queue_size);

  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  return true;
}

bool BridgeControlComponent::Proc() {
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }

  trajectory_reader_->Observe();
  const auto &trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "planning msg is not ready!";
    return false;
  }

  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    return false;
  }

  pad_msg_reader_->Observe();
  const auto &pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg != nullptr) {
    OnPad(pad_msg);
  }

  // todo: send msgs through socket

  return true;
}

void BridgeControlComponent::OnPad(
    const std::shared_ptr<control::PadMessage> &pad) {
}

}  // namespace bridge
}  // namespace apollo
