/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "cyber/statistics/statistics.h"

namespace apollo {
namespace cyber {
namespace statistics {

Statistics::Statistics() {}

bool Statistics::RegisterChanVar(const proto::RoleAttributes& role_attr) {
  if (latency_map_.find(GetProcLatencyKey(role_attr)) != latency_map_.end()) {
    AERROR << "Failed to create proc latency var: "
             "reader with the same channel already exists.";
    return false;
  }

  if (latency_map_.find(GetTranLatencyKey(role_attr)) != latency_map_.end()) {
    AERROR << "Failed to create tran latency var: "
             "reader with the same channel already exists.";
    return false;
  }

  if (latency_map_.find(GetCyberLatencyKey(role_attr)) != latency_map_.end()) {
    AERROR << "Failed to create cyber latency var: "
             "reader with the same channel already exists.";
    return false;
  }

  latency_map_[GetProcLatencyKey(role_attr)] =
          std::make_shared<::bvar::LatencyRecorder>(
                  role_attr.node_name() + "-"
                  + role_attr.channel_name(), "proc");
  if (role_attr.channel_name() != TIMER_COMPONENT_CHAN_NAME) {
    latency_map_[GetTranLatencyKey(role_attr)] =
          std::make_shared<::bvar::LatencyRecorder>(
                  role_attr.node_name() + "-" +
                  role_attr.channel_name(), "tran");
    latency_map_[GetCyberLatencyKey(role_attr)] =
          std::make_shared<::bvar::LatencyRecorder>(
                  role_attr.node_name() + "-" +
                  role_attr.channel_name(), "cyber");
    status_map_[GetStartProcessStatusKey(role_attr)] =
          std::make_shared<::bvar::Status<uint64_t>>(
                  role_attr.node_name() + "-" +
                  role_attr.channel_name() + "-process", 0);
    status_map_[GetTotalMsgsStatusKey(role_attr)] =
          std::make_shared<::bvar::Status<uint64_t>>(
                  role_attr.node_name() + "-" +
                  role_attr.channel_name() + "-total-msgs-nums", 0);
    adder_map_[GetTotalRecvStatusKey(role_attr)] =
          std::make_shared<::bvar::Adder<int32_t>>(
                  role_attr.node_name() +
                  "-" + role_attr.channel_name() + "-recv-msgs-nums");
  }
  return true;
}

StatusVarPtr Statistics::GetProcStatusVar(
                          const proto::RoleAttributes& role_attr) {
  auto v = status_map_.find(GetStartProcessStatusKey(role_attr));
  if (v == status_map_.end()) {
    return nullptr;
  }
  return v->second;
}

LatencyVarPtr Statistics::GetChanProcVar(
                    const proto::RoleAttributes& role_attr) {
  auto v = latency_map_.find(GetProcLatencyKey(role_attr));
  if (v == latency_map_.end()) {
    return nullptr;
  }
  return v->second;
}

LatencyVarPtr Statistics::GetChanTranVar(
                    const proto::RoleAttributes& role_attr) {
  auto v = latency_map_.find(GetTranLatencyKey(role_attr));
  if (v == latency_map_.end()) {
    return nullptr;
  }
  return v->second;
}

LatencyVarPtr Statistics::GetChanCyberVar(
                    const proto::RoleAttributes& role_attr) {
  auto v = latency_map_.find(GetCyberLatencyKey(role_attr));
  if (v == latency_map_.end()) {
    return nullptr;
  }
  return v->second;
}

StatusVarPtr Statistics::GetTotalMsgsStatusVar(
                      const proto::RoleAttributes& role_attr) {
  auto v = status_map_.find(GetTotalMsgsStatusKey(role_attr));
  if (v == status_map_.end()) {
    return nullptr;
  }
  return v->second;
}

AdderVarPtr Statistics::GetAdderVar(
                      const proto::RoleAttributes& role_attr) {
  auto v = adder_map_.find(GetTotalRecvStatusKey(role_attr));
  if (v == adder_map_.end()) {
    return nullptr;
  }
  return v->second;
}


}  // namespace statistics
}  // namespace cyber
}  // namespace apollo
