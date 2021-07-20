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
#include "modules/task_manager/task_manager_component.h"

#include "modules/task_manager/proto/task_manager_config.pb.h"
#include "cyber/time/rate.h"

namespace apollo {
namespace task_manager {

using apollo::cyber::ComponentBase;
using apollo::cyber::Rate;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::planning::ADCTrajectory;

bool TaskManagerComponent::Init() {
  TaskManagerConfig task_manager_conf;
  ACHECK(cyber::ComponentBase::GetProtoConfig(&task_manager_conf))
      << "Unable to load task_manager conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";

  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      task_manager_conf.topic_config().localization_pose_topic(),
      [this](const std::shared_ptr<LocalizationEstimate>& localization) {
        ADEBUG << "Received localization data: run localization callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        localization_.CopyFrom(*localization);
      });

  response_reader_ = node_->CreateReader<RoutingResponse>(
      task_manager_conf.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& response) {
        ADEBUG << "Received routing_response data: run response callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        routing_response_.CopyFrom(*response);
      });

  cyber::proto::RoleAttributes attr;
  attr.set_channel_name(
      task_manager_conf.topic_config().routing_request_topic());
  auto qos = attr.mutable_qos_profile();
  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
  qos->set_reliability(
      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
  request_writer_ = node_->CreateWriter<RoutingRequest>(attr);
  return true;
}

bool TaskManagerComponent::Proc(const std::shared_ptr<Task>& task) {
  if (task->task_type() != CYCLE_ROUTING &&
      task->task_type() != PARKING_ROUTING &&
      task->task_type() != DEAD_END_ROUTING) {
    AERROR << "Task type is not cycle_routing or parking_routing.";
    return false;
  }

  if (task->task_type() == CYCLE_ROUTING) {
    cycle_routing_manager_ = std::make_shared<CycleRoutingManager>();
    cycle_routing_manager_->Init(task->cycle_routing_task());
    routing_request_ = task->cycle_routing_task().routing_request();
    Rate rate(1.0);

    while (cycle_routing_manager_->GetCycle() > 0) {
      if (cycle_routing_manager_->GetNewRouting(localization_.pose(),
                                                &routing_request_)) {
        auto last_routing_response_ = routing_response_;
        common::util::FillHeader(node_->Name(), &routing_request_);
        request_writer_->Write(routing_request_);
        AINFO << "[TaskManagerComponent]Reach begin/end point: "
              << "routing manager send a routing request. ";
        rate.Sleep();

        if (!routing_response_.has_header()) {
          AINFO << "[TaskManagerComponent]routing failed";
          return false;
        }
        if (last_routing_response_.has_header()) {
          if (last_routing_response_.header().sequence_num() ==
              routing_response_.header().sequence_num()) {
            AINFO << "[TaskManagerComponent]No routing response: "
                  << "new routing failed";
            return false;
          }
        }
      }
      rate.Sleep();
    }
  } else if (task->task_type() == PARKING_ROUTING) {
    AERROR << "enter the parking routing task";
    parking_routing_manager_ = std::make_shared<ParkingRoutingManager>();
    parking_routing_manager_->Init(task->parking_routing_task());
    routing_request_ = task->parking_routing_task().routing_request();
    if (parking_routing_manager_->SizeVerification(
            task->parking_routing_task()) &&
        parking_routing_manager_->RoadWidthVerification(
            task->parking_routing_task())) {
      AERROR << "compelet the Verification";
      common::util::FillHeader(node_->Name(), &routing_request_);
      request_writer_->Write(routing_request_);
      AINFO << "send a auto parking task";
    } else {
      auto last_routing_response_ = routing_response_;
      if (!routing_response_.has_header()) {
           AINFO << "[TaskManagerComponent]parking routing failed";
           return false;
         }
         if (last_routing_response_.has_header()) {
           if (last_routing_response_.header().sequence_num() ==
               routing_response_.header().sequence_num()) {
             AINFO << "[TaskManagerComponent]No parking routing response: "
                   << "new parking routing failed";
             return false;
           }
         }
      AERROR << "plot verification failed, please select suitable plot!";
      return false;
    }
  } else if (task->task_type() == DEAD_END_ROUTING) {
    dead_end_routing_manager_ = std::make_shared<DeadEndRoutingManager>();
    dead_end_routing_manager_->Init(task->dead_end_routing_task());
    routing_request_ = task->dead_end_routing_task().routing_request_in();
    Rate rate(1.0);
    while (dead_end_routing_manager_->GetNumber() > 0) {
      if (dead_end_routing_manager_->GetNewRouting(localization_.pose(),
                                                   &routing_request_)) {
        common::util::FillHeader(node_->Name(), &routing_request_);
        request_writer_->Write(routing_request_);
        rate.Sleep();
      }
    rate.Sleep();
    }
  }
  return true;
}

}  // namespace task_manager
}  // namespace apollo
