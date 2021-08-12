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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/task_manager/proto/task_manager_config.pb.h"

#include "cyber/time/rate.h"

namespace apollo {
namespace task_manager {

using apollo::cyber::ComponentBase;
using apollo::cyber::Rate;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::canbus::Chassis;
using apollo::hdmap::HDMapUtil;
bool TaskManagerComponent::Init() {
  TaskManagerConfig task_manager_conf;
  ACHECK(cyber::ComponentBase::GetProtoConfig(&task_manager_conf))
      << "Unable to load task_manager conf file: "
      << cyber::ComponentBase::ConfigFilePath();

  AINFO << "Config file: " << cyber::ComponentBase::ConfigFilePath()
        << " is loaded.";
  chassis_reader_= node_->CreateReader<Chassis>(
      task_manager_conf.topic_config().chassis_topic(), 
      [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received localization data: run localization callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_.CopyFrom(*chassis);
      });
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
  // Don't send the history message when new readers are found.
  qos->set_durability(
      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_SYSTEM_DEFAULT);
  request_writer_ = node_->CreateWriter<RoutingRequest>(attr);
  return true;
}

bool TaskManagerComponent::Proc(const std::shared_ptr<Task>& task) {
  task_name_ = task->task_name();
  if (task->task_type() != CYCLE_ROUTING &&
      task->task_type() != PARKING_ROUTING && task->task_type()!=PARK_GO_ROUTING) {
    AERROR << "Task type is not cycle_routing or parking_routing or park go.";
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
  } else if (task->task_type() == PARK_GO_ROUTING) {
        AINFO<<"Enter park&go task";
        if (park_go_manager_==nullptr) {
                park_go_manager_ = std::make_shared<ParkGoManager>();
                park_go_manager_->Init(task->mutable_park_go_routing_task());
        }
        int wp_size=task->mutable_park_go_routing_task()->mutable_routing_request()->waypoint_size();
        int stage=0;
        int park_time=task->mutable_park_go_routing_task()->park_time()*1000;
        auto basemap=HDMapUtil::BaseMapPtr();
        apollo::hdmap::LaneInfoConstPtr lane;
        apollo::common::PointENU point;
        double s,l,heading;
        while (stage<wp_size) {
          if (park_go_manager_->near(localization_,stage)&&chassis_.speed_mps()<0.2) {
                cyber::SleepFor(std::chrono::milliseconds(park_time));
                stage++;
                if (stage==wp_size)
                      break;
                point.set_x(localization_.mutable_pose()->mutable_position()->x());
                point.set_y(localization_.mutable_pose()->mutable_position()->y());
                heading=localization_.mutable_pose()->heading();
                basemap->GetNearestLaneWithHeading(point,5,heading,2,&lane,&s,&l);
                routing_request_=park_go_manager_->generate(localization_,stage,lane->id().id(),s);
                common::util::FillHeader(node_->Name(), &routing_request_  );
                request_writer_->Write(routing_request_);
          }
        }
  
       
        
  }
  return true;
}

}  // namespace task_manager
}  // namespace apollo
