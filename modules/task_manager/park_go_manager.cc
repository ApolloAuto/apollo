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
#include "modules/task_manager/park_go_manager.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/task_manager/common/task_manager_gflags.h"

namespace apollo {
namespace task_manager {

using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingRequest;
ParkGoManager::ParkGoManager() {}

bool ParkGoManager::near(LocalizationEstimate& localization,int index) {
    auto loc_x=localization.mutable_pose()->mutable_position()->x();
    auto loc_y=localization.mutable_pose()->mutable_position()->y();
    auto tar_x=wp_list_[index].mutable_pose()->x();
    auto tar_y=wp_list_[index].mutable_pose()->y();
    if ((loc_x-tar_x)*(loc_x-tar_x)+(loc_y-tar_y)*(loc_y-tar_y)<25) {
          return true;
    }
    return false;
}
common::Status ParkGoManager::Init(const ParkGOTask& park_go_task) {
  // get the message form routing
    wp_list_.clear();
    ParkGOTask tmp_park_go_task;
    tmp_park_go_task.CopyFrom(park_go_task);
    for (auto wp : tmp_park_go_task.mutable_routing_request()->waypoint()) {
          wp_list_.push_back(wp);
    } 
      return common::Status::OK();
}
RoutingRequest ParkGoManager::generate(LocalizationEstimate& localization,int index ,std::string lane_id,double s) {
  RoutingRequest routing_request;
        auto wp=routing_request.add_waypoint();
        wp->mutable_pose()->set_x(localization.mutable_pose()->mutable_position()->x());
         wp->mutable_pose()->set_y(localization.mutable_pose()->mutable_position()->y());
         wp->set_id(lane_id);
         wp->set_s(s);
        wp=routing_request.add_waypoint();
        wp->CopyFrom(wp_list_[index]);
        return routing_request;
}
  





}  // namespace task_manager
}  // namespace apollo
