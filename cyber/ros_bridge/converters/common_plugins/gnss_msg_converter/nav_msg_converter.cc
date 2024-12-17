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

#include "cyber/ros_bridge/converters/common_plugins/gnss_msg_converter/nav_msg_converter.h"  // NOLINT
#include <cmath>

namespace apollo {
namespace cyber {

bool NavMsgConverter::ConvertMsg(
    InputTypes<RosNavMsgPtr>& in,
    OutputTypes<BestPoseMsgPtr, InsStatMsgPtr>& out) {
#ifdef ENABLE_ROS_MSG
  auto ros_nav_ptr = std::get<0>(in.values);
  auto& ros_nav = (*ros_nav_ptr);

  auto best_pose_msg = std::get<0>(out.values);
  auto ins_stat_msg = std::get<1>(out.values);

  auto unix_msg_time =
      ros_nav.header.stamp.sec + ros_nav.header.stamp.nanosec / 1e9;
  best_pose_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  best_pose_msg->mutable_header()->set_module_name("gnss");

  ins_stat_msg->mutable_header()->set_timestamp_sec(unix_msg_time);
  ins_stat_msg->mutable_header()->set_module_name("gnss");

  best_pose_msg->set_measurement_time(unix_msg_time);
  best_pose_msg->set_latitude(ros_nav.latitude);
  best_pose_msg->set_longitude(ros_nav.longitude);
  best_pose_msg->set_height_msl(ros_nav.altitude);
  best_pose_msg->set_undulation(0);
  best_pose_msg->set_datum_id(apollo::drivers::gnss::DatumId::WGS84);
  best_pose_msg->set_latitude_std_dev(sqrt(ros_nav.position_covariance[4]));
  best_pose_msg->set_longitude_std_dev(sqrt(ros_nav.position_covariance[0]));
  best_pose_msg->set_height_std_dev(sqrt(ros_nav.position_covariance[8]));
  if (ros_nav.status.status == -1) {
    best_pose_msg->set_sol_status(
        apollo::drivers::gnss::SolutionStatus::INVALID_FIX);
    best_pose_msg->set_sol_type(apollo::drivers::gnss::SolutionType::NONE);
    ins_stat_msg->set_pos_type(0);
  } else if (ros_nav.status.status == 0) {
    best_pose_msg->set_sol_status(
        apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
    best_pose_msg->set_sol_type(apollo::drivers::gnss::SolutionType::SINGLE);
    ins_stat_msg->set_pos_type(1);
  } else if (ros_nav.status.status == 1) {
    best_pose_msg->set_sol_status(
        apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
    best_pose_msg->set_sol_type(apollo::drivers::gnss::SolutionType::WAAS);
    ins_stat_msg->set_pos_type(2);
  } else if (ros_nav.status.status == 2) {
    best_pose_msg->set_sol_status(
        apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
    best_pose_msg->set_sol_type(
        apollo::drivers::gnss::SolutionType::NARROW_INT);
    ins_stat_msg->set_pos_type(2);
  }

#endif
  return true;
}

}  // namespace cyber
}  // namespace apollo
