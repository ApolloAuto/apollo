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
#include "modules/map/tools/map_datachecker/server/eight_route.h"

#include <cmath>
#include <vector>

namespace apollo {
namespace hdmap {

EightRoute::EightRoute(std::shared_ptr<JsonConf> sp_conf) : Alignment(sp_conf) {
  Reset();
}

void EightRoute::Reset() {
  progress_ = 0.0;
  last_progress_ = 0;
}

bool EightRoute::IsEightRoutePose(const std::vector<FramePose>& poses,
                                  int pose_index) {
  if (poses.empty() || pose_index <= 0 ||
      pose_index >= static_cast<int>(poses.size())) {
    AINFO << "params error, poses size: " << poses.size()
          << ", pose_index: " << pose_index;
    return true;
  }

  double yaw = GetYaw(poses[pose_index - 1].tx, poses[pose_index - 1].ty,
                      poses[pose_index].tx, poses[pose_index].ty);
  double yaw_diff = fabs(last_yaw_ - yaw);
  last_yaw_ = yaw;
  yaw_diff = yaw_diff < 180 ? yaw_diff : 360 - yaw_diff;

  double xdiff = poses[pose_index].tx - poses[pose_index - 1].tx;
  double ydiff = poses[pose_index].ty - poses[pose_index - 1].ty;
  double zdiff = poses[pose_index].tz - poses[pose_index - 1].tz;
  double dist = std::sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
  double during =
      poses[pose_index].time_stamp - poses[pose_index - 1].time_stamp;
  if (during < 0) {
    AINFO << "skip back pose is bad pose";
    return false;
  }
  double vel = dist / during;
  AINFO << poses[pose_index].time_stamp << ", yaw_diff:" << yaw_diff
        << ", dist: " << dist << ", during: " << during << ", vel: " << vel;
  if (yaw_diff > sp_conf_->eight_angle && vel > sp_conf_->eight_vel) {
    return true;
  }
  return false;
}

double EightRoute::GetGoodPoseDuring() {
  if (sp_good_pose_info_ == nullptr || sp_good_pose_info_->start_time < 0 ||
      sp_good_pose_info_->end_time < 0) {
    return 0.0;
  }
  return sp_good_pose_info_->end_time - sp_good_pose_info_->start_time;
}

double EightRoute::GetEightRouteProgress(const std::vector<FramePose>& poses) {
  int size = static_cast<int>(poses.size());
  int start_index = TimeToIndex(poses, start_time_);
  // select first good pose
  while (start_index < size) {
    if (IsGoodPose(poses, start_index) &&
        IsEightRoutePose(poses, start_index)) {
      AINFO << "find first good pose.index:" << start_index;
      break;
    }
    ++start_index;
  }
  if (start_index >= size) {
    AINFO << "not find first good pose, start_time: " << start_time_
          << ", start_index: " << start_index << ", pose size: " << size;
    return 0.0;
  }
  if (start_index + 1 >= size) {
    AINFO << "not have enough poses, wait for a moment";
    return 0.0;
  }
  last_yaw_ = GetYaw(poses[start_index].tx, poses[start_index].ty,
                     poses[start_index + 1].tx, poses[start_index + 1].ty);

  int not_eight_count = 0;
  for (int i = start_index + 2; i < size; ++i) {
    if (!IsGoodPose(poses, i)) {
      AINFO << "not good pose";
      return 0.0;
    }
    if (!IsEightRoutePose(poses, i)) {
      ++not_eight_count;
      AINFO << "not eight route pose";
      if (not_eight_count > sp_conf_->eight_bad_pose_tolerance) {
        AINFO << "not-eight pose count reached upper limitation";
        return_state_ = ErrorCode::ERROR_NOT_EIGHT_ROUTE;
        return 0.0;
      }
    } else {
      not_eight_count = 0;
    }
    AINFO << "good pose";
    UpdateGoodPoseInfo(poses[i]);
    //  ClearBadPoseInfo();
  }
  double eight_route_during = GetGoodPoseDuring();
  if (eight_route_during < 1e-8) {
    AINFO << "num of eight route good pose too small, during: "
          << eight_route_during;
    return_state_ = ErrorCode::SUCCESS;
    return 0.0;
  }
  return_state_ = ErrorCode::SUCCESS;
  double progress = eight_route_during / sp_conf_->eight_duration;
  if (progress >= 1.0) {
    progress = 1.0;
  }
  ClearGoodPoseInfo();
  return progress;
}

ErrorCode EightRoute::Process(const std::vector<FramePose>& poses) {
  AINFO << "[EightRoute::process] begin";
  size_t size = poses.size();
  if (size <= 1) {
    return_state_ = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
    return return_state_;
  }

  progress_ = GetEightRouteProgress(poses);
  if (return_state_ != ErrorCode::SUCCESS) {
    AINFO << "get_eight_route_progress failed.";
    return return_state_;
  }
  if (progress_ < last_progress_) {
    return_state_ = ErrorCode::ERROR_NOT_EIGHT_ROUTE;
    return return_state_;
  }

  AINFO << "[EightRoute::process] end, progress:" << progress_;
  return_state_ = ErrorCode::SUCCESS;
  return return_state_;
}

double EightRoute::GetProgress() const { return progress_; }

}  // namespace hdmap
}  // namespace apollo
