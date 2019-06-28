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
#include "modules/map/tools/map_datachecker/server/static_align.h"

#include <vector>

namespace apollo {
namespace hdmap {

StaticAlign::StaticAlign(std::shared_ptr<JSonConf> sp_conf)
    : Alignment(sp_conf) {
  sp_conf_ = sp_conf;
  static_align_detect_method_ = StaticAlignDetectMethod::DYNAMIC_CENTROID;
  Reset();
}

void StaticAlign::Reset() {
  progress_ = 0.0;
  last_progress_ = 0.0;
  start_time_ = -1.0;
  end_time_ = -1.0;
  start_index_ = -1;
  end_index_ = -1;
  sp_bad_pose_info_ = std::make_shared<BadOrGoodPoseInfo>();
  sp_good_pose_info_ = std::make_shared<BadOrGoodPoseInfo>();
  dynamic_centroid_ = Centroid3D();
}

bool StaticAlign::IsStaticPose(const FramePose& pose) {
  if (dynamic_centroid_.count == 0) {
    return true;
  }
  double move_dist_x = pose.tx - dynamic_centroid_.center.x;
  double move_dist_y = pose.ty - dynamic_centroid_.center.y;
  double move_dist_z = pose.tz - dynamic_centroid_.center.z;
  double move_dist =
      std::sqrt(move_dist_x * move_dist_x + move_dist_y * move_dist_y +
                move_dist_z * move_dist_z);
  AINFO << "dist thresh: " << sp_conf_->static_align_dist_thresh
        << ", dist: " << move_dist;
  if (move_dist <= sp_conf_->static_align_dist_thresh) {
    return true;
  }
  return false;
}

void StaticAlign::UpdateDynamicCentroid(const FramePose& pose) {
  int count = dynamic_centroid_.count;
  if (count == 0) {
    dynamic_centroid_.start_time = pose.time_stamp;
  } else {
    dynamic_centroid_.end_time = pose.time_stamp;
  }
  AINFO << "cetroid start: " << dynamic_centroid_.start_time
        << ", end: " << dynamic_centroid_.end_time;

  double x = dynamic_centroid_.center.x * count + pose.tx;
  double y = dynamic_centroid_.center.y * count + pose.ty;
  double z = dynamic_centroid_.center.z * count + pose.tz;
  ++count;

  dynamic_centroid_.count = count;
  dynamic_centroid_.center.x = x / count;
  dynamic_centroid_.center.y = y / count;
  dynamic_centroid_.center.z = z / count;
}

double StaticAlign::GetCentroidTimeDuring() {
  if (dynamic_centroid_.start_time > 0 && dynamic_centroid_.end_time > 0) {
    return dynamic_centroid_.end_time - dynamic_centroid_.start_time;
  }
  return 0.0;
}

void StaticAlign::UpdateGoodPoseInfo(const FramePose& pose) {
  UpdateDynamicCentroid(pose);
}

double StaticAlign::StaticAlignDynamicCentroid(
    const std::vector<FramePose>& poses) {
  int start_index = TimeToIndex(poses, start_time_);
  AINFO << "start_index:" << start_index << ",pose size:" << poses.size();
  dynamic_centroid_ = Centroid3D();
  for (int i = start_index + 1; i < static_cast<int>(poses.size()); ++i) {
    if (!IsGoodPose(poses, i)) {
      AINFO << "not good pose";
      return_state_ = ErrorCode::ERROR_GNSS_SIGNAL_FAIL;
      return 0.0;
    }
    if (!IsStaticPose(poses[i])) {
      AINFO << "not static pose";
      return_state_ = ErrorCode::ERROR_NOT_STATIC;
      return 0.0;
    }
    UpdateGoodPoseInfo(poses[i]);
    return_state_ = ErrorCode::SUCCESS;
  }

  double progress = GetCentroidTimeDuring() / sp_conf_->static_align_duration;
  if (progress > 1.0) {
    progress = 1.0;
  }
  return progress;
}

double StaticAlign::StaticAlignRansac(const std::vector<FramePose>& poses) {
  // TODO(yuanyijun): implementation of selecting an center by RANSAC
  return 0.0;
}

double StaticAlign::GetStaticAlignProgress(
    const std::vector<FramePose>& poses) {
  double progress = 0.0;
  switch (static_align_detect_method_) {
    case StaticAlignDetectMethod::DYNAMIC_CENTROID:
      progress = StaticAlignDynamicCentroid(poses);
      break;
    case StaticAlignDetectMethod::RANSAC:
      progress = StaticAlignRansac(poses);
      break;
    default:
      break;
  }
  ClearGoodPoseInfo();
  return progress;
}

ErrorCode StaticAlign::Process(const std::vector<FramePose>& poses) {
  AINFO << "[StaticAlign::process] begin";
  size_t size = poses.size();
  if (size <= 1) {
    AINFO << "system has no pose, exit process";
    return_state_ = ErrorCode::ERROR_VERIFY_NO_GNSSPOS;
    return return_state_;
  }

  progress_ = GetStaticAlignProgress(poses);
  if (return_state_ != ErrorCode::SUCCESS) {
    AINFO << "get_static_align_progress error, progress 0.0";
    return return_state_;
  }

  AINFO << "[StaticAlign::process] end, progress:" << progress_;
  return_state_ = ErrorCode::SUCCESS;
  return return_state_;
}

}  // namespace hdmap
}  // namespace apollo
