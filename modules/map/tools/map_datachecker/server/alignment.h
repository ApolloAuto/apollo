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
#pragma once

#include <memory>
#include <vector>

#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"
#include "modules/map/tools/map_datachecker/server/common.h"

namespace apollo {
namespace hdmap {

typedef struct BadOrGoodPoseInfo {
  BadOrGoodPoseInfo() : start_time(-1.0), end_time(-1.0), pose_count(0) {}
  double start_time;
  double end_time;
  int pose_count;
} BadOrGoodPoseInfo;

class Alignment {
 public:
  explicit Alignment(std::shared_ptr<JSonConf> sp_conf)
      : return_state_(ErrorCode::SUCCESS),
        sp_conf_(sp_conf),
        sp_good_pose_info_(std::make_shared<BadOrGoodPoseInfo>()),
        sp_bad_pose_info_(std::make_shared<BadOrGoodPoseInfo>()) {}

  virtual ~Alignment() {}
  virtual ErrorCode Process(const std::vector<FramePose>& poses) = 0;
  virtual void Reset() = 0;

  virtual double GetProgress() const { return progress_; }

  virtual void SetStartTime(double start_time) { start_time_ = start_time; }

  virtual void SetEndTime(double end_time) { end_time_ = end_time; }

  virtual void UpdateBadPoseInfo(const FramePose& pose) {
    UpdatePoseInfo(pose, sp_bad_pose_info_);
  }

  virtual void ClearBadPoseInfo() { ClearPoseInfo(sp_bad_pose_info_); }

  virtual void UpdateGoodPoseInfo(const FramePose& pose) {
    UpdatePoseInfo(pose, sp_good_pose_info_);
  }

  virtual void ClearGoodPoseInfo() { ClearPoseInfo(sp_good_pose_info_); }

  virtual bool IsGoodPose(const std::vector<FramePose>& poses, int pose_index) {
    if (pose_index <= 0 || pose_index >= static_cast<int>(poses.size())) {
      AINFO << "params error. poses size:" << poses.size()
            << ",pose_index:" << pose_index;
      return false;
    }

    unsigned int position_type = poses[pose_index].position_type;
    float diff_age = poses[pose_index].diff_age;
    double local_std = poses[pose_index].local_std;

    if (sp_conf_->position_type_range.find(position_type) !=
            sp_conf_->position_type_range.end() &&
        diff_age >= sp_conf_->diff_age_range.first &&
        diff_age <= sp_conf_->diff_age_range.second &&
        local_std <= sp_conf_->local_std_upper_limit) {
      return true;
    }
    return false;
  }

  ErrorCode GetReturnState() const { return return_state_; }

 protected:
  void UpdatePoseInfo(const FramePose& pose,
                      std::shared_ptr<BadOrGoodPoseInfo> sp_pose_info) {
    if (sp_pose_info == nullptr) {
      AERROR << "sp_pose_info is nullptr";
      return;
    }
    BadOrGoodPoseInfo& pose_info = *sp_pose_info;
    if (pose_info.pose_count == 0) {
      pose_info.start_time = pose.time_stamp;
      ++pose_info.pose_count;
      AINFO << "update start time: " << pose_info.start_time
            << ",pose count: " << pose_info.pose_count;
    } else {
      pose_info.end_time = pose.time_stamp;
      ++pose_info.pose_count;
      AINFO << "update start time: " << pose_info.start_time
            << ",pose count: " << pose_info.pose_count;
    }
  }

  void ClearPoseInfo(std::shared_ptr<BadOrGoodPoseInfo> sp_pose_info) {
    if (sp_pose_info == nullptr) {
      AERROR << "sp_pose_info is nullptr";
      return;
    }
    BadOrGoodPoseInfo& pose_info = *sp_pose_info;
    pose_info.start_time = -1.0;
    pose_info.end_time = -1.0;
    pose_info.pose_count = 0;
  }

  int TimeToIndex(const std::vector<FramePose>& poses, double time) {
    size_t size = poses.size();
    if (size == 0 || time <= 0) {
      return -1;
    }

    for (size_t i = 0; i < size; ++i) {
      if (poses[i].time_stamp >= time) {
        return static_cast<int>(i);
      }
    }
    return static_cast<int>(size);
  }

 protected:
  double progress_;
  double last_progress_;
  double start_time_;
  double end_time_;
  int start_index_;
  double end_index_;
  ErrorCode return_state_;
  std::shared_ptr<JSonConf> sp_conf_ = nullptr;
  // BadOrGoodPoseInfo _bad_pose_info, _good_pose_info;
  std::shared_ptr<BadOrGoodPoseInfo> sp_good_pose_info_ = nullptr;
  std::shared_ptr<BadOrGoodPoseInfo> sp_bad_pose_info_ = nullptr;
};

}  // namespace hdmap
}  // namespace apollo
