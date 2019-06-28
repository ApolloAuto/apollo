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
#include "modules/map/tools/map_datachecker/server/pose_collection_agent.h"

#include <vector>

#include "cyber/common/time_conversion.h"
#include "modules/map/tools/map_datachecker/server/pose_collection.h"

namespace apollo {
namespace hdmap {

PoseCollectionAgent::PoseCollectionAgent(std::shared_ptr<JSonConf> sp_conf) {
  sp_pj_transformer_ = std::make_shared<PJTransformer>(50);
  sp_conf_ = sp_conf;
  Reset();
}

void PoseCollectionAgent::Reset() {
  sp_pose_collection_ = std::make_shared<PoseCollection>(sp_conf_);
}

void PoseCollectionAgent::OnBestgnssposCallback(
    const std::shared_ptr<const apollo::drivers::gnss::GnssBestPose>
        &bestgnsspos) {
  if (sp_pose_collection_ == nullptr) {
    sp_pose_collection_ = std::make_shared<PoseCollection>(sp_conf_);
  }

  double time_stamp = apollo::cyber::common::GpsToUnixSeconds(
      bestgnsspos->measurement_time());  // in seconds
  FramePose pose;
  if (sp_conf_->use_system_time) {
    pose.time_stamp = UnixtimeNow();
    AINFO << "system time: " << std::to_string(pose.time_stamp);
  } else {
    pose.time_stamp = time_stamp;
  }
  pose.latitude = bestgnsspos->latitude();
  pose.longitude = bestgnsspos->longitude();
  pose.altitude = bestgnsspos->height_msl();
  pose.solution_status = bestgnsspos->sol_status();
  pose.position_type = bestgnsspos->sol_type();
  pose.diff_age = bestgnsspos->differential_age();
  double latitude_std = bestgnsspos->latitude_std_dev();
  double longitude_std = bestgnsspos->longitude_std_dev();
  double altitude_std = bestgnsspos->height_std_dev();
  pose.local_std =
      std::sqrt(latitude_std * latitude_std + longitude_std * longitude_std +
                altitude_std * altitude_std);
  pose.tx = pose.longitude * kDEGRESS_TO_RADIANS;
  pose.ty = pose.latitude * kDEGRESS_TO_RADIANS;
  pose.tz = pose.altitude;
  sp_pj_transformer_->LatlongToUtm(1, 1, &pose.tx, &pose.ty, &pose.tz);

  std::lock_guard<std::mutex> mutex_locker(mutex_);
  static FILE *pose_file = fopen("poses.txt", "w");
  static int count = 0;
  fprintf(stderr, "%d:%lf %lf %lf %lf 0.0 0.0 0.0 0.0\n", ++count,
          pose.time_stamp, pose.tx, pose.ty, pose.tz);
  fprintf(pose_file, "%lf %lf %lf %lf 0.0 0.0 0.0 0.0\n", pose.time_stamp,
          pose.tx, pose.ty, pose.tz);
  fflush(pose_file);
  sp_pose_collection_->Collect(pose);
}

std::shared_ptr<std::vector<FramePose>> PoseCollectionAgent::GetPoses() const {
  if (sp_pose_collection_ == nullptr) {
    return nullptr;
  }
  return sp_pose_collection_->GetPoses();
}

}  // namespace hdmap
}  // namespace apollo
