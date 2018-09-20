/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <memory>

#include "modules/drivers/velodyne/parser/velodyne_compensator_component.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool VelodyneCompensatorComponent::Init() {
  Config velodyne_config;
  if (!GetProtoConfig(&velodyne_config)) {
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  writer_ = node_->CreateWriter<PointCloud>(
      velodyne_config.compensator_channel_name());

  _compensator.reset(new Compensator(velodyne_config));

  for (int i = 0; i < queue_size_; ++i) {
    compensator_deque_.push_back(std::make_shared<PointCloud>());
    if (compensator_deque_[i] == nullptr) {
      AERROR << "fail to make shared:" << i;
      return false;
    }
    compensator_deque_[i]->mutable_point()->Reserve(140000);
  }
  return true;
}

bool VelodyneCompensatorComponent::Proc(
    const std::shared_ptr<PointCloud>& point_cloud) {
  uint64_t start = cybertron::Time().Now().ToNanosecond();
  if (index_ >= queue_size_) {
    index_ = 0;
  }
  std::shared_ptr<PointCloud> point_cloud_compensated =
      compensator_deque_[index_++];
  point_cloud_compensated->Clear();
  if (_compensator->MotionCompensation(point_cloud, point_cloud_compensated)) {
    uint64_t diff = cybertron::Time().Now().ToNanosecond() - start;
    AINFO << "compenstator diff:" << diff
          << ";meta:" << point_cloud_compensated->header().lidar_timestamp();
    point_cloud_compensated->mutable_header()->set_sequence_num(seq_);
    writer_->Write(point_cloud_compensated);
    seq_++;
  }

  return true;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
