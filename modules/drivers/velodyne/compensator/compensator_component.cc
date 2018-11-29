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

#include "modules/drivers/velodyne/compensator/compensator_component.h"
#include "modules/drivers/velodyne/proto/velodyne.pb.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool CompensatorComponent::Init() {
  CompensatorConfig config;
  if (!GetProtoConfig(&config)) {
    AWARN << "Load config failed, config file" << ConfigFilePath();
    return false;
  }

  writer_ = node_->CreateWriter<PointCloud>(config.output_channel());

  compensator_.reset(new Compensator(config));
  compensator_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  compensator_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; ++i) {
    auto point_cloud = compensator_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject:" << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(140000);
  }
  return true;
}

bool CompensatorComponent::Proc(
    const std::shared_ptr<PointCloud>& point_cloud) {
  uint64_t start = cyber::Time().Now().ToNanosecond();
  std::shared_ptr<PointCloud> point_cloud_compensated =
      compensator_pool_->GetObject();
  if (point_cloud_compensated == nullptr) {
    AWARN << "compensator fail to getobject, will be new";
    point_cloud_compensated = std::make_shared<PointCloud>();
    point_cloud_compensated->mutable_point()->Reserve(140000);
  }
  if (point_cloud_compensated == nullptr) {
    AWARN << "compensator point_cloud is nullptr";
    return false;
  }
  point_cloud_compensated->Clear();
  if (compensator_->MotionCompensation(point_cloud, point_cloud_compensated)) {
    uint64_t diff = cyber::Time().Now().ToNanosecond() - start;
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
