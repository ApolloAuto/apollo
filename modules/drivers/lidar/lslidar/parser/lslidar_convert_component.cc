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

#include "modules/drivers/lidar/lslidar/parser/lslidar_convert_component.h"

namespace apollo {
namespace drivers {
namespace lslidar {

static void my_handler(int sig) { exit(-1); }

bool LslidarConvertComponent::Init() {
  signal(SIGINT, my_handler);
  Config lslidar_config;
  if (!GetProtoConfig(&lslidar_config)) {
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  conv_.reset(new Convert());
  conv_->init(lslidar_config);

  writer_ = node_->CreateWriter<apollo::drivers::PointCloud>(
      lslidar_config.convert_channel_name());
  point_cloud_pool_.reset(
      new CCObjectPool<apollo::drivers::PointCloud>(pool_size_));
  point_cloud_pool_->ConstructAll();

  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(200000);
  }

  AINFO << "Point cloud comp convert init success";
  return true;
}

bool LslidarConvertComponent::Proc(
    const std::shared_ptr<apollo::drivers::lslidar::LslidarScan>& scan_msg) {
  std::shared_ptr<apollo::drivers::PointCloud> point_cloud_out =
      point_cloud_pool_->GetObject();
  if (point_cloud_out == nullptr) {
    AWARN << "poin cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<apollo::drivers::PointCloud>();
    point_cloud_out->mutable_point()->Reserve(200000);
  }
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return false;
  }

  uint64_t time1 = apollo::cyber::Time().Now().ToNanosecond();
  AINFO << "receive scan!---------";
  point_cloud_out->Clear();
  conv_->ConvertPacketsToPointcloud(scan_msg, point_cloud_out);
  uint64_t time2 = apollo::cyber::Time().Now().ToNanosecond();
  AINFO << "process pointcloud time: " << (time2 - time1) / 1000000000.0;

  if (point_cloud_out == nullptr || point_cloud_out->point().empty()) {
    AWARN << "point_cloud_out convert is empty.";
    return false;
  }

  writer_->Write(point_cloud_out);

  return true;
}

}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
