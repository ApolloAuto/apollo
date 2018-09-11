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

// #include <nodelet/nodelet.h>
// #include <pluginlib/class_list_macros.h>
// #include <ros/ros.h>
// #include <boost/thread.hpp>
#include <string>
#include <thread>
#include <memory>

#include <cybertron/cybertron.h>

#include "modules/drivers/velodyne/parser/velodyne_convert_component.h"

namespace apollo {
namespace drivers {
namespace velodyne {

bool VelodyneConvertComponent::Init() {
  Config velodyne_config;
  if(!apollo::cybertron::common::GetProtoFromFile(config_file_path_, &velodyne_config)){
    AWARN << "Load config failed, config file" << config_file_path_;
    return false;
  }

  conv_.reset(new Convert());
  conv_->init(velodyne_config);
  writer_ = node_->CreateWriter<PointCloud>(velodyne_config.convert_channel_name());
  for (int i = 0; i < queue_size_; ++i) {
    point_cloud_deque_.push_back(std::make_shared<PointCloud>());
    if (point_cloud_deque_[i] == nullptr) {
      AERROR << "point cloud fail to make shared.";
      return false;
    }
    point_cloud_deque_[i]->mutable_point()->Reserve(140000);
  }
  AINFO << "Point cloud comp convert init success";
  return true;
}

bool VelodyneConvertComponent::Proc(const std::shared_ptr<VelodyneScan>& scan_msg) {
  if (index_ >= queue_size_) {
    index_ = 0;
  }
  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_deque_[index_++];
  if (point_cloud_out == nullptr) {
    AWARN << "point cloud out is nullptr";
    return false;
  }
  point_cloud_out->Clear();
  conv_->convert_packets_to_pointcloud(scan_msg, point_cloud_out);

  if (point_cloud_out == nullptr || point_cloud_out->point_size() == 0) {
    AWARN << "point_cloud_out convert is empty.";
    return false;
  }
  writer_->Write(point_cloud_out);
  return true;

}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
