/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "cyber/cyber.h"
#include "modules/drivers/lidar/robosense/parser/convert.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace robosense {
using apollo::cyber::Component;
class CompRoboConvert : public Component<apollo::drivers::suteng::SutengScan> {
 public:
  bool Init() {
    // read config file
    apollo::drivers::suteng::SutengConfig config;
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, &config)) {
      AERROR << "Failed to load config file";
      return false;
    }
    AINFO << "config:" << config.DebugString();

    conv_.reset(new Convert(config));
    if (!conv_->Init()) {
      return false;
    }

    writer_ =
        node_->CreateWriter<apollo::drivers::PointCloud>(config.pc_channel());

    uint32_t point_size = 28800;  // 32256;
    AINFO << "pc fixed size:" << point_size;
    point_cloud_deque_.resize(size_);
    for (uint i = 0; i < size_; ++i) {
      point_cloud_deque_[i] = std::make_shared<apollo::drivers::PointCloud>();
      if (point_cloud_deque_[i] == nullptr) {
        AERROR << " fail to make shared";
        return false;
      }
      point_cloud_deque_[i]->mutable_point()->Reserve(point_size);
    }
    AINFO << "CompRoboConvert Init SUCC"
          << ", frame_id:" << config.frame_id();
    return true;
  }

  bool Proc(const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan) {
    uint64_t start = apollo::cyber::Time().Now().ToNanosecond();
    if (index_ >= size_) {
      index_ = 0;
    }
    auto point_cloud_send = point_cloud_deque_.at(index_++);
    if (point_cloud_send == nullptr) {
      AINFO << "null point cloud";
      return false;
    }
    // just clear header now, we will reset all other value in pb.
    point_cloud_send->mutable_header()->Clear();
    point_cloud_send->Clear();

    conv_->convert_robosense_to_pointcloud(scan, point_cloud_send);
    if (point_cloud_send == nullptr || point_cloud_send->point_size() == 0) {
      AINFO << "discard null point cloud";
      return false;
    }
    uint64_t diff = apollo::cyber::Time().Now().ToNanosecond() - start;

    point_cloud_send->mutable_header()->set_sequence_num(seq_);
    writer_->Write(point_cloud_send);

    if (seq_ % 10 == 0) {
      AINFO << " total:" << seq_ + 1 << "-RS-cost:" << diff
            << "-meta:" << point_cloud_send->header().lidar_timestamp();
    }
    seq_++;

    return true;
  }

 private:
  std::unique_ptr<Convert> conv_ = nullptr;
  std::deque<std::shared_ptr<apollo::drivers::PointCloud>> point_cloud_deque_;
  uint64_t index_ = 0;
  uint64_t seq_ = 0;
  uint64_t size_ = 8;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> writer_;
};

CYBER_REGISTER_COMPONENT(CompRoboConvert);
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo