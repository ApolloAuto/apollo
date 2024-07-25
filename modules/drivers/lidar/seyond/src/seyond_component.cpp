/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/seyond/src/seyond_component.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool SeyondComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }

  driver_ptr_ = std::make_shared<SeyondDriver>();
  this->InitBase(conf_.config_base());

  SeyondParam driver_param;
  driver_param.device_ip = conf_.device_ip();
  driver_param.port = conf_.port();
  driver_param.udp_port = conf_.udp_port();
  driver_param.reflectance_mode = conf_.reflectance_mode();
  driver_param.multiple_return = conf_.multiple_return();
  driver_param.coordinate_mode = conf_.coordinate_mode();
  driver_param.max_range = conf_.max_range();
  driver_param.min_range = conf_.min_range();
  driver_param.log_level = conf_.log_level();

  if (conf_.config_base().source_type() ==
      LidarConfigBase_SourceType_RAW_PACKET) {
    driver_param.raw_packets_mode = true;
  }

  driver_ptr_->register_publish_packet_callback(
        std::bind(&SeyondComponent::SeyondPacketCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

  driver_ptr_->register_publish_point_callback(
        std::bind(&SeyondComponent::PointCloudCallback, this));

  driver_param.print();

  scan_packets_ptr_ = std::make_shared<seyond::SeyondScan>();
  driver_ptr_->point_cloud_ptr_ = AllocatePointCloud();

  if (!driver_ptr_->init(driver_param)) {
    AERROR << "seyond Driver init failed";
    return false;
  }

  if (!driver_ptr_->start()) {
    AERROR << "seyond Driver start failed";
    return false;
  }
  AINFO << "seyond lidar init finished";
  return true;
}

void SeyondComponent::ReadScanCallback(
  const std::shared_ptr<seyond::SeyondScan>& scan_message) {
  driver_ptr_->process_scan_packet_(scan_message);
}

void SeyondComponent::PointCloudCallback() {
  WritePointCloud(driver_ptr_->point_cloud_ptr_);
  driver_ptr_->point_cloud_ptr_ = AllocatePointCloud();
}

void SeyondComponent::SeyondPacketCallback(const InnoDataPacket *pkt,
                                           bool is_next_frame) {
  if (is_next_frame) {
    frame_count_++;
    scan_packets_ptr_->set_timestamp(
        (static_cast<uint64_t>(pkt->common.ts_start_us)) * 1000);
    scan_packets_ptr_->set_measurement_time(pkt->common.ts_start_us * 1e-6);
    WriteScan(scan_packets_ptr_);
    scan_packets_ptr_ = std::make_shared<seyond::SeyondScan>();
  }
  seyond::SeyondPacket *scan_packet = scan_packets_ptr_->add_packets();
  uint64_t pkt_len = sizeof(InnoDataPacket) + pkt->item_number * pkt->item_size;
  scan_packet->mutable_data()->assign(reinterpret_cast<const char *>(pkt),
                                      pkt_len);
  scan_packet->set_table_exist(false);
  if (driver_ptr_->anglehv_table_init_ && (frame_count_ == table_send_hz_)) {
    frame_count_ = 0;
    scan_packet->set_table_exist(true);
    scan_packet->mutable_table()->assign(driver_ptr_->anglehv_table_.data(),
                                         driver_ptr_->anglehv_table_.size());
  }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
