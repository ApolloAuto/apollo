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

#include <vector>
#include "modules/drivers/lidar/vanjeelidar/src/vanjeelidar_component.h"

#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/utility/buffer.hpp>

namespace apollo {
namespace drivers {
namespace lidar {

bool VanjeelidarComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }

  this->InitBase(conf_.config_base());

  cloud_buffer_ = std::make_shared<SyncBuffering<PointCloudMsg>>();
  cloud_buffer_->Init();

  cloud_handle_thread_ = std::thread(&VanjeelidarComponent::ProcessCloud, this);

  driver_ptr_ = std::make_shared<::vanjee::lidar::LidarDriver<PointCloudMsg>>();

  ::vanjee::lidar::WJDecoderParam decoder_param;
  decoder_param.publish_mode = conf_.publish_mode();
  decoder_param.min_distance = conf_.min_distance();
  decoder_param.max_distance = conf_.max_distance();
  decoder_param.start_angle = conf_.start_angle();
  decoder_param.end_angle = conf_.end_angle();
  decoder_param.use_lidar_clock = conf_.use_lidar_clock();
  decoder_param.dense_points = conf_.dense_points();
  decoder_param.wait_for_difop = conf_.wait_for_difop();
  decoder_param.config_from_file = conf_.config_from_file();
  decoder_param.angle_path_ver = conf_.angle_path();

  ::vanjee::lidar::WJInputParam input_param;
  input_param.connect_type = conf_.connect_type();
  input_param.host_msop_port = conf_.host_msop_port();
  input_param.lidar_msop_port = conf_.lidar_msop_port();
  input_param.host_address = conf_.host_address();
  input_param.lidar_address = conf_.lidar_address();

  ::vanjee::lidar::WJDriverParam driver_param;

  driver_param.input_param = input_param;
  driver_param.decoder_param = decoder_param;

  driver_param.lidar_type = ::vanjee::lidar::strToLidarType(conf_.model());
  if (conf_.config_base().source_type() ==
      LidarConfigBase_SourceType_RAW_PACKET) {
    driver_param.input_type = InputType::RAW_PACKET;
  } else if (conf_.config_base().source_type() ==
             LidarConfigBase_SourceType_ONLINE_LIDAR) {
    driver_param.input_type = InputType::ONLINE_LIDAR;
    driver_ptr_->regPacketCallback(
        std::bind(&VanjeelidarComponent::VanjeePacketCallback, this,
                  std::placeholders::_1));
  }

  driver_ptr_->regPointCloudCallback(
      std::bind(&VanjeelidarComponent::VanjeeCloudAllocateCallback, this),
      std::bind(&VanjeelidarComponent::VanjeeCloudPutCallback, this,
                std::placeholders::_1));
  driver_ptr_->regExceptionCallback([](const ::vanjee::lidar::Error& code) {
    WJ_WARNING << code.toString() << WJ_REND;
  });

  driver_param.print();

  if (!driver_ptr_->init(driver_param)) {
    AERROR << "vanjee Driver init failed";
    return false;
  }

  if (!driver_ptr_->start()) {
    AERROR << "vanjee Driver start failed";
    return false;
  }
  AINFO << "vanjee lidar init finished";
  return true;
}

void VanjeelidarComponent::ReadScanCallback(
    const std::shared_ptr<vanjee::VanjeeScanPacket>& scan_message) {
  ADEBUG << __FUNCTION__ << " start";
  std::vector<uint8_t> pkt_vector;
  pkt_vector.assign(scan_message->data().begin(), scan_message->data().end());
  std::shared_ptr<::vanjee::lidar::Buffer> pkt =
      std::make_shared<::vanjee::lidar::Buffer>(pkt_vector.size());
  memcpy(pkt->data(), pkt_vector.data(), pkt_vector.size());
  pkt->setData(0, pkt_vector.size());
  driver_ptr_->putPacket(pkt);
}

void VanjeelidarComponent::VanjeePacketCallback(
    const ::vanjee::lidar::Packet& lidar_packet) {
  ADEBUG << __FUNCTION__ << " start";
  std::shared_ptr<vanjee::VanjeeScanPacket> scan_packet =
      std::make_shared<vanjee::VanjeeScanPacket>();
  scan_packet->set_stamp(cyber::Time::Now().ToNanosecond());
  scan_packet->mutable_data()->assign(lidar_packet.buf_.begin(),
                                      lidar_packet.buf_.end());
  WriteScan(scan_packet);
}

std::shared_ptr<PointCloudMsg>
VanjeelidarComponent::VanjeeCloudAllocateCallback() {
  return cloud_buffer_->AllocateElement();
}

void VanjeelidarComponent::VanjeeCloudPutCallback(
    std::shared_ptr<PointCloudMsg> vanjee_cloud) {
  cloud_queue_.push(vanjee_cloud);
}

void VanjeelidarComponent::PreparePointsMsg(PointCloud& msg) {
  msg.set_height(1);
  msg.set_width(msg.point_size() / msg.height());

  const auto timestamp =
      msg.point(static_cast<int>(msg.point_size()) - 1).timestamp();
  msg.set_measurement_time(
      GetSecondTimestampFromNanosecondTimestamp(timestamp));
  double lidar_time = GetSecondTimestampFromNanosecondTimestamp(timestamp);
  double diff_time = msg.header().timestamp_sec() - lidar_time;

  if (diff_time > 0.02) {
    AINFO << std::fixed << std::setprecision(16)
          << "system time: " << msg.header().timestamp_sec()
          << ", lidar time: " << lidar_time << ", diff is:" << diff_time;
  }

  msg.mutable_header()->set_lidar_timestamp(timestamp);
}

void VanjeelidarComponent::ProcessCloud() {
  while (!cyber::IsShutdown()) {
    std::shared_ptr<PointCloudMsg> msg = cloud_queue_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    auto apollo_pc = AllocatePointCloud();

    for (auto p : msg->points) {
      PointXYZIT* point = apollo_pc->add_point();
      point->set_x(p.x);
      point->set_y(p.y);
      point->set_z(p.z);
      point->set_intensity(uint32_t(p.intensity));
      point->set_timestamp(GetNanosecondTimestampFromSecondTimestamp(
          p.timestamp + msg->timestamp));
    }
    apollo_pc->set_is_dense(msg->is_dense);
    this->PreparePointsMsg(*apollo_pc);
    if (apollo_pc->point_size() != 0) {
      WritePointCloud(apollo_pc);
    }
  }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
