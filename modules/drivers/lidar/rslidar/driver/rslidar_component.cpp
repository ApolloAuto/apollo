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

#include "modules/drivers/lidar/rslidar/driver/rslidar_component.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool RslidarComponent::Init() {
    if (!GetProtoConfig(&conf_)) {
        AERROR << "load config error, file:" << config_file_path_;
        return false;
    }

    this->InitBase(conf_.config_base());

    cloud_buffer_ = std::make_shared<SyncBuffering<PointCloudMsg>>();
    cloud_buffer_->Init();
    cloud_handle_thread_ = std::thread(&RslidarComponent::ProcessCloud, this);

    driver_ptr_ = std::make_shared<
            ::robosense::lidar::LidarDriver<PointCloudMsg>>();

    ::robosense::lidar::RSDecoderParam decoder_param;
    decoder_param.min_distance = conf_.min_distance();
    decoder_param.max_distance = conf_.max_distance();
    decoder_param.start_angle = conf_.start_angle();
    decoder_param.end_angle = conf_.end_angle();
    decoder_param.use_lidar_clock = conf_.use_lidar_clock();
    decoder_param.num_blks_split = conf_.num_pkts_split();
    decoder_param.dense_points = conf_.dense_points();
    decoder_param.ts_first_point = conf_.ts_first_point();
    decoder_param.wait_for_difop = conf_.wait_for_difop();
    decoder_param.config_from_file = conf_.config_from_file();
    decoder_param.angle_path = conf_.angle_path();
    decoder_param.split_angle = conf_.split_angle();

    ::robosense::lidar::RSInputParam input_param;
    input_param.msop_port = conf_.msop_port();
    input_param.difop_port = conf_.difop_port();
    input_param.host_address = conf_.host_address();
    input_param.group_address = conf_.group_address();
    input_param.use_vlan = conf_.use_vlan();
    input_param.user_layer_bytes = conf_.user_layer_bytes();
    input_param.tail_layer_bytes = conf_.tail_layer_bytes();

    ::robosense::lidar::RSDriverParam driver_param;
    driver_param.input_param = input_param;
    driver_param.decoder_param = decoder_param;
    driver_param.lidar_type = ::robosense::lidar::strToLidarType(conf_.model());

    if (conf_.config_base().source_type()
        == LidarConfigBase_SourceType_RAW_PACKET) {
        driver_param.input_type = InputType::RAW_PACKET;
    } else if (
            conf_.config_base().source_type()
            == LidarConfigBase_SourceType_ONLINE_LIDAR) {
        driver_param.input_type = InputType::ONLINE_LIDAR;
        driver_ptr_->regPacketCallback(std::bind(
                &RslidarComponent::RsPacketCallback,
                this,
                std::placeholders::_1));
    }

    driver_ptr_->regPointCloudCallback(
            std::bind(&RslidarComponent::RsCloudAllocateCallback, this),
            std::bind(
                    &RslidarComponent::RsCloudPutCallback,
                    this,
                    std::placeholders::_1));
    driver_ptr_->regExceptionCallback(
            [](const ::robosense::lidar::Error& code) {
                RS_WARNING << code.toString() << RS_REND;
            });
    driver_param.print();
    if (!driver_ptr_->init(driver_param)) {
        AERROR << "Robosense Driver init failed";
        return false;
    }

    if (!driver_ptr_->start()) {
        AERROR << "Robosense Driver start failed";
        return false;
    }
    AINFO << "rslidar init finished";
    return true;
}

void RslidarComponent::ReadScanCallback(
        const std::shared_ptr<robosense::RobosenseScanPacket>& scan_message) {
    ADEBUG << __FUNCTION__ << " start";
    std::shared_ptr<::robosense::lidar::Packet> scan_packet
            = std::make_shared<::robosense::lidar::Packet>();
    scan_packet->buf_.assign(
            scan_message->data().begin(), scan_message->data().end());
    driver_ptr_->decodePacket(*scan_packet);
}

void RslidarComponent::RsPacketCallback(
        const ::robosense::lidar::Packet& lidar_packet) {
    ADEBUG << __FUNCTION__ << " start";
    auto scan_packet = AcquireScanMessage();
    scan_packet->set_stamp(cyber::Time::Now().ToNanosecond());
    scan_packet->mutable_data()->assign(
            lidar_packet.buf_.begin(), lidar_packet.buf_.end());
    WriteScan(scan_packet);
}

std::shared_ptr<PointCloudMsg> RslidarComponent::RsCloudAllocateCallback() {
    return cloud_buffer_->AllocateElement();
}

void RslidarComponent::RsCloudPutCallback(
        std::shared_ptr<PointCloudMsg> rs_cloud) {
    cloud_queue_.push(rs_cloud);
}

void RslidarComponent::PreparePointsMsg(PointCloud& msg) {
    msg.set_height(1);
    msg.set_width(msg.point_size() / msg.height());
    msg.set_is_dense(false);
    const auto timestamp
            = msg.point(static_cast<int>(msg.point_size()) - 1).timestamp();
    msg.set_measurement_time(
            GetSecondTimestampFromNanosecondTimestamp(timestamp));

    double lidar_time = GetSecondTimestampFromNanosecondTimestamp(timestamp);
    double diff_time = msg.header().timestamp_sec() - lidar_time;
    if (diff_time > 0.2) {
        AINFO << std::fixed << std::setprecision(16)
              << "system time: " << msg.header().timestamp_sec()
              << ", lidar time: " << lidar_time << ", diff is:" << diff_time;
    }

    if (conf_.use_lidar_clock()) {
        msg.mutable_header()->set_lidar_timestamp(timestamp);
    } else {
        msg.mutable_header()->set_lidar_timestamp(
                cyber::Time().Now().ToNanosecond());
    }
}

void RslidarComponent::ProcessCloud() {
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
            point->set_timestamp(
                    GetNanosecondTimestampFromSecondTimestamp(p.timestamp));
        }

        this->PreparePointsMsg(*apollo_pc);
        if (apollo_pc->point_size() != 0) {
            WritePointCloud(apollo_pc);
        }
    }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
