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
#include "modules/drivers/lidar/seyond/src/seyond_driver.h"

static const uint32_t KBUF_SIZE = 1024 * 1024 * 10;
static const double us_in_second_c = 1000000.0;
static const double ten_us_in_second_c = 100000.0;
namespace apollo {
namespace drivers {
namespace lidar {

static void coordinate_transfer(PointXYZIT *point, uint32_t coordinate_mode,
                                float x, float y, float z) {
  switch (coordinate_mode) {
    case 0:
      point->set_x(x);  // up
      point->set_y(y);  // right
      point->set_z(z);  // forward
      break;
    case 1:
      point->set_x(y);  // right
      point->set_y(z);  // forward
      point->set_z(x);  // up
      break;
    case 2:
      point->set_x(y);  // right
      point->set_y(x);  // up
      point->set_z(z);  // forward
      break;
    case 3:
      point->set_x(z);   // forward
      point->set_y(-y);  // -right
      point->set_z(x);   // up
      break;
    case 4:
      point->set_x(z);  // forward
      point->set_y(x);  // up
      point->set_z(y);  // right
      break;
    default:
      point->set_x(x);  // up
      point->set_y(y);  // right
      point->set_z(z);  // forward
      break;
  }
}

SeyondDriver::SeyondDriver() {
  convert_buffer_.resize(KBUF_SIZE);
}

SeyondDriver::~SeyondDriver() {
  stop();
}

bool SeyondDriver::setup_lidar() {
  // setup read from live
  const char *ip_str = param_.device_ip.c_str();
  uint16_t lidar_udp_port = 0;
  enum InnoLidarProtocol protocol_;
  if (param_.udp_port >= 0) {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;
    lidar_udp_port = param_.udp_port;
  } else {
    protocol_ = INNO_LIDAR_PROTOCOL_PCS_TCP;
  }
  handle_ = inno_lidar_open_live(lidar_name_.c_str(), ip_str, param_.port,
                                 protocol_, lidar_udp_port);
  // get internal pcs enable status
  uint32_t is_internal_pcs = 1;
  char buf[20] = {'\0'};
  int32_t ret =
      inno_lidar_get_attribute_string(handle_, "enabled", buf, sizeof(buf));
  if (ret != 0) {
    inno_log_error("cannot connect to lidar, please check the network");
    return false;
  }
  std::string enable(buf);
  is_internal_pcs = atoi(buf);
  if (is_internal_pcs != 1) {
    inno_log_error("internal pcs: %u, please turn on the pcs", is_internal_pcs);
    return false;
  }
  std::string cilent_set_key = "";
  cilent_set_key = "LidarClient_Communication/get_conn_timeout_sec";
  ret =
      inno_lidar_set_config_name_value(handle_, cilent_set_key.c_str(), "5.0");
  if (ret != 0) {
    inno_log_error("set lidar key %s failed %d", cilent_set_key.c_str(), ret);
  }

  // set misorder_correct_enable to 1, in case of point packet disorder
  cilent_set_key = "LidarClient_StageClientRead/misorder_correct_enable";
  ret = inno_lidar_set_config_name_value(handle_, cilent_set_key.c_str(), "1");
  if (ret != 0) {
    inno_log_error("set lidar key %s failed %d", cilent_set_key.c_str(), ret);
  }

  // set lidar reflectance mode
  enum InnoReflectanceMode m = param_.reflectance_mode
                                   ? INNO_REFLECTANCE_MODE_REFLECTIVITY
                                   : INNO_REFLECTANCE_MODE_INTENSITY;
  ret = inno_lidar_set_reflectance_mode(handle_, m);
  if (ret != 0) {
    inno_log_error("Lidar %s cannot set reflectance mode", lidar_name_.c_str());
  }

  // set lidar return mode
  ret = inno_lidar_set_return_mode(
      handle_, static_cast<InnoMultipleReturnMode>(param_.multiple_return));
  if (ret != 0) {
    inno_log_error("Lidar %s cannot set return mode", lidar_name_.c_str());
  }

  // set lidar callback
  ret = inno_lidar_set_callbacks(handle_, message_callback_s_, data_callback_s_,
                                 status_callback_s_, nullptr, this);
  if (ret != 0) {
    inno_log_error("Lidar %s  set callbacks error", lidar_name_.c_str());
    return false;
  }

  return true;
}

bool SeyondDriver::init(SeyondParam& param) {
  param_ = param;
  set_log_and_message_level();
  inno_lidar_setup_sig_handler();
  inno_lidar_set_logs(-1, -1, NULL, 0, 0, log_callback_s_, this, NULL, 0, 0, 1);
  return true;
}

bool SeyondDriver::start() {
  if (handle_ <= 0) {
    if (param_.raw_packets_mode) {
      inno_log_info("waiting for record to replay...");
      return true;
    }
    if (!setup_lidar()) {
      return false;
    }
  }
  inno_lidar_start(handle_);
  return true;
}

bool SeyondDriver::pause() {
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
  }
  return true;
}

bool SeyondDriver::stop() {
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
    inno_lidar_close(handle_);
  }
  handle_ = 0;
  current_frame_id_ = -1;
  return true;
}

// callback group
void SeyondDriver::message_callback_(uint32_t from_remote,
                                     enum InnoMessageLevel level,
                                     enum InnoMessageCode code,
                                     const char *msg) {
  const char *remote = "";
  if (from_remote) {
    remote = "REMOTE-";
  }

  if (level == INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_warning("%s%s level=%d, code=%d, message=%s", remote,
                     inno_log_header_g[level], level, code, msg);
  } else if (level < INNO_MESSAGE_LEVEL_WARNING) {
    inno_log_error("%s%s level=%d, code=%d, message=%s", remote,
                   inno_log_header_g[level], level, code, msg);
  }
}

int32_t SeyondDriver::status_callback_(const InnoStatusPacket *pkt) {
  // sanity check
  if (!inno_lidar_check_status_packet(pkt, 0)) {
    inno_log_error("corrupted pkt->idx = %" PRI_SIZEU, pkt->idx);
    return -1;
  }

  static uint64_t cnt = 0;
  if (cnt++ % 100 == 1) {
    constexpr uint64_t buf_size = 2048;
    char buf[buf_size]{0};

    int32_t ret = inno_lidar_printf_status_packet(pkt, buf, buf_size);
    if ((ret > 0)) {
      inno_log_info("Received status packet %u: %s", cnt, buf);
    } else if ((ret < 0)) {
      inno_log_warning("Received status packet %u: errorno: %d", cnt, ret);
    }
  }
  return 0;
}

void SeyondDriver::log_callback_(enum InnoLogLevel level, const char *header2,
                                 const char *msg) {
  switch (level) {
    case INNO_LOG_LEVEL_FATAL:
    case INNO_LOG_LEVEL_CRITICAL:
      AERROR << header2 << " " << msg;
      break;
    case INNO_LOG_LEVEL_ERROR:
    case INNO_LOG_LEVEL_TEMP:
      AERROR << header2 << " " << msg;
      break;
    case INNO_LOG_LEVEL_WARNING:
    case INNO_LOG_LEVEL_DEBUG:
      AWARN << header2 << " " << msg;
      break;
    case INNO_LOG_LEVEL_INFO:
      AINFO << header2 << " " << msg;
      break;
    case INNO_LOG_LEVEL_TRACE:
    case INNO_LOG_LEVEL_DETAIL:
    default:
      AINFO << header2 << " " << msg;
  }
}

int32_t SeyondDriver::data_callback_(const InnoDataPacket *pkt) {
  // get first frame id
  if (current_frame_id_ == -1) {
    current_frame_id_ = pkt->idx;
    inno_log_info("get first frame id: %lu", current_frame_id_);
    return 0;
  }

  if ((pkt->type == INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD) &&
      (!anglehv_table_init_)) {
    anglehv_table_.resize(sizeof(InnoAngleHVTable) + sizeof(InnoDataPacket));
    int ret = inno_lidar_get_anglehv_table(
        handle_, reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    if (ret == 0) {
      anglehv_table_init_ = true;
      inno_log_info("Get RobinW Compact Table");
    }
  }
  bool is_next_frame = false;
  if (current_frame_id_ != static_cast<int64_t>(pkt->idx)) {
    is_next_frame = true;
  }

  packet_publish_cb_(pkt, is_next_frame);

  if (is_next_frame) {
    point_cloud_ptr_->mutable_header()->set_lidar_timestamp(
        (static_cast<uint64_t>(pkt->common.ts_start_us)) * 1000);
    point_cloud_ptr_->set_measurement_time(pkt->common.ts_start_us * 1e-6);
    point_cloud_ptr_->set_height(1);
    point_cloud_ptr_->set_width(frame_points_width_);
    // data publish
    point_publish_cb_();
    current_frame_id_ = pkt->idx;
    frame_points_width_ = 0;
  }
  convert_and_parse_(pkt);

  return 0;
}

int32_t SeyondDriver::process_scan_packet_(
    const std::shared_ptr<seyond::SeyondScan> &lidar_packets) {
  if (current_frame_id_ == -1) {
    current_frame_id_ = 0;
    inno_log_info("received first scan packet...");
    return 0;
  }
  for (auto &packet : lidar_packets->packets()) {
    const InnoDataPacket *pkt =
        reinterpret_cast<const InnoDataPacket *>(packet.data().data());
    if ((param_.raw_packets_mode) && (!anglehv_table_init_) &&
        (pkt->type == INNO_ROBINW_ITEM_TYPE_COMPACT_POINTCLOUD)) {
      if (packet.table_exist()) {
        anglehv_table_init_ = true;
        anglehv_table_.resize(sizeof(InnoAngleHVTable) +
                              sizeof(InnoDataPacket));
        std::memcpy(anglehv_table_.data(), packet.table().data(),
                    sizeof(InnoAngleHVTable) + sizeof(InnoDataPacket));
        inno_log_info("Get RobinW Compact Table");
      } else {
        return 0;
      }
    }
    convert_and_parse_(pkt);
  }
  point_cloud_ptr_->mutable_header()->set_lidar_timestamp(
      lidar_packets->timestamp());
  point_cloud_ptr_->set_measurement_time(lidar_packets->measurement_time());
  point_cloud_ptr_->set_height(1);
  point_cloud_ptr_->set_width(frame_points_width_);
  frame_points_width_ = 0;
  point_publish_cb_();

  return 0;
}

void SeyondDriver::convert_and_parse_(const InnoDataPacket *pkt) {
  if (CHECK_SPHERE_POINTCLOUD_DATA(pkt->type)) {
    // convert sphere to xyz
    if (anglehv_table_init_) {
      inno_lidar_convert_to_xyz_pointcloud2(
          pkt, reinterpret_cast<InnoDataPacket *>(&convert_buffer_[0]),
          convert_buffer_.size(), false,
          reinterpret_cast<InnoDataPacket *>(anglehv_table_.data()));
    } else {
      inno_lidar_convert_to_xyz_pointcloud(
          pkt, reinterpret_cast<InnoDataPacket *>(&convert_buffer_[0]),
          convert_buffer_.size(), false);
    }
    process_data_packet_(
        reinterpret_cast<InnoDataPacket *>(&convert_buffer_[0]));
  } else if (CHECK_XYZ_POINTCLOUD_DATA(pkt->type)) {
    process_data_packet_(pkt);
  } else {
    inno_log_error("pkt type %d is not supported", pkt->type);
  }
}

int32_t SeyondDriver::process_data_packet_(const InnoDataPacket *pkt) {
  // calculate the point timestamp
  current_ts_start_ = pkt->common.ts_start_us / us_in_second_c;
  // adapt different data structures form different lidar
  if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt->type)) {
    const InnoEnXyzPoint *pt = reinterpret_cast<const InnoEnXyzPoint *>(
        reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    process_xyz_point_data_<const InnoEnXyzPoint *>(true, pkt->use_reflectance,
                                                    pkt->item_number, pt);
  } else {
    const InnoXyzPoint *pt = reinterpret_cast<const InnoXyzPoint *>(
        reinterpret_cast<const char *>(pkt) + sizeof(InnoDataPacket));
    process_xyz_point_data_<const InnoXyzPoint *>(false, pkt->use_reflectance,
                                                  pkt->item_number, pt);
  }

  return 0;
}

template <typename PointType>
void SeyondDriver::process_xyz_point_data_(bool is_en_data, bool is_use_refl,
                                           uint32_t point_num,
                                           PointType point_ptr) {
  for (uint32_t i = 0; i < point_num; i++, point_ptr++) {
    if (point_ptr->radius == 0 || point_ptr->radius > param_.max_range ||
        point_ptr->radius < param_.min_range) {
      continue;
    }
    PointXYZIT *point = point_cloud_ptr_->add_point();

    if constexpr (std::is_same<PointType, const InnoEnXyzPoint *>::value) {
      if (is_use_refl) {
        point->set_intensity(static_cast<uint>(point_ptr->reflectance));
      } else {
        point->set_intensity(static_cast<uint>(point_ptr->intensity));
      }
    } else if constexpr (std::is_same<PointType, const InnoXyzPoint *>::value) {
      point->set_intensity(static_cast<uint>(point_ptr->refl));
    }

    point->set_timestamp(static_cast<uint64_t>(
        point_ptr->ts_10us / ten_us_in_second_c + current_ts_start_));
    coordinate_transfer(point, param_.coordinate_mode, point_ptr->x,
                        point_ptr->y, point_ptr->z);

    frame_points_width_++;
  }
}

void SeyondDriver::set_log_and_message_level() {
  InnoLogLevel tmp_log_level;
  if (param_.log_level.compare("info") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_INFO;
  } else if (param_.log_level.compare("warn") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_WARNING;
  } else if (param_.log_level.compare("error") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_ERROR;
  } else {
    tmp_log_level = INNO_LOG_LEVEL_INFO;
  }
  inno_lidar_set_log_level(tmp_log_level);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
