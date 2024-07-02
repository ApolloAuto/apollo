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
#include "modules/drivers/lidar/seyond/driver/seyond_driver.h"

static const uint32_t KBUF_SIZE = 1024 * 1024 * 10;
static const double us_in_second_c = 1000000.0;
static const double ten_us_in_second_c = 100000.0;
namespace apollo {
namespace drivers {
namespace seyond {

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

SeyondDriver::SeyondDriver(const std::shared_ptr<apollo::cyber::Node> &node,
                           const apollo::drivers::seyond::Config &seyond_conf)
    : node_ptr_(node), conf_(seyond_conf) {
  point_cloud_ptr_.reset(new PointCloud);
  scan_packets_ptr_ = std::make_shared<SeyondScan>();
  convert_buffer_.resize(KBUF_SIZE);
}

SeyondDriver::~SeyondDriver() {
  stop();
}

void SeyondDriver::setup_param() {
  if (conf_.source_type() == Config::RAW_PACKET) raw_packets_mode_ = true;
  if (conf_.has_scan_channel()) scan_channel_ = conf_.scan_channel();
  if (conf_.has_pointcloud_channel())
    pointcloud_channel_ = conf_.pointcloud_channel();
  if (conf_.has_frame_id()) frame_id_ = conf_.frame_id();
  if (conf_.has_direct_mode()) direct_mode_ = conf_.direct_mode();
  if (conf_.has_aggregate_num()) aggregate_packets_num_ = conf_.aggregate_num();
  if (conf_.has_device_ip()) device_ip_ = conf_.device_ip();
  if (conf_.has_port()) port_ = conf_.port();
  if (conf_.has_udp_port()) udp_port_ = conf_.udp_port();
  if (conf_.has_reflectance_mode())
    reflectance_mode_ = conf_.reflectance_mode();
  if (conf_.has_multiple_return()) multiple_return_ = conf_.multiple_return();
  if (conf_.has_coordinate_mode()) coordinate_mode_ = conf_.coordinate_mode();
  if (conf_.has_max_range()) max_range_ = conf_.max_range();
  if (conf_.has_min_range()) min_range_ = conf_.min_range();
  if (conf_.has_log_level()) log_level_ = conf_.log_level();
}

bool SeyondDriver::init_lidar() {
  // setup read from live
  const char *ip_str = device_ip_.c_str();
  uint16_t lidar_udp_port = 0;
  enum InnoLidarProtocol protocol_ =
      (udp_port_ >= 0 ? INNO_LIDAR_PROTOCOL_PCS_UDP
                      : INNO_LIDAR_PROTOCOL_PCS_TCP);
  handle_ = inno_lidar_open_live(lidar_name_.c_str(), ip_str, port_, protocol_,
                                 lidar_udp_port);
  // get internal pcs enable status
  uint32_t is_internal_pcs = 1;
  char buf[20] = {'\0'};
  int32_t ret =
      inno_lidar_get_attribute_string(handle_, "enabled", buf, sizeof(buf));
  if (ret != 0) {
    inno_log_error("can not get lidar pcs enable status");
    return false;
  }
  std::string enable(buf);
  is_internal_pcs = atoi(buf);
  if (is_internal_pcs != 1) {
    inno_log_error("error getting PCS enable status: %u", is_internal_pcs);
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

  if (set_input_parameter_() != 0) {
    return false;
  }

  return true;
}

int32_t SeyondDriver::set_input_parameter_() {
  int32_t ret = 0;
  // set lidar reflectance mode
  enum InnoReflectanceMode m = reflectance_mode_
                                   ? INNO_REFLECTANCE_MODE_REFLECTIVITY
                                   : INNO_REFLECTANCE_MODE_INTENSITY;
  ret = inno_lidar_set_reflectance_mode(handle_, m);
  if (ret != 0) {
    inno_log_error("Lidar %s cannot set reflectance mode", lidar_name_.c_str());
    return ret;
  }

  // set lidar return mode
  ret = inno_lidar_set_return_mode(handle_,
                                   (InnoMultipleReturnMode)multiple_return_);
  if (ret != 0) {
    inno_log_error("Lidar %s cannot set return mode", lidar_name_.c_str());
    return ret;
  }

  // set lidar callback
  ret = inno_lidar_set_callbacks(handle_, message_callback_s_, data_callback_s_,
                                 status_callback_s_,
                                 apollo_get_time_in_second_s, this);
  if (ret != 0) {
    inno_log_error("Lidar %s  set callbacks error", lidar_name_.c_str());
    return ret;
  }

  return 0;
}

bool SeyondDriver::init() {
  setup_param();
  set_log_and_message_level();
  inno_lidar_setup_sig_handler();
  inno_lidar_set_logs(-1, -1, NULL, 0, 0, log_callback_s_, this, NULL, 0, 0, 1);
  scan_writer_ptr_ = node_ptr_->CreateWriter<SeyondScan>(scan_channel_);
  scan_reader_ptr_ = node_ptr_->CreateReader<SeyondScan>(
      scan_channel_, std::bind(&SeyondDriver::process_scan_packet_, this,
                               std::placeholders::_1));
  pcd_writer_ptr_ = node_ptr_->CreateWriter<PointCloud>(pointcloud_channel_);
  return true;
}

bool SeyondDriver::start() {
  if (handle_ <= 0) {
    if (raw_packets_mode_) {
      return true;
    }
    if (!init_lidar()) {
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

  if (direct_mode_) {
    return process_packets_and_publish_frame_(pkt);
  } else {
    return collect_packets_and_publish_packets_(pkt);
  }
}

int32_t SeyondDriver::process_packets_and_publish_frame_(
    const InnoDataPacket *pkt) {
  // check if get the whole frame, and publish
  if ((current_frame_id_ != static_cast<int64_t>(pkt->idx))) {
    point_cloud_ptr_->mutable_header()->set_frame_id(frame_id_);
    point_cloud_ptr_->mutable_header()->set_sequence_num(current_frame_id_ %
                                                         UINT_MAX);
    point_cloud_ptr_->mutable_header()->set_lidar_timestamp(
        (static_cast<uint64_t>(pkt->common.ts_start_us)) * 1000);
    point_cloud_ptr_->set_frame_id(frame_id_);
    point_cloud_ptr_->set_measurement_time(pkt->common.ts_start_us * 1e-6);
    point_cloud_ptr_->set_height(1);
    point_cloud_ptr_->set_width(frame_points_width_);
    point_cloud_ptr_->mutable_header()->set_timestamp_sec(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count() /
        1e9);
    // data publish
    pcd_writer_ptr_->Write(point_cloud_ptr_);
    point_cloud_ptr_.reset(new PointCloud);
    current_frame_id_ = pkt->idx;
    frame_points_width_ = 0;
  }
  convert_and_parse_(pkt);
  return 0;
}

int32_t SeyondDriver::collect_packets_and_publish_packets_(
    const InnoDataPacket *pkt) {
  // check if get the whole frame, and publish
  if (current_frame_id_ != static_cast<int64_t>(pkt->idx)) {
    frame_count_++;
    scan_packets_ptr_->set_sequence_num(current_frame_id_ % UINT_MAX);
    scan_packets_ptr_->set_timestamp(
        (static_cast<uint64_t>(pkt->common.ts_start_us)) * 1000);
    scan_packets_ptr_->set_measurement_time(pkt->common.ts_start_us * 1e-6);
    scan_packets_ptr_->set_is_last_scan(true);
    scan_packets_ptr_->set_size(packets_width_);
    packets_width_ = 0;
    scan_writer_ptr_->Write(scan_packets_ptr_);
    current_frame_id_ = pkt->idx;
    scan_packets_ptr_ = std::make_shared<SeyondScan>();
  } else if (aggregate_packets_num_ == packets_width_) {
    scan_packets_ptr_->set_is_last_scan(false);
    scan_packets_ptr_->set_size(packets_width_);
    packets_width_ = 0;
    scan_writer_ptr_->Write(scan_packets_ptr_);
    scan_packets_ptr_ = std::make_shared<SeyondScan>();
  }
  SeyondPacket *scan_packet = scan_packets_ptr_->add_packets();
  uint64_t pkt_len = sizeof(InnoDataPacket) + pkt->item_number * pkt->item_size;
  scan_packet->mutable_data()->assign(reinterpret_cast<const char *>(pkt),
                                      pkt_len);
  scan_packet->set_table_exist(false);
  if (anglehv_table_init_ && (frame_count_ == table_send_hz_)) {
    frame_count_ = 0;
    scan_packet->set_table_exist(true);
    scan_packet->mutable_table()->assign(anglehv_table_.data(),
                                         anglehv_table_.size());
  }
  packets_width_++;

  return 0;
}

int32_t SeyondDriver::process_scan_packet_(
    const std::shared_ptr<SeyondScan> &lidar_packets) {
  for (auto &packet : lidar_packets->packets()) {
    const InnoDataPacket *pkt =
        reinterpret_cast<const InnoDataPacket *>(packet.data().data());
    if ((raw_packets_mode_) && (!anglehv_table_init_) &&
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
  if (lidar_packets->is_last_scan()) {
    point_cloud_ptr_->mutable_header()->set_frame_id(frame_id_);
    point_cloud_ptr_->mutable_header()->set_sequence_num(
        lidar_packets->sequence_num());
    point_cloud_ptr_->mutable_header()->set_lidar_timestamp(
        lidar_packets->timestamp());
    point_cloud_ptr_->set_frame_id(frame_id_);
    point_cloud_ptr_->set_measurement_time(lidar_packets->measurement_time());
    point_cloud_ptr_->set_height(1);
    point_cloud_ptr_->set_width(frame_points_width_);
    point_cloud_ptr_->mutable_header()->set_timestamp_sec(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count() /
        1e9);
    pcd_writer_ptr_->Write(point_cloud_ptr_);
    point_cloud_ptr_.reset(new PointCloud);
    frame_points_width_ = 0;
  }

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
    if (point_ptr->radius == 0 || point_ptr->radius > max_range_ ||
        point_ptr->radius < min_range_) {
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
    coordinate_transfer(point, coordinate_mode_, point_ptr->x, point_ptr->y,
                        point_ptr->z);

    frame_points_width_++;
  }
}

void SeyondDriver::set_log_and_message_level() {
  InnoLogLevel tmp_log_level;
  if (log_level_.compare("info") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_INFO;
  } else if (log_level_.compare("warn") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_WARNING;
  } else if (log_level_.compare("error") == 0) {
    tmp_log_level = INNO_LOG_LEVEL_ERROR;
  } else {
    tmp_log_level = INNO_LOG_LEVEL_INFO;
  }
  inno_lidar_set_log_level(tmp_log_level);
}

}  // namespace seyond
}  // namespace drivers
}  // namespace apollo
