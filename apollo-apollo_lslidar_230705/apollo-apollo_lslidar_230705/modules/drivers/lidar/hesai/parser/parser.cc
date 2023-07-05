/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include <cmath>
#include <memory>
#include <string>

#include "modules/drivers/lidar/hesai/parser/parser.h"

namespace apollo {
namespace drivers {
namespace hesai {

using apollo::drivers::PointCloud;

Parser::Parser(const std::shared_ptr<::apollo::cyber::Node>& node,
               const Config& conf)
    : node_(node), conf_(conf) {
  tz_second_ = conf_.time_zone() * 3600;
  start_angle_ = static_cast<int>(conf_.start_angle() * 100);
}

Parser::~Parser() { Stop(); }

bool Parser::Init() {
  if (inited_) {
    return true;
  }

  // init calibration
  if (!conf_.is_online_calibration() && conf_.calibration_file() != "") {
    if (!LoadCalibration(conf_.calibration_file().c_str())) {
      AERROR << "load local calibration file[" << conf_.calibration_file()
             << "] error";
      return false;
    }
    is_calibration_ = true;
  } else {
    online_calibration_thread_ =
        std::thread(&Parser::LoadCalibrationThread, this);
  }

  // init writer
  raw_pointcloud_writer_ =
      node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());

  if (raw_pointcloud_writer_ == nullptr) {
    AERROR << "create writer:" << conf_.pointcloud_channel()
           << " error, check cyber is init?";
    return false;
  }

  // raw_pointcloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  // raw_pointcloud_pool_->ConstructAll();
  // for (int i = 0; i < pool_size_; i++) {
  //   auto point_cloud = raw_pointcloud_pool_->GetObject();
  //   if (point_cloud == nullptr) {
  //     AERROR << "fail to getobject, i: " << i;
  //     return false;
  //   }
  //   point_cloud->mutable_point()->Reserve(70000);
  // }

  raw_pointcloud_pool_.resize(pool_size_);
  for (int i = 0; i < pool_size_; i++) {
    raw_pointcloud_pool_[i] = std::make_shared<PointCloud>();
    if (raw_pointcloud_pool_[i] == nullptr) {
      AERROR << "make shared PointCloud error,oom";
      return false;
    }
    raw_pointcloud_pool_[i]->mutable_point()->Reserve(70000);
  }

  ResetRawPointCloud();
  inited_ = true;
  return true;
}

void Parser::ResetRawPointCloud() {
  // raw_pointcloud_out_ = raw_pointcloud_pool_->GetObject();
  // if (raw_pointcloud_out_ == nullptr) {
  // raw_pointcloud_out_ = std::make_shared<PointCloud>();
  // raw_pointcloud_out_->mutable_point()->Reserve(70000);
  // AWARN << "raw pointcloud out make new";
  // }
  // raw_pointcloud_out_->Clear();
  //
  raw_pointcloud_out_ = raw_pointcloud_pool_.at(pool_index_);
  AINFO << "pool index:" << pool_index_;
  raw_pointcloud_out_->Clear();
  raw_pointcloud_out_->mutable_point()->Reserve(70000);
  pool_index_ = (pool_index_ + 1) % pool_size_;
}

bool Parser::Parse(const std::shared_ptr<HesaiScan>& scan) {
  ResetRawPointCloud();
  bool is_end = false;
  for (int i = 0; i < scan->firing_pkts_size(); ++i) {
    const auto& pkt = scan->firing_pkts(i);
    uint8_t* data =
        reinterpret_cast<uint8_t*>(const_cast<char*>(pkt.data().c_str()));
    ParseRawPacket(data, pkt.data().size(), &is_end);
  }
  PublishRawPointCloud(scan->header().sequence_num());
  return true;
}

void Parser::Parse(const uint8_t* data, int size, bool* is_end) {
  bool t_is_end = false;
  ParseRawPacket(data, size, &t_is_end);
  ++packet_nums_;
  *is_end = CheckIsEnd(t_is_end);
  if (*is_end == false) {
    return;
  }
  packet_nums_ = 0;
  PublishRawPointCloud();
  ResetRawPointCloud();
}

bool Parser::CheckIsEnd(bool is_end) {
  if (packet_nums_ >= max_packets_) {
    AWARN << "over max packets, packets:" << packet_nums_
          << ", max packets:" << max_packets_;
    return true;
  }
  if (is_end && packet_nums_ < min_packets_) {
    AWARN << "receive too less packets:" << packet_nums_ << ", not end"
          << ", min packets:" << min_packets_;
    return false;
  }
  return is_end;
}

void Parser::PublishRawPointCloud(int seq) {
  int size = raw_pointcloud_out_->point_size();
  if (size == 0) {
    AWARN << "All points size is NAN! Please check hesai:" << conf_.model();
    return;
  }

  raw_pointcloud_out_->mutable_header()->set_sequence_num(seq_index_++);
  if (seq > 0) {
    raw_pointcloud_out_->mutable_header()->set_sequence_num(seq);
  }
  raw_pointcloud_out_->mutable_header()->set_frame_id(conf_.frame_id());
  raw_pointcloud_out_->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());
  raw_pointcloud_out_->set_height(1);
  raw_pointcloud_out_->set_width(size);
  const auto timestamp =
      raw_pointcloud_out_->point(static_cast<int>(size) - 1).timestamp();
  raw_pointcloud_out_->set_measurement_time(static_cast<double>(timestamp) /
                                            1e9);
  raw_pointcloud_out_->mutable_header()->set_lidar_timestamp(timestamp);
  raw_pointcloud_writer_->Write(raw_pointcloud_out_);
}

void Parser::LoadCalibrationThread() {
  TcpCmdClient tcp_cmd(conf_.ip(), conf_.tcp_cmd_port());
  std::string content;
  AINFO << "start LoadCalibrationThread";
  while (running_) {
    AWARN << "calibration has not inited";
    if (!tcp_cmd.GetCalibration(&content)) {
      AERROR << "tcp get calibration error";
      sleep(1);
      continue;
    }

    if (!LoadCalibration(content)) {
      AERROR << "load calibration error";
      continue;
    }
    AINFO << "online calibration success";
    is_calibration_ = true;
    break;
  }
  AINFO << "exit LoadCalibrationThread, calibration:" << is_calibration_;
}

void Parser::Stop() {
  running_.store(false);
  if (online_calibration_thread_.joinable()) {
    online_calibration_thread_.join();
  }
}

bool Parser::LoadCalibration(const char* path_file) {
  std::string path(path_file);
  std::string content;
  if (!apollo::cyber::common::GetContent(path, &content)) {
    AERROR << "get calibration file content error, file:" << path_file;
    return false;
  }
  return LoadCalibration(content);
}

bool Parser::LoadCalibration(const std::string& content) {
  AINFO << "parse calibration content:" << content;
  std::istringstream ifs(content);
  std::string line;
  if (!std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    AERROR << "lidar calibration content is empty, content:" << content;
    return false;
  }

  double azimuthOffset[LASER_COUNT_L64];
  double elev_angle[LASER_COUNT_L64];

  int line_counter = 0;
  while (std::getline(ifs, line)) {
    if (line_counter++ >= LASER_COUNT_L64) {
      break;
    }

    double elev = 0, azimuth = 0;
    int line_id = 0;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> line_id;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (line_id != line_counter) {
      AERROR << "parser lidar calibration content error, line_id[" << line_id
             << "] != line_counter[" << line_counter << "]";
      return false;
    }

    elev_angle[line_id - 1] = elev;
    azimuthOffset[line_id - 1] = azimuth;
  }

  AINFO << "laser count:" << line_counter;
  if (line_counter != LASER_COUNT && line_counter != LASER_COUNT_L64) {
    AERROR << "now only support hesai40p, hesai64";
    return false;
  }

  for (int i = 0; i < line_counter; ++i) {
    elev_angle_map_[i] = elev_angle[i];
    horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }
  return true;
}

void Parser::CheckPktTime(double time_sec) {
  double now = apollo::cyber::Time().Now().ToSecond();
  double diff = std::abs(now - time_sec);
  if (diff > 0.1) {
    // AWARN << conf_.frame_id() << " time too big, diff:" << diff
    //       << "host time:" << now << ";lidar time:" << time_sec;
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
