/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/oculii_radar/parser/oculii_radar_udp_parser.h"

#include <arpa/inet.h>
#include <iostream>
#include <cerrno>
#include <cmath>
#include <vector>
#include <cstring>
#include <fcntl.h>        // NOLINT
#include <limits.h>       // NOLINT
#include <poll.h>         // NOLINT
#include <utility>
#include <sstream>
#include <sys/socket.h>   // NOLINT

#include "Eigen/Core"
#include "Eigen/Dense"

#include "cyber/cyber.h"

#include "modules/drivers/radar/oculii_radar/common/const_val.h"
#include "modules/drivers/radar/oculii_radar/common/type_def.h"

namespace apollo {
namespace drivers {
namespace radar {

OculiiRadarUdpParser::OculiiRadarUdpParser() {}

bool OculiiRadarUdpParser::Init(uint16_t port) {
  socket_ = -1;
  frame_number_ = -1;
  raw_buffer_ = std::shared_ptr<uint8_t>(
    new uint8_t[OCULII_MAX_MESSAGE_LENGTH], std::default_delete<uint8_t[]>());
  queue_ = std::shared_ptr<BoundedQueue<
    std::pair<int, std::shared_ptr<uint8_t>>>>(
      new BoundedQueue<std::pair<int, std::shared_ptr<uint8_t>>>());
  queue_->Init(OCULII_QUEUE_SIZE);

  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == socket_) {
    AERROR << "socket open error";
    return false;
  }

  sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = INADDR_ANY;

  if (bind(socket_, reinterpret_cast<sockaddr *>(&address),
           sizeof(sockaddr)) == -1) {
    AERROR << "socket bind error, port:" << port;
    return false;
  }

  if (fcntl(socket_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "fcntl udp socket error";
    return false;
  }

  async_result_ = cyber::Async(&OculiiRadarUdpParser::AsyncUdpInput, this);
  return true;
}

OculiiRadarUdpParser::~OculiiRadarUdpParser() {
  if (running_.load()) {
    if (socket_ != 0)
      close(socket_);
    running_.exchange(false);
  }
}

void OculiiRadarUdpParser::AsyncUdpInput() {
  running_.exchange(true);

  fd_set fds;
  /* Timeout. */
  struct timeval tv;

  static sockaddr_in address{};
  static socklen_t address_length = sizeof(address);
  int r = 0;

  int nbytes = -1;

  while (true) {
    FD_ZERO(&fds);
    FD_SET(socket_, &fds);

    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(socket_+1, &fds, nullptr, nullptr, &tv);
    if (r < 0) {
      if (errno != EINTR) {
        AERROR << "select socket error:" << strerror(errno);
      }
      continue;
    }
    if (r == 0) {
      AERROR << "select socket timeout";
      continue;
    }

    nbytes = recvfrom(socket_, raw_buffer_.get(), ETHERNET_MTU, 0,
                  reinterpret_cast<sockaddr *>(&address),
                  &address_length);

    if (nbytes <= 0) {
      AERROR << "receive 0 bytes packet!";
      continue;
    }

    auto packet = std::shared_ptr<uint8_t>(
        new uint8_t[nbytes], std::default_delete<uint8_t[]>());

    memcpy(reinterpret_cast<void*>(packet.get()),
            reinterpret_cast<void*>(raw_buffer_.get()), nbytes);

    queue_->WaitEnqueue(std::make_pair(nbytes, packet));
  }

  // abnormal exit
  return;
}

int OculiiRadarUdpParser::Parse(
    apollo::drivers::OculiiPointCloud& oculii_output) {
  int16_t nbyte = -1;
  bool multi_cast_mode;
  OculiiHandshake handshake;
  OculiiHeader header;
  OculiiFooter footer;
  auto acquired_packet_pair =
    std::make_pair<int, std::shared_ptr<uint8_t>>(-1, nullptr);   // NOLINT

  while (nbyte != OCULII_HANDSHAKE_SIZE) {
    if (!queue_->WaitDequeue(&acquired_packet_pair)) {
      AERROR << "dequeue failed";
      return -1;
    }
    nbyte = acquired_packet_pair.first;
    memcpy(reinterpret_cast<void*>(&handshake),
            reinterpret_cast<void*>(acquired_packet_pair.second.get()), nbyte);
  }
  uint32_t data_length = handshake.data_length;
  uint32_t offset = 0;
  auto buffer = std::shared_ptr<uint8_t>(
    new uint8_t[data_length], std::default_delete<uint8_t[]>());

  while (data_length > 0) {
    auto packet_pair = std::make_pair<int, std::shared_ptr<uint8_t>>(   // NOLINT
      -1, nullptr);
    if (!queue_->WaitDequeue(&packet_pair)) {
      AERROR << "dequeue data packet failed";
      return -1;
    }
    int packet_length = packet_pair.first;
    if (packet_length > OCULII_MAX_MESSAGE_LENGTH) {
      AERROR << "udp receive data is larger than OCULII_MAX_MESSAGE_LENGTH";
      return -1;
    }

    if (offset > handshake.data_length) {
      AERROR << "the length of data is not corresponding with the handshake";
      return -1;
    }
    memcpy(reinterpret_cast<void*>(buffer.get()+offset),
      reinterpret_cast<void*>(packet_pair.second.get()), packet_length);
    offset += packet_length;
    data_length -= packet_length;
  }

  if (data_length != 0) {
    AERROR << "the length of data is not corresponding with the handshake";
    return -1;
  }

  int parser_offset = 0;
  parser_offset += ParseHeader(header,
    reinterpret_cast<void*>(buffer.get()+parser_offset));
  if (frame_number_ < 0) {
    frame_number_ = static_cast<int64_t>(header.frame_number);
  }
  else if (frame_number_+1 != static_cast<int64_t>(header.frame_number)) {  // NOLINT
    AWARN << "packet id is not subsequent, last id: " << \
      frame_number_ << " current id: " << header.frame_number;
    AWARN << "may caused inference incorrectly";
    frame_number_ = static_cast<int64_t>(header.frame_number);
  }

  std::vector<std::shared_ptr<OculiiDetection>> detections;
  std::vector<std::shared_ptr<OculiiTrack>> tracks;

  for (uint16_t i = 0; i < header.dections_number; i++) {
    detections.push_back(std::shared_ptr<OculiiDetection>(new OculiiDetection));
    parser_offset += ParseDection(*(detections[i].get()),
      reinterpret_cast<void*>(buffer.get()+parser_offset));
  }

  for (uint16_t i = 0; i < header.tracks_number; i++) {
    tracks.push_back(std::shared_ptr<OculiiTrack>(new OculiiTrack));
    parser_offset += ParseTrack(*(tracks[i].get()),
      reinterpret_cast<void*>(buffer.get()+parser_offset));
  }

  parser_offset += ParseFooter(footer,
    reinterpret_cast<void*>(buffer.get()+parser_offset));

  // parse raw data
  oculii_output.set_packet_id(frame_number_);
  oculii_output.set_ego_speed(static_cast<float>(header.host_speed) / 100);
  oculii_output.set_ego_angle(static_cast<float>(header.host_angle) / 100);
  oculii_output.set_detection_size(header.dections_number);
  oculii_output.set_track_size(header.tracks_number);
  oculii_output.set_width(header.dections_number);
  oculii_output.set_height(1);

  float header_range_idx =
    static_cast<float>(header.range_accuracy_idx) / 10000;
  float header_doppler_idx =
    static_cast<float>(header.doppler_accuracy_idx) / 10000;
  float header_azimuth_idx =
    static_cast<float>(header.azimuth_accuracy_idx) / 10000;
  float header_elevation_idx =
    static_cast<float>(header.elevation_accuracy_idx) / 10000;

  float footer_range_idx =
    static_cast<float>(footer.range_accuracy_idx) / 10000;
  float footer_doppler_idx =
    static_cast<float>(footer.doppler_accuracy_idx) / 10000;
  float footer_azimuth_idx =
    static_cast<float>(footer.azimuth_accuracy_idx) / 10000;
  float footer_elevation_idx =
    static_cast<float>(footer.elevation_accuracy_idx) / 10000;

  multi_cast_mode = IsMultiCastMode(
    header_range_idx, header_doppler_idx, header_azimuth_idx,
    header_elevation_idx, footer_range_idx, footer_doppler_idx,
    footer_azimuth_idx, footer_elevation_idx);

  for (uint16_t i = 0; i < header.dections_number; i++) {
    auto detection = detections[i];
    auto point = oculii_output.add_point();
    auto raw_point = oculii_output.add_raw_pointclouds();

    uint16_t dot_flag;
    uint16_t power_value;

    uint16_t range_index_raw, doppler_index_raw;
    uint16_t azimuth_index_raw, beta_index_raw;
    float ele_accu, azi_accu, dopp_accu, rang_accu;
    float beta_decode, azimuth_decode, doppler_decode, range_decode;

    dot_flag = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 56) & 0x40);       // NOLINT
    power_value = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 40) & 0xFFFF);  // NOLINT
    if (multi_cast_mode) {
      if (dot_flag == DETECTION_FLAG::USING_IDX_HEADER) {
        ele_accu = header_elevation_idx;
        azi_accu = header_azimuth_idx;
        dopp_accu = header_doppler_idx;
        rang_accu = header_range_idx;
      } else if (dot_flag == DETECTION_FLAG::USING_IDX_FOOTER) {
        ele_accu = footer_elevation_idx;
        azi_accu = footer_azimuth_idx;
        dopp_accu = footer_doppler_idx;
        rang_accu = footer_range_idx;
      } else {
        AERROR << "incorrect flag received";
        return -1;
      }
    } else {
      ele_accu = header_elevation_idx;
      azi_accu = header_azimuth_idx;
      dopp_accu = header_doppler_idx;
      rang_accu = header_range_idx;
    }

    beta_index_raw = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 30) & 0x03FF);     // NOLINT
    azimuth_index_raw = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 20) & 0x03FF);  // NOLINT
    doppler_index_raw = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 10) & 0x03FF);  // NOLINT
    range_index_raw = (((*(reinterpret_cast<uint64_t*>(detection->raw))) >> 0) & 0x03FF);     // NOLINT

    // label mask
    if (0x0200 & beta_index_raw)
      beta_decode = static_cast<int>(
        (beta_index_raw & 0x01FF) - 0x0200) * ele_accu;
    else
      beta_decode = beta_index_raw * ele_accu;
    if (0x0200 & azimuth_index_raw)
      azimuth_decode = static_cast<int>(
        (azimuth_index_raw & 0x01FF) - 0x0200) * azi_accu;
    else
      azimuth_decode = azimuth_index_raw * azi_accu;
    if (0x0200 & doppler_index_raw)
      doppler_decode = static_cast<int>(
        (doppler_index_raw & 0x01FF) - 0x0200) * dopp_accu;
    else
      doppler_decode = doppler_index_raw * dopp_accu;

    range_decode = range_index_raw * rang_accu;

    raw_point->set_elevation(beta_decode);
    raw_point->set_azimuth(azimuth_decode);
    raw_point->set_doppler(doppler_decode);
    raw_point->set_range(range_decode);
    raw_point->set_power(static_cast<float>(power_value) / 100);

    Eigen::Translation3d trans(Eigen::Vector3d(0, 0, 0));
    Eigen::Quaterniond quater(-0.5, 0.5, -0.5, 0.5);
    Eigen::Affine3d radar2lidar_pose =
      trans * quater.toRotationMatrix();

    float point_x = std::sin(raw_point->azimuth() * DEG2RAD) * \
      std::cos(raw_point->elevation() * DEG2RAD) * raw_point->range();
    float point_y = std::sin(raw_point->elevation() * DEG2RAD) * \
      raw_point->range();
    float point_z = std::cos(raw_point->azimuth() * DEG2RAD) * \
      std::cos(raw_point->elevation() * DEG2RAD) * raw_point->range();

    Eigen::Vector3d vec(point_x, point_y, point_z);
    vec = radar2lidar_pose * vec;

    point->set_x(vec.x());
    point->set_y(vec.y());
    point->set_z(vec.z());

    point->set_intensity(raw_point->power());
    point->set_velocity(raw_point->doppler());
  }

  for (uint16_t i = 0; i < header.tracks_number; i++) {
    auto track = tracks[i];
    auto track_res = oculii_output.add_tracks();
    track_res->set_id(track->id);
    track_res->set_x_pos(static_cast<float>(track->x_pos) / 100);
    track_res->set_y_pos(static_cast<float>(track->y_pos) / 100);
    track_res->set_z_pos(static_cast<float>(track->z_pos) / 100);
    track_res->set_x_dot(static_cast<float>(track->x_dot) / 100);
    track_res->set_y_dot(static_cast<float>(track->y_dot) / 100);
    track_res->set_z_dot(static_cast<float>(track->z_dot) / 100);
    track_res->set_confidence(track->confidence);
    track_res->set_track_class(
      static_cast<OculiiTrackTarget::ObstacleClass>(track->track_class));
  }

  return 0;
}

int OculiiRadarUdpParser::ParseHeader(OculiiHeader& header, void* buffer) {
  memcpy(reinterpret_cast<void*>(&header), buffer, OCULII_HEADER_SIZE);
  return OCULII_HEADER_SIZE;
}

int OculiiRadarUdpParser::ParseDection(OculiiDetection& dection, void* buffer) {
  memcpy(reinterpret_cast<void*>(&dection), buffer, OCULII_DECTION_BLOCK_SIZE);
  return OCULII_DECTION_BLOCK_SIZE;
}

int OculiiRadarUdpParser::ParseTrack(OculiiTrack& track, void* buffer) {
  memcpy(reinterpret_cast<void*>(&track), buffer, OCULII_TRACKER_BLOCK_SIZE);
  return OCULII_TRACKER_BLOCK_SIZE;
}

int OculiiRadarUdpParser::ParseFooter(OculiiFooter& footer, void* buffer) {
  memcpy(reinterpret_cast<void*>(&footer), buffer, OCULII_FOOTER_BLOCK_LENGTH);
  return OCULII_FOOTER_BLOCK_LENGTH;
}

bool OculiiRadarUdpParser::IsMultiCastMode(float hr, float hd,
    float ha, float he, float fr, float fd, float fa, float fe) {
  if (fabs(fr) < 10e-5 && fabs(fa) < 10e-5 && \
      fabs(fd) < 10e-5 && fabs(fe) < 10e-5)
    return false;
  else if (fabs(fr - hr) > 10e-5 || fabs(fa - ha) > 10e-5 ||
           fabs(fd - hd) > 10e-5 || fabs(fe - he) > 10e-5)
    return true;
  else
    return false;
}


}  // namespace radar
}  // namespace drivers
}  // namespace apollo
