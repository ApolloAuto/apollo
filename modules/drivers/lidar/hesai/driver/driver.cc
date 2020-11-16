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

#include "modules/drivers/lidar/hesai/driver/driver.h"

namespace apollo {
namespace drivers {
namespace hesai {

bool HesaiDriver::Init() {
  if (node_ == nullptr) {
    AERROR << "node is nullptr";
    return false;
  }
  scan_writer_ = node_->CreateWriter<HesaiScan>(conf_.scan_channel());
  if (scan_writer_ == nullptr) {
    AERROR << "writer:" << conf_.scan_channel()
           << " create error, check cyber is inited.";
    return false;
  }

  if (conf_.model() == HESAI40P) {
    pkt_buffer_capacity_ = HESAI40_MAX_PACKETS * 10;
  } else if (conf_.model() == HESAI64) {
    pkt_buffer_capacity_ = HESAI64_MAX_PACKETS * 10;
  } else {
    AERROR << "Not support model:" << conf_.model();
    return false;
  }

  AINFO << "packet buffer capacity:" << pkt_buffer_capacity_;
  pkt_buffer_.resize(pkt_buffer_capacity_);
  for (int i = 0; i < pkt_buffer_capacity_; ++i) {
    pkt_buffer_[i] = std::make_shared<HesaiPacket>();
  }

  if (!parser_->Init()) {
    AERROR << "parser init error";
    return false;
  }

  scan_buffer_.resize(scan_buffer_size_);
  for (int i = 0; i < scan_buffer_size_; ++i) {
    scan_buffer_[i] = std::make_shared<HesaiScan>();
    if (scan_buffer_[i] == nullptr) {
      AERROR << "make scan buffer error";
      return false;
    }
  }
  tz_second_ = conf_.time_zone() * 3600;
  input_.reset(new Input(conf_.lidar_recv_port(), conf_.gps_recv_port()));
  poll_thread_ = std::thread(&HesaiDriver::PollThread, this);
  process_thread_ = std::thread(&HesaiDriver::ProcessThread, this);
  return true;
}

void HesaiDriver::PollThread() {
  AINFO << "Poll thread start";
  while (running_) {
    auto start = std::chrono::steady_clock::now();
    std::shared_ptr<HesaiPacket>& pkt = pkt_buffer_[pkt_index_];
    if (input_->GetPacket(pkt.get()) == -1) {
      continue;
    }
    auto end = std::chrono::steady_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    if (duration > 50) {
      AWARN << "recv packet cost:" << duration;
    }
    {
      std::lock_guard<std::mutex> lck(packet_mutex_);
      pkt_queue_.push_back(pkt);
      packet_condition_.notify_all();
    }
    pkt_index_ = (pkt_index_ + 1) % pkt_buffer_capacity_;
  }
}

void HesaiDriver::ProcessGps(const HesaiPacket& pkt) {
  if (pkt.size != GPS_PACKET_SIZE) {
    return;
  }

  const uint8_t* recvbuf = pkt.data;
  int index = 0;
  GPS gps;
  gps.flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  gps.year = ((recvbuf[index] & 0xff) - 0x30) +
             ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  gps.month = ((recvbuf[index] & 0xff) - 0x30) +
              ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  gps.day = ((recvbuf[index] & 0xff) - 0x30) +
            ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  gps.second = ((recvbuf[index] & 0xff) - 0x30) +
               ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  gps.minute = ((recvbuf[index] & 0xff) - 0x30) +
               ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  gps.hour = ((recvbuf[index] & 0xff) - 0x30) +
             ((recvbuf[index + 1] & 0xff) - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  gps.fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                 ((recvbuf[index + 2] & 0xff) << 16) |
                 ((recvbuf[index + 3] & 0xff) << 24);

  struct tm t;
  t.tm_sec = gps.second;
  t.tm_min = gps.minute;
  t.tm_hour = gps.hour;
  t.tm_mday = gps.day;

  // UTC's month start from 1, but mktime only accept month from 0.
  t.tm_mon = gps.month - 1;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  t.tm_year = gps.year + 100;
  t.tm_isdst = 0;

  // static_cast<double>(mktime(&t) + tz_second_));
  uint64_t gps_timestamp = static_cast<uint64_t>(timegm(&t));
  auto now = apollo::cyber::Time().Now().ToNanosecond();
  AINFO << "hesai gps time:" << gps_timestamp << ", sys time:" << now
        << ", diff:" << now - gps_timestamp;
}

void HesaiDriver::ProcessThread() {
  std::shared_ptr<HesaiPacket> pkt = nullptr;
  bool is_end = false;
  int seq = 0;
  AINFO << "packet process thread start";
  while (running_) {
    {
      std::unique_lock<std::mutex> lck(packet_mutex_);
      if (pkt_queue_.empty()) {
        packet_condition_.wait(lck);
      }
      if (pkt_queue_.empty()) {
        // exit notify empty
        continue;
      }
      pkt = pkt_queue_.front();
      pkt_queue_.pop_front();
    }

    if (pkt->size == GPS_PACKET_SIZE) {
      ProcessGps(*pkt);
      continue;
    }
    scan_buffer_[index_]->add_firing_pkts()->set_data(pkt->data, pkt->size);
    parser_->Parse(pkt->data, pkt->size, &is_end);

    if (is_end && scan_buffer_[index_]->firing_pkts_size() > 0) {
      is_end = false;
      auto now = apollo::cyber::Time().Now();
      scan_buffer_[index_]->mutable_header()->set_timestamp_sec(now.ToSecond());
      scan_buffer_[index_]->mutable_header()->set_frame_id(conf_.frame_id());
      scan_buffer_[index_]->mutable_header()->set_sequence_num(seq++);
      scan_buffer_[index_]->set_model(conf_.model());
      scan_buffer_[index_]->set_basetime(0);
      scan_writer_->Write(scan_buffer_[index_]);
      AINFO << "publish scan size:" << scan_buffer_[index_]->firing_pkts_size()
            << ", index:" << index_;
      ++index_;
      index_ = index_ % scan_buffer_size_;
      scan_buffer_[index_]->Clear();
    }
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
