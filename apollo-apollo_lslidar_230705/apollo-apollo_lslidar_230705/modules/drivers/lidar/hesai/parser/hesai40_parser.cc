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

#include "modules/drivers/lidar/hesai/parser/hesai40_parser.h"

namespace apollo {
namespace drivers {
namespace hesai {

using ::apollo::cyber::Node;
using apollo::drivers::PointXYZIT;

Hesai40Parser::Hesai40Parser(const std::shared_ptr<Node> &node,
                             const Config &conf)
    : Parser(node, conf) {
  // init the block time offset, us
  block_offset_[9] = 55.56 * 0.0 + 28.58;
  block_offset_[8] = 55.56 * 1.0 + 28.58;
  block_offset_[7] = 55.56 * 2.0 + 28.58;
  block_offset_[6] = 55.56 * 3.0 + 28.58;
  block_offset_[5] = 55.56 * 4.0 + 28.58;
  block_offset_[4] = 55.56 * 5.0 + 28.58;
  block_offset_[3] = 55.56 * 6.0 + 28.58;
  block_offset_[2] = 55.56 * 7.0 + 28.58;
  block_offset_[1] = 55.56 * 8.0 + 28.58;
  block_offset_[0] = 55.56 * 9.0 + 28.58;

  // init the laser shot time offset, us
  laser_offset_[3] = 3.62;
  laser_offset_[39] = 3.62;
  laser_offset_[35] = 4.92;
  laser_offset_[27] = 6.23;
  laser_offset_[11] = 8.19;
  laser_offset_[15] = 8.19;
  laser_offset_[31] = 9.5;
  laser_offset_[23] = 11.47;
  laser_offset_[28] = 12.77;
  laser_offset_[16] = 14.74;
  laser_offset_[2] = 16.04;
  laser_offset_[38] = 16.04;
  laser_offset_[34] = 17.35;
  laser_offset_[24] = 18.65;
  laser_offset_[8] = 20.62;
  laser_offset_[12] = 20.62;
  laser_offset_[30] = 21.92;
  laser_offset_[20] = 23.89;
  laser_offset_[25] = 25.19;
  laser_offset_[13] = 27.16;
  laser_offset_[1] = 28.47;
  laser_offset_[37] = 28.47;
  laser_offset_[33] = 29.77;
  laser_offset_[5] = 31.74;
  laser_offset_[21] = 31.7447;
  laser_offset_[9] = 33.71;
  laser_offset_[29] = 35.01;
  laser_offset_[17] = 36.98;
  laser_offset_[22] = 38.95;
  laser_offset_[10] = 40.91;
  laser_offset_[0] = 42.22;
  laser_offset_[36] = 42.22;
  laser_offset_[32] = 43.52;
  laser_offset_[4] = 45.49;
  laser_offset_[18] = 45.49;
  laser_offset_[6] = 47.46;
  laser_offset_[26] = 48.76;
  laser_offset_[14] = 50.73;
  laser_offset_[19] = 52.7;
  laser_offset_[9] = 54.67;

  for (int i = 0; i < LASER_COUNT; i++) {
    elev_angle_map_[i] = pandar40p_elev_angle_map[i];
    horizatal_azimuth_offset_map_[i] =
        pandar40p_horizatal_azimuth_offset_map[i];
  }
}

Hesai40Parser::~Hesai40Parser() {}

void Hesai40Parser::ParseRawPacket(const uint8_t *buf, const int len,
                                   bool *is_end) {
  // PrintMem(buf, len);
  if (len != PACKET_SIZE && len != PACKET_SIZE_WITH_UDPSEQ) {
    AWARN << "packet size:" << len << " mismatch internal size:" << PACKET_SIZE;
    return;
  }
  Hesai40Packet packet;
  int index = 0;
  // 10 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    Hesai40PBlock &block = packet.blocks[i];
    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x units
    for (int j = 0; j < LASER_COUNT; j++) {
      Hesai40PUnit &unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      // distance is M.
      unit.distance =
          static_cast<double>(range) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      // TODO(Philip.Pi): Filtering wrong data for LiDAR.
      if ((unit.distance == 0x010101 && unit.intensity == 0x0101) ||
          unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }

      index += RAW_MEASURE_SIZE;
    }
  }
  index += RESERVE_SIZE;  // skip reserved bytes
  index += REVOLUTION_SIZE;

  packet.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                ((buf[index + 2] & 0xff) << 16) |
                ((buf[index + 3] & 0xff) << 24);
  packet.usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet.echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  // parse the UTC Time.
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet.t.tm_year = (buf[index + 0] & 0xff) + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  packet.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet.t.tm_mday = buf[index + 2] & 0xff;
  packet.t.tm_hour = buf[index + 3] & 0xff;
  packet.t.tm_min = buf[index + 4] & 0xff;
  packet.t.tm_sec = buf[index + 5] & 0xff;
  packet.t.tm_isdst = 0;

  // calc point xyzi
  for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
    int azimuthGap = 0; /* To do */
    if (last_azimuth_ > packet.blocks[i].azimuth) {
      azimuthGap = static_cast<int>(packet.blocks[i].azimuth) +
                   (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(packet.blocks[i].azimuth) -
                   static_cast<int>(last_azimuth_);
    }

    if (last_azimuth_ != packet.blocks[i].azimuth &&
        azimuthGap < 600 /* 6 degree*/) {
      /* for all the blocks */
      if ((last_azimuth_ > packet.blocks[i].azimuth &&
           start_angle_ <= packet.blocks[i].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= packet.blocks[i].azimuth)) {
        *is_end = true;
      }
    }

    CalcPointXYZIT(&packet, i);
    last_azimuth_ = packet.blocks[i].azimuth;
  }
}

void Hesai40Parser::CalcPointXYZIT(Hesai40Packet *pkt, int blockid) {
  Hesai40PBlock *block = &pkt->blocks[blockid];
  double unix_second = static_cast<double>(mktime(&pkt->t) + tz_second_);
  double timestamp = unix_second + (static_cast<double>(pkt->usec)) / 1000000.0;
  CheckPktTime(timestamp);

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the units in a block */
    Hesai40PUnit &unit = block->units[i];
    /* skip wrong points */
    if (unit.distance <= 0.5 || unit.distance > 200.0) {
      continue;
    }
    double xyDistance =
        unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
    float x = static_cast<float>(
        xyDistance *
        sinf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    float y = static_cast<float>(
        xyDistance *
        cosf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    float z = static_cast<float>(unit.distance *
                                 sinf(degreeToRadian(elev_angle_map_[i])));

    PointXYZIT *new_point = raw_pointcloud_out_->add_point();
    new_point->set_x(-y);
    new_point->set_y(x);
    new_point->set_z(z);
    new_point->set_intensity(unit.intensity);

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      timestamp -=
          (static_cast<double>(block_offset_[blockid / 2] + laser_offset_[i]) /
           1000000.0f);
    } else {
      timestamp -=
          (static_cast<double>(block_offset_[blockid] + laser_offset_[i]) /
           1000000.0f);
    }
    uint64_t stamp = static_cast<uint64_t>(timestamp * 1e9);
    new_point->set_timestamp(stamp);
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
