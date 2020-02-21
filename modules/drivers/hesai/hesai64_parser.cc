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

#include "modules/drivers/hesai/parser.h"

namespace apollo {
namespace drivers {
namespace hesai {

Hesai64Parser::Hesai64Parser(const std::shared_ptr<Node> &node,
                             const Config &conf)
    : Parser(node, conf) {
  // init the block time offset, us
  block_offset_[5] = 55.56 * 0.0 + 42.58;
  block_offset_[4] = 55.56 * 1.0 + 42.58;
  block_offset_[3] = 55.56 * 2.0 + 42.58;
  block_offset_[2] = 55.56 * 3.0 + 42.58;
  block_offset_[1] = 55.56 * 4.0 + 42.58;
  block_offset_[0] = 55.56 * 5.0 + 42.58;
  // init the laser shot time offset, us
  laser_offset_[50] = 1.304 * 0 + 1.968 * 0 + 3.62;
  laser_offset_[60] = 1.304 * 0 + 1.968 * 0 + 3.62;
  laser_offset_[44] = 1.304 * 1 + 1.968 * 0 + 3.62;
  laser_offset_[59] = 1.304 * 1 + 1.968 * 0 + 3.62;
  laser_offset_[38] = 1.304 * 2 + 1.968 * 0 + 3.62;
  laser_offset_[56] = 1.304 * 2 + 1.968 * 0 + 3.62;
  laser_offset_[8] = 1.304 * 3 + 1.968 * 0 + 3.62;
  laser_offset_[54] = 1.304 * 3 + 1.968 * 0 + 3.62;
  laser_offset_[48] = 1.304 * 4 + 1.968 * 0 + 3.62;
  laser_offset_[62] = 1.304 * 4 + 1.968 * 0 + 3.62;
  laser_offset_[42] = 1.304 * 5 + 1.968 * 0 + 3.62;
  laser_offset_[58] = 1.304 * 5 + 1.968 * 0 + 3.62;
  laser_offset_[6] = 1.304 * 6 + 1.968 * 0 + 3.62;
  laser_offset_[55] = 1.304 * 6 + 1.968 * 0 + 3.62;
  laser_offset_[52] = 1.304 * 7 + 1.968 * 0 + 3.62;
  laser_offset_[63] = 1.304 * 7 + 1.968 * 0 + 3.62;
  laser_offset_[46] = 1.304 * 8 + 1.968 * 0 + 3.62;
  laser_offset_[61] = 1.304 * 8 + 1.968 * 0 + 3.62;
  laser_offset_[40] = 1.304 * 9 + 1.968 * 0 + 3.62;
  laser_offset_[57] = 1.304 * 9 + 1.968 * 0 + 3.62;
  laser_offset_[5] = 1.304 * 10 + 1.968 * 0 + 3.62;
  laser_offset_[53] = 1.304 * 10 + 1.968 * 0 + 3.62;
  laser_offset_[4] = 1.304 * 11 + 1.968 * 0 + 3.62;
  laser_offset_[47] = 1.304 * 11 + 1.968 * 0 + 3.62;
  laser_offset_[3] = 1.304 * 12 + 1.968 * 0 + 3.62;
  laser_offset_[49] = 1.304 * 12 + 1.968 * 0 + 3.62;
  laser_offset_[2] = 1.304 * 13 + 1.968 * 0 + 3.62;
  laser_offset_[51] = 1.304 * 13 + 1.968 * 0 + 3.62;
  laser_offset_[1] = 1.304 * 14 + 1.968 * 0 + 3.62;
  laser_offset_[45] = 1.304 * 14 + 1.968 * 0 + 3.62;
  laser_offset_[0] = 1.304 * 15 + 1.968 * 0 + 3.62;
  laser_offset_[43] = 1.304 * 15 + 1.968 * 0 + 3.62;
  laser_offset_[23] = 1.304 * 15 + 1.968 * 1 + 3.62;
  laser_offset_[32] = 1.304 * 15 + 1.968 * 1 + 3.62;
  laser_offset_[26] = 1.304 * 15 + 1.968 * 2 + 3.62;
  laser_offset_[41] = 1.304 * 15 + 1.968 * 2 + 3.62;
  laser_offset_[20] = 1.304 * 15 + 1.968 * 3 + 3.62;
  laser_offset_[35] = 1.304 * 15 + 1.968 * 3 + 3.62;
  laser_offset_[14] = 1.304 * 15 + 1.968 * 4 + 3.62;
  laser_offset_[29] = 1.304 * 15 + 1.968 * 4 + 3.62;
  laser_offset_[21] = 1.304 * 15 + 1.968 * 5 + 3.62;
  laser_offset_[36] = 1.304 * 15 + 1.968 * 5 + 3.62;
  laser_offset_[15] = 1.304 * 15 + 1.968 * 6 + 3.62;
  laser_offset_[30] = 1.304 * 15 + 1.968 * 6 + 3.62;
  laser_offset_[9] = 1.304 * 15 + 1.968 * 7 + 3.62;
  laser_offset_[24] = 1.304 * 15 + 1.968 * 7 + 3.62;
  laser_offset_[18] = 1.304 * 15 + 1.968 * 8 + 3.62;
  laser_offset_[33] = 1.304 * 15 + 1.968 * 8 + 3.62;
  laser_offset_[12] = 1.304 * 15 + 1.968 * 9 + 3.62;
  laser_offset_[27] = 1.304 * 15 + 1.968 * 9 + 3.62;
  laser_offset_[19] = 1.304 * 15 + 1.968 * 10 + 3.62;
  laser_offset_[34] = 1.304 * 15 + 1.968 * 10 + 3.62;
  laser_offset_[13] = 1.304 * 15 + 1.968 * 11 + 3.62;
  laser_offset_[28] = 1.304 * 15 + 1.968 * 11 + 3.62;
  laser_offset_[7] = 1.304 * 15 + 1.968 * 12 + 3.62;
  laser_offset_[22] = 1.304 * 15 + 1.968 * 12 + 3.62;
  laser_offset_[16] = 1.304 * 15 + 1.968 * 13 + 3.62;
  laser_offset_[31] = 1.304 * 15 + 1.968 * 13 + 3.62;
  laser_offset_[10] = 1.304 * 15 + 1.968 * 14 + 3.62;
  laser_offset_[25] = 1.304 * 15 + 1.968 * 14 + 3.62;
  laser_offset_[17] = 1.304 * 15 + 1.968 * 15 + 3.62;
  laser_offset_[37] = 1.304 * 15 + 1.968 * 15 + 3.62;
  laser_offset_[11] = 1.304 * 15 + 1.968 * 16 + 3.62;
  laser_offset_[39] = 1.304 * 15 + 1.968 * 16 + 3.62;

  for (int j = 0; j < LASER_COUNT_L64; j++) {
    elev_angle_map_[j] = pandarGeneral_elev_angle_map[j];
    horizatal_azimuth_offset_map_[j] =
        pandarGeneral_horizatal_azimuth_offset_map[j];
  }

  min_packets_ = HESAI64_MIN_PACKETS;
  max_packets_ = HESAI64_MAX_PACKETS;
}

Hesai64Parser::~Hesai64Parser() {}

void Hesai64Parser::ParseRawPacket(const uint8_t *buf, const int len,
                                   bool *is_end) {
  if (len != PACKET_SIZE_L64 && len != PACKET_SIZE_L64_WITH_UDPSEQ) {
    AWARN << "packet size:" << len
          << " mismatch internal size:" << PACKET_SIZE_L64;
    return;
  }

  // Parser Packet
  Hesai64Packet packet;
  int index = 0;
  int block = 0;
  // Parse 8 bytes header
  packet.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet.header.chLaserNumber = buf[index + 2] & 0xff;
  packet.header.chBlockNumber = buf[index + 3] & 0xff;
  packet.header.chReturnType = buf[index + 4] & 0xff;
  packet.header.chDisUnit = buf[index + 5] & 0xff;
  index += HEAD_SIZE;

  if (packet.header.sob != 0xEEFF) {
    AERROR << "error start of packet!";
    return;
  }

  for (block = 0; block < packet.header.chBlockNumber; block++) {
    packet.blocks[block].azimuth =
        (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_AZIMUTH_SIZE;

    uint8_t unit = 0;
    for (unit = 0; unit < packet.header.chLaserNumber; unit++) {
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
      packet.blocks[block].units[unit].distance =
          static_cast<double>(range * packet.header.chDisUnit) / 1000.0;
      packet.blocks[block].units[unit].reflectivity = (buf[index + 2] & 0xff);
      index += HS_LIDAR_L64_UNIT_SIZE;
    }
  }

  index += RESERVE_SIZE;
  index += ENGINE_VELOCITY_SIZE;

  packet.timestamp = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                     ((buf[index + 2] & 0xff) << 16) |
                     ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  packet.echo = buf[index] & 0xff;

  index += ECHO_SIZE;
  index += FACTORY_INFO_SIZE;

  packet.utc_time[0] = buf[index] & 0xff;
  packet.utc_time[1] = buf[index + 1] & 0xff;
  packet.utc_time[2] = buf[index + 2] & 0xff;
  packet.utc_time[3] = buf[index + 3] & 0xff;
  packet.utc_time[4] = buf[index + 4] & 0xff;
  packet.utc_time[5] = buf[index + 5] & 0xff;

  // CalcPointXYZIT
  for (int i = 0; i < packet.header.chBlockNumber; ++i) {
    int azimuthGap = 0;
    if (last_azimuth_ > packet.blocks[i].azimuth) {
      azimuthGap = static_cast<int>(packet.blocks[i].azimuth) +
                   (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(packet.blocks[i].azimuth) -
                   static_cast<int>(last_azimuth_);
    }

    if (last_azimuth_ != packet.blocks[i].azimuth && azimuthGap < 600) {
      if ((last_azimuth_ > packet.blocks[i].azimuth &&
           start_angle_ <= packet.blocks[i].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= packet.blocks[i].azimuth)) {
        *is_end = true;
      }
    }
    CalcPointXYZIT(&packet, i, packet.header.chLaserNumber);
    last_azimuth_ = packet.blocks[i].azimuth;
  }
}

void Hesai64Parser::CalcPointXYZIT(Hesai64Packet *pkt, int blockid,
                                   uint8_t chLaserNumber) {
  Hesai64Block *block = &pkt->blocks[blockid];
  struct tm tTm;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = pkt->utc_time[0] + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = pkt->utc_time[1] - 1;
  tTm.tm_mday = pkt->utc_time[2];
  tTm.tm_hour = pkt->utc_time[3];
  tTm.tm_min = pkt->utc_time[4];
  tTm.tm_sec = pkt->utc_time[5];
  tTm.tm_isdst = 0;

  double unix_second = static_cast<double>(mktime(&tTm) + tz_second_);
  double timestamp =
      unix_second + (static_cast<double>(pkt->timestamp)) / 1000000.0;

  CheckPktTime(timestamp);

  for (int i = 0; i < chLaserNumber; i++) {
    /* for all the units in a block */
    Hesai64Unit &unit = block->units[i];

    /* skip wrong distance */
    if (unit.distance < 0.1 || unit.distance > 200.0) {
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
    new_point->set_intensity(unit.reflectivity);

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

