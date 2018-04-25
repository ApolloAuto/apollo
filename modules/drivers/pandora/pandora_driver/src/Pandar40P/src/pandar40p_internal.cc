/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <sstream>

#include "src/input.h"
#include "src/pandar40p_internal.h"

namespace apollo {
namespace drivers {
namespace hesai {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float pandar40p_elev_angle_map[] = {
    6.96,   5.976,  4.988,   3.996,   2.999,   2.001,  1.667,   1.333,
    1.001,  0.667,  0.333,   0,       -0.334,  -0.667, -1.001,  -1.334,
    -1.667, -2.001, -2.331,  -2.667,  -3,      -3.327, -3.663,  -3.996,
    -4.321, -4.657, -4.986,  -5.311,  -5.647,  -5.974, -6.957,  -7.934,
    -8.908, -9.871, -10.826, -11.772, -12.705, -13.63, -14.543, -15.444};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float pandar40p_horizatal_azimuth_offset_map[] = {
    0.005,  0.006,  0.006,  0.006,  -2.479, -2.479, 2.491,  -4.953,
    -2.479, 2.492,  -4.953, -2.479, 2.492,  -4.953, 0.007,  2.491,
    -4.953, 0.006,  4.961,  -2.479, 0.006,  4.96,   -2.478, 0.006,
    4.958,  -2.478, 2.488,  4.956,  -2.477, 2.487,  2.485,  2.483,
    0.004,  0.004,  0.003,  0.003,  -2.466, -2.463, -2.46,  -2.457};

Pandar40P_Internal::Pandar40P_Internal(
    std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    std::string frame_id) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  start_angle_ = start_angle > 36000 ? 0 : start_angle;

  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
    cos_lookup_table_[rotIndex] = cosf(rotation);
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }

  input_.reset(new Input(lidar_port, gps_port));

  pcl_callback_ = pcl_callback;
  gps_callback_ = gps_callback;

  // init the block time offset, us
  blockOffset_[9] = 55.1f * 0.0 + 45.18f;
  blockOffset_[8] = 55.1f * 1.0 + 45.18f;
  blockOffset_[7] = 55.1f * 2.0 + 45.18f;
  blockOffset_[6] = 55.1f * 3.0 + 45.18f;
  blockOffset_[5] = 55.1f * 4.0 + 45.18f;
  blockOffset_[4] = 55.1f * 5.0 + 45.18f;
  blockOffset_[3] = 55.1f * 6.0 + 45.18f;
  blockOffset_[2] = 55.1f * 7.0 + 45.18f;
  blockOffset_[1] = 55.1f * 8.0 + 45.18f;
  blockOffset_[0] = 55.1f * 9.0 + 45.18f;

  // init the laser shot time offset, us
  laserOffset_[3] = 0.93f * 1.0f;
  laserOffset_[35] = 0.93f * 2.0f;
  laserOffset_[39] = 0.93f * 3.0f;
  laserOffset_[23] = 0.93f * 3.0f + 1.6f * 1.0f;
  laserOffset_[16] = 0.93f * 3.0f + 1.6f * 2.0f;
  laserOffset_[27] = 0.93f * 4.0f + 1.6f * 2.0f;
  laserOffset_[11] = 0.93f * 4.0f + 1.6f * 3.0f;
  laserOffset_[31] = 0.93f * 5.0f + 1.6f * 3.0f;
  laserOffset_[28] = 0.93f * 6.0f + 1.6f * 3.0f;
  laserOffset_[15] = 0.93f * 6.0f + 1.6f * 4.0f;
  laserOffset_[2] = 0.93f * 7.0f + 1.6f * 4.0f;
  laserOffset_[34] = 0.93f * 8.0f + 1.6f * 4.0f;
  laserOffset_[38] = 0.93f * 9.0f + 1.6f * 4.0f;
  laserOffset_[20] = 0.93f * 9.0f + 1.6f * 5.0f;
  laserOffset_[13] = 0.93f * 9.0f + 1.6f * 6.0f;
  laserOffset_[24] = 0.93f * 9.0f + 1.6f * 7.0f;
  laserOffset_[8] = 0.93f * 9.0f + 1.6f * 8.0f;
  laserOffset_[30] = 0.93f * 10.0f + 1.6f * 8.0f;
  laserOffset_[25] = 0.93f * 11.0f + 1.6f * 8.0f;
  laserOffset_[12] = 0.93f * 11.0f + 1.6f * 9.0f;
  laserOffset_[1] = 0.93f * 12.0f + 1.6f * 9.0f;
  laserOffset_[33] = 0.93f * 13.0f + 1.6f * 9.0f;
  laserOffset_[37] = 0.93f * 14.0f + 1.6f * 9.0f;
  laserOffset_[17] = 0.93f * 14.0f + 1.6f * 10.0f;
  laserOffset_[10] = 0.93f * 14.0f + 1.6f * 11.0f;
  laserOffset_[21] = 0.93f * 14.0f + 1.6f * 12.0f;
  laserOffset_[5] = 0.93f * 14.0f + 1.6f * 13.0f;
  laserOffset_[29] = 0.93f * 15.0f + 1.6f * 13.0f;
  laserOffset_[22] = 0.93f * 15.0f + 1.6f * 14.0f;
  laserOffset_[9] = 0.93f * 15.0f + 1.6f * 15.0f;
  laserOffset_[0] = 0.93f * 16.0f + 1.6f * 15.0f;
  laserOffset_[32] = 0.93f * 17.0f + 1.6f * 15.0f;
  laserOffset_[36] = 0.93f * 18.0f + 1.6f * 15.0f;
  laserOffset_[14] = 0.93f * 18.0f + 1.6f * 16.0f;
  laserOffset_[7] = 0.93f * 18.0f + 1.6f * 17.0f;
  laserOffset_[18] = 0.93f * 18.0f + 1.6f * 18.0f;
  laserOffset_[4] = 0.93f * 19.0f + 1.6f * 18.0f;
  laserOffset_[26] = 0.93f * 20.0f + 1.6f * 18.0f;
  laserOffset_[19] = 0.93f * 20.0f + 1.6f * 19.0f;
  laserOffset_[6] = 0.93f * 20.0f + 1.6f * 20.0f;

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the laser offset */
    elev_angle_map_[i] = pandar40p_elev_angle_map[i];
    horizatal_azimuth_offset_map_[i] =
        pandar40p_horizatal_azimuth_offset_map[i];
  }

  frame_id_ = frame_id;
  tz_second_ = tz * 3600;
}

Pandar40P_Internal::~Pandar40P_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int Pandar40P_Internal::LoadCorrectionFile(std::string correction_content) {
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  double azimuthOffset[LASER_COUNT];
  double elev_angle[LASER_COUNT];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    if (lineCounter++ >= LASER_COUNT) break;

    int lineId = 0;
    double elev, azimuth;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (lineId != lineCounter) {
      return -1;
    }

    elev_angle[lineId - 1] = elev;
    azimuthOffset[lineId - 1] = azimuth;
  }

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the laser offset */
    elev_angle_map_[i] = elev_angle[i];
    horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }

  return 0;
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void Pandar40P_Internal::ResetStartAngle(uint16_t start_angle) {
  start_angle_ = start_angle;
}

int Pandar40P_Internal::Start() {
  Stop();
  enable_lidar_recv_thr_ = true;
  enable_lidar_process_thr_ = true;
  lidar_process_thr_ = new boost::thread(
      boost::bind(&Pandar40P_Internal::ProcessLiarPacket, this));
  lidar_recv_thr_ =
      new boost::thread(boost::bind(&Pandar40P_Internal::RecvTask, this));
}

void Pandar40P_Internal::Stop() {
  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }
  return;
}

void Pandar40P_Internal::RecvTask() {
  int ret = 0;
  while (enable_lidar_recv_thr_) {
    PandarPacket pkt;
    int rc = input_->getPacket(&pkt);
    if (rc == -1) {
      continue;
    }

    if (pkt.size == GPS_PACKET_SIZE) {
      PandarGPS gpsMsg;
      ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
      if (ret == 0) {
        ProcessGps(gpsMsg);
      }
      continue;
    }

    PushLiDARData(pkt);
  }
}

void Pandar40P_Internal::ProcessLiarPacket() {
  struct timespec ts;
  int ret = 0;
  static int packetIndex = 0;

  boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());

  while (enable_lidar_process_thr_) {
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      std::cout << "get time error" << std::endl;
    }

    ts.tv_sec += 1;
    if (sem_timedwait(&lidar_sem_, &ts) == -1) {
      continue;
    }

    pthread_mutex_lock(&lidar_lock_);
    PandarPacket packet = lidar_packets_.front();
    lidar_packets_.pop_front();
    pthread_mutex_unlock(&lidar_lock_);

    if (packet.size == PACKET_SIZE) {
      Pandar40PPacket pkt;
      ret = ParseRawData(&pkt, packet.data, packet.size);
      if (ret != 0) {
        continue;
      }
      // increate pakcet_index
      packetIndex++;

      for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
        /* ready a round ? */
        uint16_t current_azimuth = pkt.blocks[i].azimuth;
        int limit_degree = RATE_PER_PACKET * 100/2;
        int gap = std::min(std::abs(current_azimuth - start_angle_),
                36000 - std::abs(current_azimuth - start_angle_));

        // at lease 150 packets, no more than 182
        if ((packetIndex > PACKETS_PER_ROUND * 5/6 && gap <= limit_degree) ||
                packetIndex > PACKETS_PER_ROUND + 1) {
          // ok
          if (pcl_callback_ && outMsg->points.size() > 0) {
            // std::cout << std::fixed << std::setprecision(18)
            //    << "Size: " << packetIndex << ", current: " << current_azimuth
            //    << ", timestamp_: " << timestamp_ << std::endl;
            pcl_callback_(outMsg, timestamp_);
            outMsg.reset(new PPointCloud());
            // reset
            packetIndex = 0; timestamp_ = 0;
          }
        }

        CalcPointXYZIT(&pkt, i, outMsg);
      }
    } else {
      continue;
    }

    outMsg->header.frame_id = frame_id_;
    outMsg->height = 1;
  }
}

void Pandar40P_Internal::PushLiDARData(PandarPacket packet) {
  pthread_mutex_lock(&lidar_lock_);
  lidar_packets_.push_back(packet);
  sem_post(&lidar_sem_);
  pthread_mutex_unlock(&lidar_lock_);
}

void Pandar40P_Internal::ProcessGps(const PandarGPS &gpsMsg) {
  struct tm t;
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;

  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;

  // UTC's month start from 1, but mktime only accept month from 0.
  t.tm_mon = gpsMsg.month - 1;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  t.tm_year = gpsMsg.year + 100;
  t.tm_isdst = 0;

  if (gps_callback_) {
    gps_callback_(static_cast<double>(mktime(&t) + tz_second_) + 1);
  }
}

int Pandar40P_Internal::ParseRawData(Pandar40PPacket *packet,
                                     const uint8_t *buf, const int len) {
  if (len != PACKET_SIZE) {
    std::cout << "packet size mismatch Pandar40P_Internal " << len << ","
              << PACKET_SIZE << std::endl;
    return -1;
  }

  int index = 0;
  // 10 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    Pandar40PBlock &block = packet->blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x units
    for (int j = 0; j < LASER_COUNT; j++) {
      Pandar40PUnit &unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      // distance is M.
      unit.distance =
          (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
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

  packet->usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  packet->usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet->echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  // parse the UTC Time.

  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet->t.tm_year = (buf[index + 0] & 0xff) + 100;
  // UTC's month start from 1, but mktime only accept month from 0.
  packet->t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet->t.tm_mday = buf[index + 2] & 0xff;
  packet->t.tm_hour = buf[index + 3] & 0xff;
  packet->t.tm_min = buf[index + 4] & 0xff;
  packet->t.tm_sec = buf[index + 5] & 0xff;
  packet->t.tm_isdst = 0;

  return 0;
}

int Pandar40P_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf,
                                 const int size) {
  if (size != GPS_PACKET_SIZE) {
    return -1;
  }
  int index = 0;
  packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  packet->year =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  packet->month =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  packet->day =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  packet->second =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  packet->minute =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  packet->hour =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  packet->fineTime =
      (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
#ifdef DEBUG
  if (packet->year != 18) {
    printf("error gps\n");
    char str[128];
    int fd = open("/var/tmp/error_gps.txt", O_RDWR | O_CREAT, 0666);
    lseek(fd, 0, SEEK_END);
    int i = 0;
    for (i = 0; i < 512; i++) {
      snprintf(str, "%02x ", recvbuf[i], 127);
      write(fd, str, strlen(str));
    }
    write(fd, "\n", 1);
    close(fd);
  }
#endif
  return 0;
}

void Pandar40P_Internal::CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                                        boost::shared_ptr<PPointCloud> cld) {
  Pandar40PBlock *block = &pkt->blocks[blockid];

  double unix_second =
      static_cast<double>(mktime(&pkt->t) + 1 + tz_second_);  // 1 second offset

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the units in a block */
    Pandar40PUnit &unit = block->units[i];
    PPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.5 || unit.distance > 200.0) {
      continue;
    }

    double xyDistance =
        unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
    point.x = static_cast<float>(
        xyDistance *
        sinf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.y = static_cast<float>(
        xyDistance *
        cosf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
                            (static_cast<double>(block->azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance *
                                 sinf(degreeToRadian(elev_angle_map_[i])));

    // point.x = static_cast<float>(xyDistance *
    // sin_lookup_table_[xylookup_id]);
    // point.y = static_cast<float>(xyDistance *
    // cos_lookup_table_[xylookup_id]);
    // point.z = static_cast<float>(unit.distance *
    // sin_lookup_table_[zlookup_id]);

    point.intensity = unit.intensity;

    point.timestamp =
        unix_second + (static_cast<double>(pkt->usec)) / 1000000.0;

    if (pkt->echo == 0x39) {
      // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
      point.timestamp =
          point.timestamp - (static_cast<double>(blockOffset_[blockid / 2] +
                                                 laserOffset_[i / 2]) /
                             1000000.0f);
    } else {
      point.timestamp =
          point.timestamp -
          (static_cast<double>(blockOffset_[blockid] + laserOffset_[i]) /
           1000000.0f);
    }

    point.ring = i;
    // get smallest timestamp
    if ((timestamp_ > 0 && timestamp_ < point.timestamp)
            || timestamp_ <= 0) {
      timestamp_ = point.timestamp;
    }

    cld->push_back(point);
  }
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
