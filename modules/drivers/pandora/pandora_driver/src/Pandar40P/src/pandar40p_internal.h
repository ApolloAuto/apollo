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

#ifndef SRC_PANDAR40P_INTERNAL_H_
#define SRC_PANDAR40P_INTERNAL_H_

#include <boost/function.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>

#include "pandar40p/point_types.h"
#include "src/input.h"

#define RATE_PER_PACKET (2)
#define PACKETS_PER_ROUND (360 / RATE_PER_PACKET)
#define SOB_ANGLE_SIZE (4)
#define RAW_MEASURE_SIZE (3)
#define LASER_COUNT (40)
#define BLOCKS_PER_PACKET (10)
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE (4)
#define FACTORY_INFO_SIZE (1)
#define ECHO_SIZE (1)
#define RESERVE_SIZE (8)
#define REVOLUTION_SIZE (2)
#define INFO_SIZE                                                  \
  (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + RESERVE_SIZE + \
  REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)

#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define HesaiLidarSDK_DEFAULT_GPS_RECV_PORT 10110

struct Pandar40PUnit_s {
  uint8_t intensity;
  double distance;
};
typedef struct Pandar40PUnit_s Pandar40PUnit;

struct Pandar40PBlock_s {
  uint16_t azimuth;
  uint16_t sob;
  Pandar40PUnit units[LASER_COUNT];
};
typedef struct Pandar40PBlock_s Pandar40PBlock;

struct Pandar40PPacket_s {
  Pandar40PBlock blocks[BLOCKS_PER_PACKET];
  struct tm t;
  uint32_t usec;
  int echo;
};
typedef struct Pandar40PPacket_s Pandar40PPacket;

struct PandarGPS_s {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
};
typedef struct PandarGPS_s PandarGPS;

#define ROTATION_MAX_UNITS (36001)

namespace apollo {
namespace drivers {
namespace hesai {

class Pandar40P_Internal {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        type       				The device type
   */
  Pandar40P_Internal(
      std::string device_ip, uint16_t lidar_port, uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
      boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
      std::string frame_id);

  /**
   * @brief load the correction file
   * @param correction The path of correction file
   */
  int LoadCorrectionFile(std::string correction);

  /**
   * @brief load the correction file
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  ~Pandar40P_Internal();

  int Start();
  void Stop();

 private:
  void RecvTask();
  void ProcessGps(const PandarGPS &gpsMsg);
  void ProcessLiarPacket();
  void PushLiDARData(PandarPacket packet);
  int ParseRawData(Pandar40PPacket *packet, const uint8_t *buf, const int len);
  int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);
  void CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                      boost::shared_ptr<PPointCloud> cld);

  pthread_mutex_t lidar_lock_;
  sem_t lidar_sem_;
  boost::thread *lidar_recv_thr_;
  boost::thread *lidar_process_thr_;
  bool enable_lidar_recv_thr_;
  bool enable_lidar_process_thr_;
  int start_angle_ = 0;
  double timestamp_ = 0;

  std::list<struct PandarPacket_s> lidar_packets_;

  boost::shared_ptr<Input> input_;
  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)>
      pcl_callback_;
  boost::function<void(double timestamp)> gps_callback_;

  float sin_lookup_table_[ROTATION_MAX_UNITS];
  float cos_lookup_table_[ROTATION_MAX_UNITS];

  float elev_angle_map_[LASER_COUNT];
  float horizatal_azimuth_offset_map_[LASER_COUNT];

  float blockOffset_[BLOCKS_PER_PACKET];
  float laserOffset_[LASER_COUNT];

  int tz_second_;
  std::string frame_id_;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif  // SRC_PANDAR40P_INTERNAL_H_
