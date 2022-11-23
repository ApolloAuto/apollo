/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#pragma once
#include <errno.h>
#include <stdint.h>

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "boost/format.hpp"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/robosense/proto/lidars_filter_config.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng.pb.h"
#include "modules/drivers/lidar/robosense/proto/sensor_suteng_conf.pb.h"

#include "modules/drivers/lidar/robosense/lib/calibration.h"
#include "modules/drivers/lidar/robosense/lib/const_variables.h"
#include "modules/drivers/lidar/robosense/lib/data_type.h"
#include "modules/drivers/lidar/robosense/parser/robosense_parser.h"

namespace apollo {
namespace drivers {
namespace robosense {

class Robosense16PParser : public RobosenseParser {
 public:
  explicit Robosense16PParser(
      const apollo::drivers::suteng::SutengConfig& config);
  ~Robosense16PParser() {}

  void generate_pointcloud(
      const std::shared_ptr<apollo::drivers::suteng::SutengScan const>&
          scan_msg,
      const std::shared_ptr<apollo::drivers::PointCloud>& out_msg);
  void order(const std::shared_ptr<apollo::drivers::PointCloud>& cloud) {
    int width = 16;
    cloud->set_width(width);
    int height = cloud->point_size() / cloud->width();
    cloud->set_height(height);
  }
  uint32_t GetPointSize() override { return POINT_SIZE; };
  void setup() override;

 private:
#pragma pack(push, 1)
  static constexpr uint32_t POINT_SIZE = 28800;
  static constexpr float DISTANCE_RESOLUTION_16P = 0.0025f;
  /**< meters */  // 16p
  static constexpr int SCANS_PER_FIRING = 16;
  static constexpr int FIRINGS_PER_BLOCK = 2;
  static constexpr float DEGREE_TO_RADIAN =
      M_PI / 18000.f;  // 16P resolution: 0.01 degree
  float cor_hori_angles[SCANS_PER_FIRING];
  float cor_vert_angles[SCANS_PER_FIRING];
  uint64_t last_difop_time_ = 0;
  // lens center
  static constexpr float RX_16P = 0.03498f;
  static constexpr float RY_16P = -0.015f;
  static constexpr float RZ_16P = 0.0f;

  typedef struct channel_data_16p {
    uint16_t distance;
    uint8_t reflectivity;
  } channel_data_16p_t;

  typedef struct raw_block_16p {
    uint16_t header;  // UPPER_BANK or LOWER_BANK
    uint16_t rotation;
    channel_data_16p_t channel_data[SCANS_PER_BLOCK];
  } raw_block_16p_t;

  typedef struct raw_msop_packet_16p {
    raw_block_16p_t blocks[BLOCKS_PER_PACKET];
    uint8_t unused[4];
    uint16_t flag;
  } raw_packet_16p_t;

  typedef struct utc_time_16p {
    uint8_t sec[6];
    uint8_t us[4];
  } utc_time_16p_t;  // offset: 303

  typedef struct angle_16p {
    uint8_t sign;    // 0x00: +, 0x01:-, 0xff: err
    uint16_t value;  // big endian, resolution: 0.01
  } angle_16p_t;

  typedef struct vertical_angles_16p {
    angle_16p_t angles[16];
  } vertical_angles_16p_t;  // offset: 468

  typedef struct horizontal_angles_16p {
    angle_16p_t angles[16];
  } horizontal_angles_16p_t;  // offset: 564

#pragma pack(pop)

  uint64_t get_timestamp(double base_time, float time_offset,
                         uint16_t laser_block_id) {
    return 0;
  }
  void unpack_params(const apollo::drivers::suteng::SutengPacket& pkt);
  float parse_angle(angle_16p_t angle_pkt);
  void unpack_robosense(
      const apollo::drivers::suteng::SutengPacket& pkt,
      const std::shared_ptr<apollo::drivers::PointCloud>& cloud,
      uint32_t* index);
  void init_params();

  static bool pkt_start;
  static uint64_t base_stamp;

  float SUTENG_VERT_16P_[16] = {
      0.22689280275926285,   0.2617993877991494,   0.15707963267948966,
      0.19198621771937624,   0.08726646259971647,  0.12217304763960307,
      0.017453292519943295,  0.05235987755982989,  -0.05235987755982989,
      -0.017453292519943295, -0.12217304763960307, -0.08726646259971647,
      -0.19198621771937624,  -0.15707963267948966, -0.2617993877991494,
      -0.22689280275926285};

  // unit: us
  const float TIME_OFFSET_16P[32][12] = {
      {0.00, 111.11, 222.22, 333.33, 444.44, 555.56, 666.67, 777.78, 888.89,
       1000.00, 1111.11, 1222.23},
      {3.15, 114.26, 225.37, 336.48, 447.59, 558.70, 669.81, 780.92, 892.03,
       1003.14, 1114.25, 1225.36},
      {6.30, 117.41, 228.52, 339.63, 450.74, 561.85, 672.96, 784.07, 895.18,
       1006.29, 1117.40, 1228.51},
      {9.45, 120.56, 231.67, 342.78, 453.89, 565.00, 676.11, 787.22, 898.33,
       1009.44, 1120.55, 1231.66},
      {13.26, 124.38, 235.49, 346.60, 457.71, 568.82, 679.94, 791.05, 902.16,
       1013.27, 1124.38, 1235.49},
      {17.08, 128.19, 239.30, 350.41, 461.52, 572.64, 683.75, 794.86, 905.97,
       1017.08, 1128.19, 1239.31},
      {20.56, 131.67, 242.78, 353.90, 465.01, 576.12, 687.23, 798.34, 909.46,
       1020.57, 1131.68, 1242.79},
      {23.71, 134.82, 245.93, 357.05, 468.16, 579.27, 690.38, 801.49, 912.61,
       1023.72, 1134.83, 1245.94},
      {26.53, 137.64, 248.75, 359.86, 470.97, 582.08, 693.19, 804.30, 915.41,
       1026.52, 1137.63, 1248.74},
      {27.77, 138.88, 249.99, 361.10, 472.21, 583.32, 694.43, 805.54, 916.65,
       1027.76, 1138.87, 1249.98},
      {31.49, 142.60, 253.72, 364.83, 475.94, 587.05, 698.16, 809.28, 920.39,
       1031.50, 1142.61, 1253.72},
      {32.73, 143.85, 254.96, 366.07, 477.18, 588.29, 699.41, 810.52, 921.63,
       1032.74, 1143.85, 1254.96},
      {36.46, 147.57, 258.68, 369.79, 480.90, 592.01, 703.12, 814.23, 925.34,
       1036.45, 1147.56, 1258.67},
      {38.94, 150.05, 261.16, 372.27, 483.39, 594.50, 705.61, 816.72, 927.83,
       1038.95, 1150.07, 1261.18},
      {41.42, 152.54, 263.65, 374.76, 485.87, 596.98, 708.10, 819.21, 930.32,
       1041.43, 1152.54, 1263.65},
      {43.91, 155.02, 266.13, 377.24, 488.35, 599.46, 710.57, 821.68, 932.79,
       1043.90, 1155.01, 1266.12},
      {55.56, 166.67, 277.78, 388.89, 500.00, 611.11, 722.22, 833.33, 944.44,
       1055.55, 1166.66, 1277.77},
      {58.70, 169.82, 280.93, 392.04, 503.15, 614.26, 725.38, 836.49, 947.60,
       1058.71, 1169.82, 1280.93},
      {61.85, 172.97, 284.08, 395.19, 506.30, 617.41, 728.53, 839.64, 950.75,
       1061.86, 1172.97, 1284.08},
      {65.00, 176.11, 287.23, 398.34, 509.45, 620.56, 731.67, 842.79, 953.90,
       1065.01, 1176.12, 1287.23},
      {68.82, 179.93, 291.04, 402.15, 513.26, 624.38, 735.49, 846.60, 957.71,
       1068.82, 1179.93, 1291.05},
      {72.64, 183.75, 294.86, 405.97, 517.08, 628.19, 739.30, 850.41, 961.52,
       1072.63, 1183.74, 1294.85},
      {76.12, 187.23, 298.34, 409.45, 520.56, 631.67, 742.78, 853.89, 965.00,
       1076.11, 1187.22, 1298.33},
      {79.27, 190.38, 301.49, 412.60, 523.71, 634.82, 745.93, 857.04, 968.15,
       1079.26, 1190.37, 1301.48},
      {82.08, 193.19, 304.31, 415.42, 526.53, 637.64, 748.75, 859.87, 970.98,
       1082.09, 1193.20, 1304.31},
      {83.32, 194.44, 305.55, 416.66, 527.77, 638.88, 750.00, 861.11, 972.22,
       1083.33, 1194.44, 1305.55},
      {87.05, 198.16, 309.27, 420.38, 531.49, 642.60, 753.71, 864.82, 975.93,
       1087.04, 1198.15, 1309.26},
      {88.29, 199.40, 310.51, 421.62, 532.73, 643.85, 754.96, 866.07, 977.18,
       1088.29, 1199.40, 1310.52},
      {92.01, 203.13, 314.24, 425.35, 536.46, 647.57, 758.69, 869.80, 980.91,
       1092.02, 1203.13, 1314.24},
      {94.50, 205.61, 316.72, 427.83, 538.94, 650.05, 761.16, 872.27, 983.38,
       1094.49, 1205.60, 1316.71},
      {96.98, 208.09, 319.20, 430.31, 541.42, 652.54, 763.65, 874.76, 985.87,
       1096.98, 1208.09, 1319.21},
      {99.46, 210.57, 321.68, 432.80, 543.91, 655.02, 766.13, 877.24, 988.36,
       1099.47, 1210.58, 1321.69}};
};

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
