/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <string>

#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/parser_common.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

struct HuaCeMessage {
  std::string messageID;
  int GPSWeek;
  double GPSTime;
  double Heading;
  double Pitch;
  double Roll;
  double GyroX;
  double GyroY;
  double GyroZ;
  double AccX;
  double AccY;
  double AccZ;
  double Latitude;
  double Longitude;
  double Altitude;
  double Ve;
  double Vn;
  double Vu;
  double V;
  int NSV1;
  int NSV2;
  SolutionStatus solution_status;
  SolutionType solution_type;
  double Age;
  std::string WarningCs;
  uint8_t satellites_num = 0;
  uint8_t wheel_speed_status = 0;

  double gps_timestamp_sec = 0;
  // 数据标准差
  double lat_std = 0.0;
  double lon_std = 0.0;
  double alti_std = 0.0;
  double vn_std = 0.0;
  double ve_std = 0.0;
  double vu_std = 0.0;
  double v_std = 0.0;

  double roll_std = 0.0;
  double pitch_std = 0.0;
  double yaw_std = 0.0;
};

class HuaCeBaseParser : public Parser {
 public:
  HuaCeBaseParser() {}
  explicit HuaCeBaseParser(const config::Config &config);

  virtual bool PrepareMessage() = 0;

  virtual void GetMessages(MessageInfoVec *messages);

 protected:
  void PrepareMessageStatus(const uint8_t &system_state,
                            const uint8_t &satellite_status);
  HuaCeMessage decode_message_;

 private:
  void FillGnssBestpos();
  void FillIns();
  void FillInsStat();
  void FillImu();
  void FillHeading();

  GnssBestPose bestpos_;
  // bestpos 1hz rate control
  RateControl bestpos_ratecontrol_{PERIOD_NS_1HZ};
  Imu imu_;
  Heading heading_;
  Ins ins_;
  InsStat ins_stat_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
