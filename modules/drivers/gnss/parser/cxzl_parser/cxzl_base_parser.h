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

enum CxzlSoluStatusT : int {
  INIT_STATUS = 0,
  SATELLITE_NAVIGATION = 1,
  INTEGRATED_NAVIGATION = 2,
  INERTIAL_NAVIGATION = 3,
};

enum CxzlSatelliteStatusT : int {
  POS_NONE = 0,
  POS_LOCATEDIRECT_SINGLE = 1,
  POS_LOCATEDIRECT_DIFF = 2,
  POS_LOCATEDIRECT_COMB = 3,
  POS_LOCATEDIRECT_FIXED = 4,
  POS_LOCATEDIRECT_FLOAT = 5,
  POS_LOCATE_SINGLE = 6,
  POS_LOCATE_DIFF = 7,
  POS_LOCATE_FIXED = 8,
  POS_LOCATE_FLOAT = 9,
};

struct CxzlMessage {
  int GPSWeek;
  double GPSTime;

  double GyroX;
  double GyroY;
  double GyroZ;
  double AccX;
  double AccY;
  double AccZ;

  CxzlSoluStatusT Solution_status;
  int NSV1;
  CxzlSatelliteStatusT Satellite_status;
  int NSV2;
  double Age;
  int NSVD1;
  int NSVD2;
  
  double Altitude;

  double Pe_std = 0.0;
  double Pn_std = 0.0;
  double Pu_std = 0.0;

  double Ve;
  double Vn;
  double Vu;
  double V;
  double vn_std = 0.0;
  double ve_std = 0.0;
  double vu_std = 0.0;
  double v_std = 0.0;

  double Veh_AccX;
  double Veh_AccY;
  double Veh_AccZ;

  double Roll;
  double Pitch;
  double Heading;
  double roll_std = 0.0;
  double pitch_std = 0.0;
  double heading_std = 0.0;
  double Veh_GyroX;
  double Veh_GyroY;
  double Veh_GyroZ;

  double Longitude;
  double Latitude;

  unsigned char check;
  double gps_timestamp_sec = 0;
};

struct CxzlProtocol {
  std::string cxhead = "$CXINSPVA";
  size_t CXZL_SIZE = 42;
};


class CxzlBaseParser : public Parser {
 public:
  CxzlBaseParser() {}
  explicit CxzlBaseParser(const config::Config &config);

  virtual bool PrepareMessage();

  virtual void GetMessages(MessageInfoVec *messages);

 protected:
  void PrepareMessageStatus(const uint8_t &system_state,
                            const uint8_t &satellite_status);
  CxzlMessage decode_message_;

 private:
  void FillGnssBestpos();
  void FillIns();
  void FillInsStat();
  void FillImu();
  void FillHeading();
  void PrepareMessageCxzl(const std::vector<std::string> &fields);

  GnssBestPose bestpos_;
  // bestpos 1hz rate control
  RateControl bestpos_ratecontrol_{PERIOD_NS_1HZ};
  Imu imu_;
  Heading heading_;
  Ins ins_;
  InsStat ins_stat_;

  std::string input_str;
  CxzlProtocol protocol_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
