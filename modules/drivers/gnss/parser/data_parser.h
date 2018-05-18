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

#ifndef MODULES_DRIVERS_GNSS_DATA_PARSER_H_
#define MODULES_DRIVERS_GNSS_DATA_PARSER_H_


#include <proj_api.h>
#include <std_msgs/String.h>
#include <memory>
#include "ros/include/geometry_msgs/TransformStamped.h"
#include "ros/include/ros/ros.h"
#include "ros/include/tf2_ros/transform_broadcaster.h"

#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/proto/config.pb.h"

#include "modules/drivers/gnss//proto/ins.pb.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"

#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/localization/proto/gps.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss_status::GnssStatus;
using ::apollo::drivers::gnss_status::InsStatus;

class DataParser {
 public:
  explicit DataParser(const config::Config &config);
  ~DataParser() {}
  bool Init();
  void ParseRawData(const std_msgs::String::ConstPtr &msg);

 private:
  void DispatchMessage(Parser::MessageType type, MessagePtr message);
  void PublishInsStat(const MessagePtr message);
  void PublishOdometry(const MessagePtr message);
  void PublishCorrimu(const MessagePtr message);
  void PublishImu(const MessagePtr message);
  void PublishBestpos(const MessagePtr message);
  void PublishEphemeris(const MessagePtr message);
  void PublishObservation(const MessagePtr message);
  void PublishHeading(const MessagePtr message);
  void CheckInsStatus(drivers::gnss::Ins *ins);
  void CheckGnssStatus(drivers::gnss::Gnss *gnss);
  void GpsToTransformStamped(const ::apollo::localization::Gps &gps,
                             geometry_msgs::TransformStamped *transform);

  bool inited_flag_ = false;
  config::Config config_;
  std::unique_ptr<Parser> data_parser_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  GnssStatus gnss_status_;
  InsStatus ins_status_;
  uint32_t ins_status_record_ = static_cast<uint32_t>(0);
  projPJ wgs84pj_source_;
  projPJ utm_target_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_DATA_PARSER_H_
