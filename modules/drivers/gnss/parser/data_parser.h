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
#include <memory>

#include "cybertron/cybertron.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"

#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"

#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss_status::GnssStatus;
using ::apollo::drivers::gnss_status::InsStatus;

using ::apollo::drivers::gnss::InsStat;
using ::apollo::drivers::gnss::GnssBestPose;
using ::apollo::drivers::gnss::GnssEphemeris;
using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::drivers::gnss::Heading;
using ::apollo::drivers::gnss::Imu;
using ::apollo::localization::CorrectedImu;
using ::apollo::drivers::gnss::Ins;
using ::apollo::localization::Gps;

using apollo::cybertron::Node;
using apollo::cybertron::Writer;
using apollo::cybertron::Reader;

class DataParser {
 public:
  explicit DataParser(const config::Config &config, const std::shared_ptr<Node>& node);
  ~DataParser() {}
  bool Init();
  void ParseRawData(const std::string& msg);

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
//  void GpsToTransformStamped(const ::apollo::localization::Gps &gps,
//                             geometry_msgs::TransformStamped *transform);

  bool inited_flag_ = false;
  config::Config config_;
  std::unique_ptr<Parser> data_parser_;
  cybertron::tf2_cybertron::TransformBroadcaster tf_broadcaster_;

  GnssStatus gnss_status_;
  InsStatus ins_status_;
  uint32_t ins_status_record_ = static_cast<uint32_t>(0);
  projPJ wgs84pj_source_;
  projPJ utm_target_;

  std::shared_ptr<Node> node_ = nullptr;
  std::shared_ptr<Writer<GnssStatus>> gnssstatus_writer_ = nullptr;
  std::shared_ptr<Writer<InsStatus>> insstatus_writer_ = nullptr;
  std::shared_ptr<Writer<Gnss>> gnss_writer_ = nullptr;
  std::shared_ptr<Writer<GnssBestPose>> gnssbestpose_writer_ = nullptr;
  std::shared_ptr<Writer<CorrectedImu>> corrimu_writer_ = nullptr;
  std::shared_ptr<Writer<Imu>> rawimu_writer_ = nullptr;
  std::shared_ptr<Writer<Gps>> gps_writer_ = nullptr;
  std::shared_ptr<Writer<Ins>> ins_writer_ = nullptr;
  std::shared_ptr<Writer<InsStat>> insstat_writer_ = nullptr;
  std::shared_ptr<Writer<GnssEphemeris>> gnssephemeris_writer_ = nullptr;
  std::shared_ptr<Writer<EpochObservation>> epochobservation_writer_ = nullptr;
  std::shared_ptr<Writer<Heading>> heading_writer_ = nullptr;

};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_DATA_PARSER_H_
