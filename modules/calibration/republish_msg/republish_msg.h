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

/**
 * @file
 */

#ifndef MODEULES_CALIBRATION_REPUBLISH_MSG_H_
#define MODEULES_CALIBRATION_REPUBLISH_MSG_H_

#include <string>

#include "Eigen/Eigen"

#include "modules/calibration/republish_msg/proto/relative_odometry.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::calibration
 * @brief apollo::calibration
 */
namespace apollo {
namespace calibration {

class RepublishMsg : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  // Upon receiving INS status data
  void OnInsStat(const drivers::gnss::InsStat& msg);
  // Upon receiving GPS data
  void OnGps(const localization::Gps& msg);

  // Gps offset pose
  Eigen::Affine3d offset_;
  // first Gps message flag
  bool is_first_gps_msg_;
  // latest INS status
  uint32_t position_type_;
};

}  // namespace calibration
}  // namespace apollo

#endif  // MODULES_CALIBRATION_REPUBLISH_MSG_H_
