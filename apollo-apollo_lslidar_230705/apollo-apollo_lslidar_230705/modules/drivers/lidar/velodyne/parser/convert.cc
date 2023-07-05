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

#include "modules/drivers/lidar/velodyne/parser/convert.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::PointCloud;
using apollo::drivers::velodyne::VelodyneScan;

void Convert::init(const Config& velodyne_config) {
  config_ = velodyne_config;
  // we use Beijing time by default

  parser_.reset(VelodyneParserFactory::CreateParser(config_));
  if (parser_.get() == nullptr) {
    AFATAL << "Create parser failed.";
    return;
  }
  parser_->setup();
}

/** @brief Callback for raw scan messages. */
void Convert::ConvertPacketsToPointcloud(
    const std::shared_ptr<VelodyneScan>& scan_msg,
    std::shared_ptr<PointCloud> point_cloud) {
  ADEBUG << "Convert scan msg seq " << scan_msg->header().sequence_num();

  parser_->GeneratePointcloud(scan_msg, point_cloud);

  if (point_cloud == nullptr || point_cloud->point().empty()) {
    AERROR << "point cloud has no point";
    return;
  }

  if (config_.organized()) {
    parser_->Order(point_cloud);
    point_cloud->set_is_dense(false);
  } else {
    point_cloud->set_is_dense(true);
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
