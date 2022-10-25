/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar_surestar/parser/convert.h"

#include <memory>

#include <pcl/common/time.h>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace surestar {

/** @brief Constructor. */
Convert::Convert(
    const apollo::drivers::surestar::SurestarConfig& surestar_config) {
  _config = surestar_config;
  _config.set_view_direction(0.0);
  _config.set_view_width(2.0 * M_PI);
}

bool Convert::Init() {
  _parser = SurestarParserFactory::create_parser(_config);
  if (_parser == nullptr) {
    AERROR << " can not create surestar parser";
    return false;
  }
  _parser->setup();
  return true;
}

Convert::~Convert() {
  if (_parser != nullptr) {
    delete _parser;
  }
}

uint32_t Convert::GetPointSize() { return _parser->GetPointSize(); }

/** @brief Callback for raw scan messages. */
void Convert::convert_surestar_to_pointcloud(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan const>&
        scan_msg,
    const std::shared_ptr<apollo::drivers::PointCloud>& point_cloud) {
  _parser->generate_pointcloud(scan_msg, point_cloud);

  if (point_cloud == nullptr || point_cloud->point_size() == 0) {
    AERROR << " point cloud has no point";
    return;
  }

  if (_config.organized()) {
    _parser->order(point_cloud);
    point_cloud->set_is_dense(false);
  } else {
    point_cloud->set_is_dense(true);
  }
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
