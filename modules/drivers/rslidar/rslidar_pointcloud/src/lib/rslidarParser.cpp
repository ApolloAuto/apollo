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

#include "rslidar_pointcloud/rslidarParser.h"

#include <pcl/common/time.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "rslidar_pointcloud/util.h"

namespace apollo {
namespace drivers {
namespace rslidar {

void rslidarParser::sed_out_passage(){
	ROS_INFO_ONCE("start");
}

rslidarParser *RslidarParserFactory::create_parser(Config_p config) {

  if (config.model == "RS16") {
    return new rslidar16Parser();
  } else if (config.model == "RS32") {
   // config.calibration_online = false;
    return new rslidar32Parser();
  } else {
    ROS_ERROR_STREAM(
        "invalid model, must be 64E_S2|64E_S3S"
        << "|64E_S3D_STRONGEST|64E_S3D_LAST|64E_S3D_DUAL|HDL32E|VLP16");
    return nullptr;
  }
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
