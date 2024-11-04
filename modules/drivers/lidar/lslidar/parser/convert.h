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

#pragma once

#include <memory>
#include <string>

// #include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/lslidar/proto/config.pb.h"
#include "modules/drivers/lidar/lslidar/proto/lslidar.pb.h"

#include "modules/drivers/lidar/lslidar/parser/lslidar_parser.h"

namespace apollo {
namespace drivers {
namespace lslidar {
namespace parser {

using apollo::drivers::PointCloud;
using apollo::drivers::lslidar::LslidarScan;

// convert lslidar data to pointcloud and republish
class Convert {
 public:
    Convert() = default;
    virtual ~Convert() = default;

    // init lslidar config struct from private_nh
    void init(const Config& lslidar_config);

    // convert lslidar data to pointcloud and public
    void ConvertPacketsToPointcloud(
            const std::shared_ptr<apollo::drivers::lslidar::LslidarScan>&
                    scan_msg,
            std::shared_ptr<apollo::drivers::PointCloud> point_cloud_out);

 private:
    // RawData class for converting data to point cloud
    std::unique_ptr<LslidarParser> parser_;
    std::string channel_pointcloud_;

    /// configuration parameters, get config struct from lslidar_parser.h
    Config config_;
};

}  // namespace parser
}  // namespace lslidar
}  // namespace drivers
}  // namespace apollo
