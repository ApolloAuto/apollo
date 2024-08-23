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

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/zvision/parser/parser.h"

#include "modules/drivers/lidar/proto/zvision.pb.h"
#include "modules/drivers/lidar/proto/zvision_config.pb.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      using apollo::drivers::PointCloud;
      using apollo::drivers::zvision::ZvisionScan;

      // convert zvision data to pointcloud and republish
      class Convert
      {
      public:
        Convert() = default;
        virtual ~Convert() = default;

        // init zvision config struct from private_nh
        // void init(ros::NodeHandle& node, ros::NodeHandle& private_nh);
        void init(const Config &zvision_config);

        // convert zvision data to pointcloud and public
        void ConvertPacketsToPointcloud(const std::shared_ptr<ZvisionScan> &scan_msg,
                                        std::shared_ptr<PointCloud> point_cloud_out);

        // parser calibration is ok
        bool CalibrationInitOk();

      private:
        // RawData class for converting data to point cloud
        std::unique_ptr<ZvisionParser> parser_;

        // std::string topic_packets_;
        std::string channel_pointcloud_;

        /// configuration parameters, get config struct from zvision_parser.h
        Config config_;
        // queue size for ros node pub
        // int queue_size_ = 10;
      };

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
