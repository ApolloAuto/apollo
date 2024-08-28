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

#include "modules/drivers/lidar/zvision/parser/convert.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      using apollo::drivers::PointCloud;
      using apollo::drivers::zvision::ZvisionScan;

      void Convert::init(const Config &zvision_config)
      {
        config_ = zvision_config;
        // we use Beijing time by default

        parser_.reset(ZvisionParserFactory::CreateParser(config_));
        if (parser_.get() == nullptr)
        {
          AFATAL << "Create parser failed.";
          return;
        }
        parser_->setup();
      }

      /** @brief Callback for raw scan messages. */
      void Convert::ConvertPacketsToPointcloud(
          const std::shared_ptr<ZvisionScan> &scan_msg,
          std::shared_ptr<PointCloud> point_cloud)
      {
        ADEBUG << "Convert scan msg seq " << scan_msg->header().sequence_num();

        parser_->GeneratePointcloud(scan_msg, point_cloud);

        if (point_cloud == nullptr || point_cloud->point().empty())
        {
          // if(!parser_->CalibrationInitOk())
          AERROR << "point cloud has no point";
          return;
        }
        point_cloud->set_is_dense(true);
      }

      bool Convert::CalibrationInitOk()
      {
        return parser_->CalibrationInitOk();
      }

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
