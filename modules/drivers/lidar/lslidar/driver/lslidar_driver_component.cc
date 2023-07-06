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

#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/lidar/lslidar/driver/lslidar_driver_component.h"

namespace apollo
{
  namespace drivers
  {
    namespace lslidar
    {

      static void my_handler(int sig)
      {
        exit(-1);
      }

      bool LslidarDriverComponent::Init()
      {
        signal(SIGINT, my_handler);
        AINFO << "Lslidar driver component init";
        Config lslidar_config;
        if (!GetProtoConfig(&lslidar_config))
        {
          return false;
        }

        // start the driver
        writer_ = node_->CreateWriter<apollo::drivers::lslidar::LslidarScan>(lslidar_config.scan_channel());
        LslidarDriver *driver = LslidarDriverFactory::CreateDriver(lslidar_config);
        if (driver == nullptr)
        {
          return false;
        }
        dvr_.reset(driver);
        dvr_->Init();
        // spawn device poll thread
        runing_ = true;
        device_thread_ = std::shared_ptr<std::thread>(
            new std::thread(std::bind(&LslidarDriverComponent::device_poll, this)));
        device_thread_->detach();

        return true;
      }

      /** @brief Device poll thread main loop. */
      void LslidarDriverComponent::device_poll()
      {
        signal(SIGINT, my_handler);
        while (!apollo::cyber::IsShutdown())
        {
          // poll device until end of file
          std::shared_ptr<LslidarScan> scan = std::make_shared<apollo::drivers::lslidar::LslidarScan>();

          bool ret = dvr_->Poll(scan);
          if (ret)
          {
            common::util::FillHeader("lslidar", scan.get());
            AINFO << "publish scan!";
            double time1 = apollo::cyber::Time().Now().ToSecond();
            writer_->Write(scan);
            double time2 = apollo::cyber::Time().Now().ToSecond();
            AINFO << "apollo::cyber::Time((time2 - time1)" << apollo::cyber::Time((time2 - time1) / 2.0).ToNanosecond();
          }
          else
          {
            AWARN << "device poll failed";
          }
        }

        AERROR << "CompLslidarDriver thread exit";
        runing_ = false;
      }

    } // namespace lslidar
  }   // namespace drivers
} // namespace apollo
