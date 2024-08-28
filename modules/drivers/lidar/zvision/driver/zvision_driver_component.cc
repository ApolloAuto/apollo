
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

#include "modules/drivers/lidar/zvision/driver/zvision_driver_component.h"

#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      ZvisionDriverComponent::~ZvisionDriverComponent()
      {
        if (device_thread_->joinable())
        {
          device_thread_->join();
        }
      }
      bool ZvisionDriverComponent::Init()
      {
        {
          AINFO << "Zvision driver component init";
          Config zvision_config;
          if (!GetProtoConfig(&zvision_config))
          {
            return false;
          }
          AINFO << "Zvision config: " << zvision_config.DebugString();
          // start the driver
          writer_ = node_->CreateWriter<ZvisionScan>(zvision_config.scan_channel());
          ZvisionDriver *driver = ZvisionDriverFactory::CreateDriver(zvision_config);
          if (driver == nullptr)
          {
            AERROR << "Create driver error.";
            return false;
          }
          dvr_.reset(driver);
          dvr_->Init();
          // spawn device poll thread
          runing_ = true;
          device_thread_ = std::shared_ptr<std::thread>(
              new std::thread(std::bind(&ZvisionDriverComponent::device_poll, this)));
          device_thread_->detach();

          return true;
        }
      }

      void ZvisionDriverComponent::device_poll()
      {
        while (!apollo::cyber::IsShutdown())
        {
          // poll device until end of file
          std::shared_ptr<ZvisionScan> scan = std::make_shared<ZvisionScan>();
          bool ret = dvr_->Poll(scan);
          if (ret)
          {
            common::util::FillHeader("zvision", scan.get());
            writer_->Write(scan);
            // printf("publish zvision scan, size is %d %s \n", ret, scan.get());
          }
          else
          {
            // printf("poll failed, ret = %d\n", ret);
            AWARN << "device poll failed";
          }
        }

        AERROR << "CompZvisionDriver thread exit";
        runing_ = false;
      }

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
