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

#include "modules/drivers/lidar/proto/config.pb.h"
// #include "modules/drivers/lidar/proto/zvision.pb.h"
// #include "modules/drivers/lidar/proto/zvision_config.pb.h"

#include "modules/drivers/lidar/common/driver_factory/driver_base.h"
#include "modules/drivers/lidar/zvision/driver/socket_input.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      constexpr uint32_t PACKET_PER_SCAN_ML30 = 125;
      constexpr uint32_t PACKET_PER_SCAN_ML30S = 160;
      constexpr uint32_t PACKET_PER_SCAN_EZ6 = 600;

      class ZvisionDriver
      {
      public:
        explicit ZvisionDriver(const Config &config) : config_(config) {}
        virtual ~ZvisionDriver();

        virtual bool Poll(const std::shared_ptr<ZvisionScan> &scan);
        virtual void PollCalibrationData();
        virtual void Init();

        void SetScanPackets(const uint32_t packets);

      protected:
        Config config_;
        std::unique_ptr<Input> input_ = nullptr;
        std::string topic_;
        uint32_t packets_per_scan_ = 0;
        bool poll_calibration_ok_ = false;

        std::thread calibration_thread_;

        virtual int PollStandard(std::shared_ptr<ZvisionScan> scan);
      };

      class Ml30Driver : public ZvisionDriver
      {
      public:
        explicit Ml30Driver(const Config &config) : ZvisionDriver(config) { config_.set_packets_per_scan(125); }
        ~Ml30Driver() {}

        void Init() override;
        bool Poll(const std::shared_ptr<ZvisionScan> &scan) override;
      };

      class Ml30sa1Driver : public ZvisionDriver
      {
      public:
        explicit Ml30sa1Driver(const Config &config) : ZvisionDriver(config) { config_.set_packets_per_scan(160); }
        ~Ml30sa1Driver() {}

        void Init() override;
        bool Poll(const std::shared_ptr<ZvisionScan> &scan) override;
      };

      class MlxDriver : public ZvisionDriver
      {
      public:
        explicit MlxDriver(const Config &config) : ZvisionDriver(config) { config_.set_packets_per_scan(400); }
        ~MlxDriver() {}

        void Init() override;
        bool Poll(const std::shared_ptr<ZvisionScan> &scan) override;
      };

      class EZ6Driver : public ZvisionDriver
      {
      public:
        #ifdef ez6_sample_a
        // sample A
        explicit EZ6Driver(const Config &config) : ZvisionDriver(config) { config_.set_packets_per_scan(300);}
        #endif
        #ifdef ez6_sample_b
        // sample B
        explicit EZ6Driver(const Config &config) : ZvisionDriver(config) { config_.set_packets_per_scan(600);}
        #endif
        ~EZ6Driver() {}

        void Init() override;
        bool Poll(const std::shared_ptr<ZvisionScan> &scan) override;
      };

      class ZvisionDriverFactory
      {
      public:
        static ZvisionDriver *CreateDriver(const Config &config);
      };

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
