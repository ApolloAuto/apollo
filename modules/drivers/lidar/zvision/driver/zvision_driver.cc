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

#include "modules/drivers/lidar/zvision/driver/zvision_driver.h"

#include <cmath>
#include <ctime>
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

      ZvisionDriver::~ZvisionDriver()
      {
      }

      void ZvisionDriver::Init()
      {
        packets_per_scan_ = config_.packets_per_scan();
        AINFO << "publishing " << packets_per_scan_ << " packets per scan";

        // open zvision input device
        input_.reset(new SocketInput());
        input_->init(config_.lidar_recv_port());
      }

      void ZvisionDriver::SetScanPackets(const uint32_t packets) { packets_per_scan_ = packets; }

      /** poll the device
       *
       *  @returns true unless end of file reached
       */
      bool ZvisionDriver::Poll(const std::shared_ptr<ZvisionScan> &scan)
      {

        int poll_result = PollStandard(scan);

        if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL)
        {
          return false; // poll again
        }

        if (scan->firing_pkts().empty())
        {
          AINFO << "Get an empty scan from port: " << config_.lidar_recv_port();
          return false;
        }

        // publish message using time of last packet read
        ADEBUG << "Publishing a full Zvision scan.";
        scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
        scan->mutable_header()->set_frame_id(config_.frame_id());
        scan->set_model(config_.model());

        return true;
      }

      int ZvisionDriver::PollStandard(std::shared_ptr<ZvisionScan> scan)
      {
        // Since the zvision delivers data at a very high rate, keep reading and
        // publishing scans as fast as possible.
        for (uint32_t i = 0; i < config_.packets_per_scan(); ++i)
        {
          ZvisionPacket *packet = scan->add_firing_pkts();
          while (true)
          {
            // printf("config_.packets_per_scan() = %d \n", config_.packets_per_scan());
            int rc = input_->get_firing_data_packet(packet);
            // keep reading until full packet received
            uint32_t udp_seq = 0;
            if (rc < 0)
              return rc; // end of file reached?

            if (rc == 0)
            {
              #ifdef ez6_sample_a
              // sample A
              udp_seq = ((unsigned char)packet->data()[11]) + (((unsigned char)packet->data()[10] & 0xF) << 8);
              #endif
              #ifdef ez6_sample_b
              // sample B
              udp_seq = ((unsigned char)packet->data()[13]) + (((unsigned char)packet->data()[12] & 0xF) << 8);
              #endif
              // printf("PollStandard rc == 0  udp_seq = %d \n", udp_seq);
            }
            if (i == udp_seq)
              break;
            else
              ; // ROS_INFO("Find a packet, seq wanted[%d], seq read[%d]", i, udp_seq);
              // printf("Find a packet, seq wanted[%d], seq read[%d] \n", i, udp_seq);
          }
        }
        return 0;
      }

      void ZvisionDriver::PollCalibrationData(void)
      {
        while (!cyber::IsShutdown())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }

      void Ml30Driver::Init()
      {
        packets_per_scan_ = config_.packets_per_scan();
        AINFO << "publishing " << packets_per_scan_ << " packets per scan";

        // open zvision input device
        input_.reset(new SocketInput());
        input_->init(config_.lidar_recv_port());
      }

      bool Ml30Driver::Poll(const std::shared_ptr<ZvisionScan> &scan)
      {

        int poll_result = PollStandard(scan);
        if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL)
        {
          return false; // poll again
        }

        if (scan->firing_pkts().empty())
        {
          AINFO << "Get an empty scan from port: " << config_.lidar_recv_port();
          return false;
        }

        // publish message using time of last packet read
        ADEBUG << "Publishing a full Zvision scan.";
        scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
        scan->mutable_header()->set_frame_id(config_.frame_id());
        scan->set_model(config_.model());

        return true;
      }

      void Ml30sa1Driver::Init()
      {
        packets_per_scan_ = config_.packets_per_scan();
        AINFO << "publishing " << packets_per_scan_ << " packets per scan";

        // open zvision input device
        input_.reset(new SocketInput());
        input_->init(config_.lidar_recv_port());
      }

      bool Ml30sa1Driver::Poll(const std::shared_ptr<ZvisionScan> &scan)
      {

        int poll_result = PollStandard(scan);

        if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL)
        {
          return false; // poll again
        }

        if (scan->firing_pkts().empty())
        {
          AINFO << "Get an empty scan from port: " << config_.lidar_recv_port();
          return false;
        }

        // publish message using time of last packet read
        ADEBUG << "Publishing a full Zvision scan.";
        scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
        scan->mutable_header()->set_frame_id(config_.frame_id());
        scan->set_model(config_.model());

        return true;
      }

      void MlxDriver::Init()
      {
        packets_per_scan_ = config_.packets_per_scan();
        AINFO << "publishing " << packets_per_scan_ << " packets per scan";

        // open zvision input device
        input_.reset(new SocketInput());
        input_->init(config_.lidar_recv_port());
      }

      bool MlxDriver::Poll(const std::shared_ptr<ZvisionScan> &scan)
      {

        int poll_result = PollStandard(scan);

        if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL)
        {
          return false; // poll again
        }

        if (scan->firing_pkts().empty())
        {
          AINFO << "Get an empty scan from port: " << config_.lidar_recv_port();
          return false;
        }

        // publish message using time of last packet read
        ADEBUG << "Publishing a full Zvision scan.";
        scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
        scan->mutable_header()->set_frame_id(config_.frame_id());
        scan->set_model(config_.model());

        return true;
      }

      void EZ6Driver::Init()
      {
        packets_per_scan_ = config_.packets_per_scan();
        AINFO << "publishing " << packets_per_scan_ << " packets per scan";

        // open zvision input device
        input_.reset(new SocketInput());
        input_->init(config_.lidar_recv_port());
      }

      bool EZ6Driver::Poll(const std::shared_ptr<ZvisionScan> &scan)
      {

        int poll_result = PollStandard(scan);

        if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL)
        {
          return false; // poll again
        }

        if (scan->firing_pkts().empty())
        {
          AINFO << "Get an empty scan from port: " << config_.lidar_recv_port();
          return false;
        }

        // publish message using time of last packet read
        ADEBUG << "Publishing a full Zvision scan.";
        scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
        scan->mutable_header()->set_frame_id(config_.frame_id());
        scan->set_model(config_.model());

        // printf("Su cyber::Time().Now().ToSecond() ---------> %u \n", cyber::Time().Now().ToSecond());
        // printf("Su cconfig_.frame_id() ---------> %u \n", config_.frame_id());
        // printf("Su config_.model() ---------> %u \n", config_.model());

        return true;
      }

      ZvisionDriver *ZvisionDriverFactory::CreateDriver(const Config &config)
      {
        ZvisionDriver *driver = nullptr;
        switch (config.model())
        {
        case ML30B1:
        {
          driver = new Ml30Driver(config);
          break;
        }
        case ML30SA1:
        {
          driver = new Ml30sa1Driver(config);
          break;
        }
        case MLX:
        {
          driver = new MlxDriver(config);
          break;
        }
        case EZ6:
        {
          driver = new EZ6Driver(config);
          break;
        }
        default:
          AERROR << "invalid model, must be ML30B1|ML30SA1|EZ6";
          break;
        }
        return driver;
      }

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
