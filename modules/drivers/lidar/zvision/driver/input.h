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

#include <unistd.h>
#include <cstdio>
#include <memory>

#include "cyber/cyber.h"

#include "modules/drivers/lidar/proto/zvision.pb.h"

namespace apollo
{
  namespace drivers
  {
    namespace zvision
    {

      static const size_t FIRING_DATA_PACKET_SIZE = 1304;
      static const size_t FIRING_DATA_PACKET_SIZE_EZ6A = 1320;
      static const size_t FIRING_DATA_PACKET_SIZE_EZ6B = 1058;
      static const size_t CALIBRATION_DATA_PACKET_SIZE = 1040;
      static const size_t IMU_DATA_PACKET_SIZE = 37;
      static const size_t ETHERNET_HEADER_SIZE = 42;
      static const int SOCKET_TIMEOUT = -2;
      static const int RECIEVE_FAIL = -3;

      /** @brief Pure virtual Zvision input base class */
      class Input
      {
      public:
        Input() {}
        virtual ~Input() {}

        /** @brief Read one Zvision packet.
         *
         * @param pkt points to ZvisionPacket message
         *
         * @returns 0 if successful,
         *          -1 if end of file
         *          > 0 if incomplete packet (is this possible?)
         */
        virtual int get_firing_data_packet(ZvisionPacket *pkt) = 0;
        virtual void init() {}
        virtual void init(const int &port) {}
      };

    } // namespace zvision
  } // namespace drivers
} // namespace apollo
