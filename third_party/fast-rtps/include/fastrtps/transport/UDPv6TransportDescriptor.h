// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UDPV6_TRANSPORT_DESCRIPTOR 
#define UDPV6_TRANSPORT_DESCRIPTOR

#include <fastrtps/transport/TransportInterface.h>

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * Transport configuration
 *
 * - bufferSize:    length of the buffers used for transmission. Passing
 *                  a buffer of different size will cause transmission to
 *                  fail.
 *
 * - interfaceWhiteList: Lists the allowed interfaces.
 * @ingroup TRANSPORT_MODULE
 */
typedef struct UDPv6TransportDescriptor: public TransportDescriptorInterface {
   //! Length of the send buffer.
   uint32_t sendBufferSize;
   //! Length of the receive buffer.
   uint32_t receiveBufferSize;
   //! Allowed interfaces in an IP string format.
   std::vector<std::string> interfaceWhiteList;
   //! Specified time to live (8bit - 255 max TTL)
   uint8_t TTL;

   virtual ~UDPv6TransportDescriptor(){}

   RTPS_DllAPI UDPv6TransportDescriptor();

   RTPS_DllAPI UDPv6TransportDescriptor(const UDPv6TransportDescriptor& t);
} UDPv6TransportDescriptor;

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
