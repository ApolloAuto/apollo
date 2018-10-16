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

#ifndef TEST_UDPV4_TRANSPORT_H
#define TEST_UDPV4_TRANSPORT_H
#include <fastrtps/transport/UDPv4Transport.h>
#include <fastrtps/rtps/messages/RTPS_messages.h>
#include <fastrtps/rtps/common/SequenceNumber.h>
#include <fastrtps/rtps/messages/CDRMessage.h>
#include <vector>

#include "test_UDPv4TransportDescriptor.h"

namespace eprosima{
namespace fastrtps{
namespace rtps{

/*
 * This transport acts as a shim over UDPv4, allowing
 * packets to be dropped under certain criteria.
 */ 
class test_UDPv4Transport : public UDPv4Transport
{
public:
   RTPS_DllAPI test_UDPv4Transport(const test_UDPv4TransportDescriptor& descriptor);

   virtual bool Send(const octet* sendBuffer, uint32_t sendBufferSize, const Locator_t& localLocator, const Locator_t& remoteLocator);
  
   // Handle to a persistent log of dropped packets. Defaults to length 0 (no logging) to prevent wasted resources.
   RTPS_DllAPI static std::vector<std::vector<octet> > DropLog;
   RTPS_DllAPI static uint32_t DropLogLength;

private:
   uint8_t mDropDataMessagesPercentage;
   bool mDropParticipantBuiltinTopicData;
   bool mDropPublicationBuiltinTopicData;
   bool mDropSubscriptionBuiltinTopicData;
   uint8_t mDropDataFragMessagesPercentage;
   uint8_t mDropHeartbeatMessagesPercentage;
   uint8_t mDropAckNackMessagesPercentage;
   std::vector<SequenceNumber_t> mSequenceNumberDataMessagesToDrop;
   uint8_t mPercentageOfMessagesToDrop;

   bool LogDrop(const octet* buffer, uint32_t size);
   bool PacketShouldDrop(const octet* sendBuffer, uint32_t sendBufferSize);
   bool RandomChanceDrop();
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif
