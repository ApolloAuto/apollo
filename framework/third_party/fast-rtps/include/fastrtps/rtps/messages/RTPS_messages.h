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

/**
 * @file RTPS_messages.h	
 */

#ifndef RTPS_MESSAGES_H_
#define RTPS_MESSAGES_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../common/Types.h"
#include "../common/Guid.h"

#include <iostream>
 #include <bitset>
namespace eprosima{
namespace fastrtps{
namespace rtps{

// //!@brief Enumeration of the different Submessages types

const int PAD = 0x01;
const int ACKNACK = 0x06;
const int HEARTBEAT = 0x07;
const int GAP = 0x08;
const int INFO_TS = 0x09;
const int INFO_SRC = 0x0c;
const int INFO_REPLY_IP4 = 0x0d;
const int INFO_DST = 0x0e;
const int INFO_REPLY = 0x0f;
const int NACK_FRAG = 0x12;
const int HEARTBEAT_FRAG = 0x13;
const int DATA = 0x15;
const int DATA_FRAG = 0x16;
const int SEC_PREFIX = 0x31;
const int SRTPS_PREFIX = 0x33;
//!@brief Structure Header_t, RTPS Message Header Structure.
//!@ingroup COMMON_MODULE
 struct Header_t{
     //!Protocol version
     ProtocolVersion_t version;
     //!Vendor ID
     VendorId_t vendorId;
     //!GUID prefix
     GuidPrefix_t guidPrefix;
     Header_t():
         version(c_ProtocolVersion)
     {
         set_VendorId_eProsima(vendorId);
     }
     ~Header_t(){
     }
 };

 /**
  * @param output
  * @param h
  * @return
  */
 inline std::ostream& operator<<(std::ostream& output,const Header_t& h){
     output << "RTPS HEADER of Version: " << (int)h.version.m_major << "." << (int)h.version.m_minor;
     output << "  || VendorId: " <<std::hex<< (int)h.vendorId[0] << "." <<(int)h.vendorId[1] << std::dec;
     output << "GuidPrefix: " << h.guidPrefix;
     return output;
 }

 //!@brief Structure SubmessageHeader_t, used to contain the header information of a submessage.
 struct SubmessageHeader_t{
     octet submessageId;
     uint16_t submessageLength;
     uint32_t submsgLengthLarger;
     SubmessageFlag flags;

     SubmessageHeader_t():
         submessageId(0),
         submessageLength(0),
         submsgLengthLarger(0),
         flags(0)
     {}
 };

 using std::cout;
 using std::endl;
 using std::bitset;

 /**
  * @param output
  * @param sh
  * @return
  */
 inline std::ostream& operator<<(std::ostream& output,const SubmessageHeader_t& sh){
     output << "Submessage Header, ID: " <<std::hex<< (int)sh.submessageId << std::dec;
     output << " length: " << (int)sh.submessageLength << " flags " << (bitset<8>) sh.flags;
     return output;
 }
}
}
}


#endif
#endif /* RTPS_MESSAGES_H_ */
