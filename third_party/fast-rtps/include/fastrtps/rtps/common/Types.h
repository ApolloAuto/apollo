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
 * @file Types.h
 */

#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <stddef.h>
#include <iostream>
#include <cstdint>
#include <stdint.h>

#include "../../fastrtps_dll.h"


namespace eprosima{
namespace fastrtps{
namespace rtps{

/*!
 * @brief This enumeration represents endianness types.
 * @ingroup COMMON_MODULE
 */
enum Endianness_t{
    //! @brief Big endianness.
    BIGEND = 0x1,
    //! @brief Little endianness.
    LITTLEEND = 0x0
};

//!Reliability enum used for internal purposes
//!@ingroup COMMON_MODULE
typedef enum ReliabilityKind_t{
    RELIABLE,
    BEST_EFFORT
}ReliabilityKind_t;

//!Durability kind
//!@ingroup COMMON_MODULE
typedef enum DurabilityKind_t
{
    VOLATILE,
    TRANSIENT_LOCAL
}DurabilityKind_t;

//!Endpoint kind
//!@ingroup COMMON_MODULE
typedef enum EndpointKind_t{
    READER,
    WRITER
}EndpointKind_t;

//!Topic kind
typedef enum TopicKind_t{
    NO_KEY,
    WITH_KEY
}TopicKind_t;


#if __BIG_ENDIAN__
const Endianness_t DEFAULT_ENDIAN = BIGEND;
#else
const Endianness_t DEFAULT_ENDIAN = LITTLEEND;
#endif

#define EPROSIMA_BIG_ENDIAN 0


typedef unsigned char octet;
//typedef unsigned int uint;
//typedef unsigned short ushort;
typedef unsigned char SubmessageFlag;
typedef uint32_t BuiltinEndpointSet_t;
typedef uint32_t Count_t;

#define BIT0 0x1
#define BIT1 0x2
#define BIT2 0x4
#define BIT3 0x8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define BIT(i) ((i==0) ? BIT0 : (i==1) ? BIT1 :(i==2)?BIT2:(i==3)?BIT3:(i==4)?BIT4:(i==5)?BIT5:(i==6)?BIT6:(i==7)?BIT7:0x0)

//!@brief Structure ProtocolVersion_t, contains the protocol version.
struct RTPS_DllAPI ProtocolVersion_t{
    octet m_major;
    octet m_minor;
    ProtocolVersion_t():
        m_major(2),
        m_minor(1)
    {

    };
    ProtocolVersion_t(octet maj,octet min):
        m_major(maj),
        m_minor(min)
    {

    }
};


const ProtocolVersion_t c_ProtocolVersion_2_0(2,0);
const ProtocolVersion_t c_ProtocolVersion_2_1(2,1);
const ProtocolVersion_t c_ProtocolVersion_2_2(2,2);

const ProtocolVersion_t c_ProtocolVersion(2,1);

//!@brief Structure VendorId_t, specifying the vendor Id of the implementation.
typedef octet VendorId_t[2];

const VendorId_t c_VendorId_Unknown={0x00,0x00};
const VendorId_t c_VendorId_eProsima={0x01,0x0F};


static inline void set_VendorId_Unknown(VendorId_t& id)
{
    id[0]=0x0;id[1]=0x0;
}

static inline void set_VendorId_eProsima(VendorId_t& id)
{
    id[0]=0x01;id[1]=0x0F;
}

}
}
}

#endif /* COMMON_TYPES_H_ */
