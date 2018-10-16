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
 * @file CDRMessage_t.h	
 */

#ifndef CDRMESSAGE_T_H_
#define CDRMESSAGE_T_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "Types.h"
#include <stdlib.h>
#include <cstring>

namespace eprosima{
namespace fastrtps{
namespace rtps{


//!Max size of RTPS message in bytes.
#define RTPSMESSAGE_DEFAULT_SIZE 10500  //max size of rtps message in bytes
#define RTPSMESSAGE_COMMON_RTPS_PAYLOAD_SIZE 536 //common payload a rtps message has TODO(Ricardo) It is necessary?
#define RTPSMESSAGE_COMMON_DATA_PAYLOAD_SIZE 10000 //common data size
#define RTPSMESSAGE_HEADER_SIZE 20  //header size in bytes
#define RTPSMESSAGE_SUBMESSAGEHEADER_SIZE 4
#define RTPSMESSAGE_DATA_EXTRA_INLINEQOS_SIZE 4
#define RTPSMESSAGE_INFOTS_SIZE 12

#define RTPSMESSAGE_OCTETSTOINLINEQOS_DATASUBMSG 16 //may change in future versions
#define RTPSMESSAGE_OCTETSTOINLINEQOS_DATAFRAGSUBMSG 28 //may change in future versions
#define RTPSMESSAGE_DATA_MIN_LENGTH 24

/**
 * @brief Structure CDRMessage_t, contains a serialized message.
 * @ingroup COMMON_MODULE
 */
struct RTPS_DllAPI CDRMessage_t{
    //! Default constructor
    CDRMessage_t():wraps(false){
        pos = 0;
        length = 0;
        buffer = (octet*) malloc(RTPSMESSAGE_DEFAULT_SIZE);
        max_size = RTPSMESSAGE_DEFAULT_SIZE;

#if EPROSIMA_BIG_ENDIAN
        msg_endian = BIGEND;
#else
        msg_endian = LITTLEEND;
#endif
    }

    ~CDRMessage_t()
    {
        if(buffer != nullptr && !wraps)
            free(buffer);
    }

    /**
     * Constructor with maximum size
     * @param size Maximum size
     */
    CDRMessage_t(uint32_t size)
    {
        wraps = false;
        pos = 0;
        length = 0;

        if(size != 0)
            buffer = (octet*)malloc(size);
        else
            buffer = nullptr;

        max_size = size;

#if EPROSIMA_BIG_ENDIAN
        msg_endian = BIGEND;
#else
        msg_endian = LITTLEEND;
#endif
    }

    CDRMessage_t(const CDRMessage_t& message)
    {
        wraps = false;
        pos = 0;
        length = message.length;
        max_size = message.max_size;
        msg_endian = message.msg_endian;
        
        if(max_size != 0)
        {
            buffer =  (octet*)malloc(max_size);
            memcpy(buffer, message.buffer, length);
        }
        else
            buffer = nullptr;
    }

    CDRMessage_t(CDRMessage_t&& message)
    {
        wraps = message.wraps;
        message.wraps = false;
        pos = message.pos;
        message.pos = 0;
        length = message.length;
        message.length = 0;
        max_size = message.max_size;
        message.max_size = 0;
        msg_endian = message.msg_endian;
#if EPROSIMA_BIG_ENDIAN
        message.msg_endian = BIGEND;
#else
        message.msg_endian = LITTLEEND;
#endif
        buffer = message.buffer;
        message.buffer = nullptr;
    }

    //!Pointer to the buffer where the data is stored.
    octet* buffer;
    //!Read or write position.
    uint32_t pos;
    //!Max size of the message.
    uint32_t max_size;
    //!Current length of the message.
    uint32_t length;
    //!Endianness of the message.
    Endianness_t msg_endian;
    //Whether this message is wrapping a buffer managed elsewhere.
    bool wraps;
};

}
}
}
#endif
#endif /* CDRMESSAGE_T_H_ */
