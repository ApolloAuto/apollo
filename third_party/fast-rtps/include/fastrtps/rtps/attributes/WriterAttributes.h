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
 * @file WriterAttributes.h
 *
 */
#ifndef WRITERATTRIBUTES_H_
#define WRITERATTRIBUTES_H_

#include "../common/Time_t.h"
#include "../common/Guid.h"
#include "../flowcontrol/ThroughputControllerDescriptor.h"
#include "EndpointAttributes.h"

namespace eprosima{
namespace fastrtps{
namespace rtps{


typedef enum RTPSWriterPublishMode : octet
{
    SYNCHRONOUS_WRITER,
    ASYNCHRONOUS_WRITER
} RTPSWriterPublishMode;


/**
 * Class WriterTimes, defining the times associated with the Reliable Writers events.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class  WriterTimes
{
    public:

        WriterTimes()
        {
            initialHeartbeatDelay.fraction = 200*1000*1000;
            heartbeatPeriod.seconds = 3;
            nackResponseDelay.fraction = 200*1000*1000;
        };
        virtual ~WriterTimes(){};

        //! Initial heartbeat delay. Default value ~45ms.
        Duration_t initialHeartbeatDelay;
        //! Periodic HB period, default value 3s.
        Duration_t heartbeatPeriod;
        //!Delay to apply to the response of a ACKNACK message, default value ~45ms.
        Duration_t nackResponseDelay;
        //!This time allows the RTPSWriter to ignore nack messages too soon after the data as sent, default value 0s.
        Duration_t nackSupressionDuration;
};

/**
 * Class WriterAttributes, defining the attributes of a RTPSWriter.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class  WriterAttributes
{
    public:
        WriterAttributes() : mode(SYNCHRONOUS_WRITER),
            disableHeartbeatPiggyback(false)
        {
            endpoint.endpointKind = WRITER;
            endpoint.durabilityKind = TRANSIENT_LOCAL;
            endpoint.reliabilityKind = RELIABLE;
        };

        virtual ~WriterAttributes(){};
        //!Attributes of the associated endpoint.
        EndpointAttributes endpoint;
        //!Writer Times (only used for RELIABLE).
        WriterTimes times;
        //!Indicates if the Writer is synchronous or asynchronous
        RTPSWriterPublishMode mode;
        // Throughput controller, always the last one to apply 
        ThroughputControllerDescriptor throughputController;
        //! Disable the sending of heartbeat piggybacks.
        bool disableHeartbeatPiggyback;
};

/**
 * Class RemoteReaderAttributes, to define the attributes of a Remote Reader.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class  RemoteReaderAttributes
{
    public:
        RemoteReaderAttributes()
        {
            endpoint.endpointKind = READER;
            expectsInlineQos = false;
        };
        virtual ~RemoteReaderAttributes()
        {

        };
        //!Attributes of the associated endpoint.
        EndpointAttributes endpoint;
        //!GUID_t of the reader.
        GUID_t guid;
        //!Expects inline QOS.
        bool expectsInlineQos;
};
}
}
}


#endif /* WRITERATTRIBUTES_H_ */
