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

#ifndef THROUGHPUT_CONTROLLER_DESCRIPTOR_H
#define THROUGHPUT_CONTROLLER_DESCRIPTOR_H

#include <fastrtps/fastrtps_dll.h>
#include <cstdint>

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * Descriptor for a Throughput Controller, containing all constructor information
 * for it.
 * @ingroup NETWORK_MODULE
 */
struct ThroughputControllerDescriptor
{
    //! Packet size in bytes that this controller will allow in a given period.
    uint32_t bytesPerPeriod;
    //! Window of time in which no more than 'bytesPerPeriod' bytes are allowed.
    uint32_t periodMillisecs;

    RTPS_DllAPI ThroughputControllerDescriptor();
    RTPS_DllAPI ThroughputControllerDescriptor(uint32_t size, uint32_t time);
};

} // namespace rtps
} // namespace fastrtps
} // namespace eprosima

#endif // THROUGHPUT_CONTROLLER_DESCRIPTOR_H
