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
 * @file WriterQos.h
 *
 */

#ifndef WRITERQOS_H_
#define WRITERQOS_H_

#include "QosPolicies.h"

namespace eprosima {
namespace fastrtps {


/**
 * Class WriterQos, containing all the possible Qos that can be set for a determined Publisher.
 * Although these values can be set and are transmitted
 * during the Endpoint Discovery Protocol, not all of the behaviour associated with them has been implemented in the library.
 * Please consult each of them to check for implementation details and default values.
 * @ingroup FASTRTPS_ATTRIBUTES_MODULE
 */
class  WriterQos{
public:
	RTPS_DllAPI WriterQos();
	RTPS_DllAPI virtual ~WriterQos();
	//!Durability Qos, implemented in the library.
	DurabilityQosPolicy m_durability;
	//!Durability Service Qos, NOT implemented in the library.
	DurabilityServiceQosPolicy m_durabilityService;
	//!Deadline Qos, NOT implemented in the library.
	DeadlineQosPolicy m_deadline;
	//!Latency Budget Qos, NOT implemented in the library.
	LatencyBudgetQosPolicy m_latencyBudget;
	//!Liveliness Qos, implemented in the library.
	LivelinessQosPolicy m_liveliness;
	//!Reliability Qos, implemented in the library.
	ReliabilityQosPolicy m_reliability;
	//!Lifespan Qos, NOT implemented in the library.
	LifespanQosPolicy m_lifespan;
	//!UserData Qos, NOT implemented in the library.
	UserDataQosPolicy m_userData;
	//!Time Based Filter Qos, NOT implemented in the library.
	TimeBasedFilterQosPolicy m_timeBasedFilter;
	//!Ownership Qos, NOT implemented in the library.
	OwnershipQosPolicy m_ownership;
	//!Owenership Strength Qos, NOT implemented in the library.
	OwnershipStrengthQosPolicy m_ownershipStrength;
	//!Destination Order Qos, NOT implemented in the library.
	DestinationOrderQosPolicy m_destinationOrder;
	//!Presentation Qos, NOT implemented in the library.
	PresentationQosPolicy m_presentation;
	//!Partition Qos, implemented in the library.
	PartitionQosPolicy m_partition;
	//!Topic Data Qos, NOT implemented in the library.
	TopicDataQosPolicy m_topicData;
	//!Group Data Qos, NOT implemented in the library.
	GroupDataQosPolicy m_groupData;
	//!Publication Mode Qos, implemented in the library.
	PublishModeQosPolicy m_publishMode;
	/**
	 * Set Qos from another class
	 * @param qos Reference from a WriterQos object.
	 * @param first_time Boolean indicating whether is the first time (If not some parameters cannot be set).
	 */
	RTPS_DllAPI void setQos(const WriterQos& qos, bool first_time);
	/**
	 * Check if the Qos values are compatible between each other.
	 * @return True if correct.
	 */
	RTPS_DllAPI bool checkQos();

	RTPS_DllAPI bool canQosBeUpdated(WriterQos& qos);
};



} /* namespace  */
} /* namespace eprosima */

#endif /* WRITERQOS_H_ */
