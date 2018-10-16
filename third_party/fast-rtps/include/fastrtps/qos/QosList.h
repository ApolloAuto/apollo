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
 * @file QosList.h
 */

#ifndef QOSLIST_H_
#define QOSLIST_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../rtps/common/all_common.h"
#include "ParameterList.h"


namespace eprosima {
namespace fastrtps {

/**
 * QosList_t class contains two QoS parameter lists: One for all Qos policies and another for those that can be sent inline.
 * @ingroup PARAMETER_MODULE
 */
class QosList_t {
public:
	QosList_t();
	virtual ~QosList_t();
	//! All the Qos as a parameter list.
	ParameterList_t allQos;
	//! Only the Qos that can be sent as inline.
	ParameterList_t inlineQos;
};

/**
 * QosList class, that contains static methods to add Qos to a QosList_t structure.
 * @ingroup PARAMETER_MODULE
 */
class QosList
{
public:
	/**
	 * @name AddQos methods.
	 * @param qos Pointer to the QOsList_t list.
	 * @param pid PID of the parameter to add to the QosList_t.
	 * @param param Parameter to add.
	 * @return True if correct.
	 */
	///@{
	static bool addQos(QosList_t* qos,ParameterId_t pid ,std::string& string_in);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,Locator_t& loc);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,uint32_t uintlong);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,const GUID_t& guid);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,ProtocolVersion_t& protocol);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,VendorId_t& vendor);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,octet o1,octet o2,octet o3,octet o4);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,const EntityId_t& entity);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,Time_t& entity);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,bool in_bool);
	static bool addQos(QosList_t* qos,ParameterId_t pid ,std::string& str1,std::string& str2);
	static bool addQos(QosList_t* qos, ParameterId_t pid,	std::vector<octet>& ocVec);
	static bool addQos(QosList_t* qos,ParameterId_t pid, const ParameterPropertyList_t& list);
	static bool addQos(QosList_t* qos,ParameterId_t pid, const IdentityToken& identity_token);
	///@}
};

} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* QOSLIST_H_ */
