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
 * @file CDRMessagePool.h
 *
 */

#ifndef CDRMESSAGEPOOL_H_
#define CDRMESSAGEPOOL_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../common/CDRMessage_t.h"
#include <vector>
#include <mutex>

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
*@ingroup COMMON_MODULE
*/
class CDRMessagePool {
public:
	/**
	* @param defaultGroupSize Number of messages per allocated group.
	*/
	CDRMessagePool(uint32_t defaultGroupSize);
	virtual ~CDRMessagePool();
	
	//!
	CDRMessage_t& reserve_CDRMsg();
	
	/**
	* @param payload Payload size for the reserved message.
	*/
	CDRMessage_t& reserve_CDRMsg(uint16_t payload);
	
	/**
	* @param obj
	*/
	void release_CDRMsg(CDRMessage_t& obj);
protected:
	std::vector<CDRMessage_t*> m_free_objects;
	std::vector<CDRMessage_t*> m_all_objects;
	uint16_t m_group_size;
	void allocateGroup();
	void allocateGroup(uint16_t payload);
    std::mutex *mutex_;
};





}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* CDRMESSAGEPOOL_H_ */
