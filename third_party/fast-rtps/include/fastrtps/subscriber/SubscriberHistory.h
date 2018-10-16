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
 * @file SubscriberHistory.h
 *
 */

#ifndef SUBSCRIBERHISTORY_H_
#define SUBSCRIBERHISTORY_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <fastrtps/rtps/resources/ResourceManagement.h>
#include "../rtps/history/ReaderHistory.h"
#include "../qos/QosPolicies.h"
#include "SampleInfo.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps {

namespace rtps{
class WriterProxy;
}
using namespace rtps;

class SubscriberImpl;

/**
 * Class SubscriberHistory, container of the different CacheChanges of a subscriber
 *  @ingroup FASTRTPS_MODULE
 */
class SubscriberHistory: public ReaderHistory {
public:
	typedef std::pair<InstanceHandle_t,std::vector<CacheChange_t*>> t_p_I_Change;
	typedef std::vector<t_p_I_Change> t_v_Inst_Caches;
	
	/**
	* Constructor. Requires information about the subscriner
	* @param pimpl Pointer to the subscriber implementation
	* @param payloadMax Maximum payload size per change
	* @param history History QoS policy for the reader
	* @param resource Resource Limit QoS policy for the reader
	*/
	SubscriberHistory(SubscriberImpl* pimpl,uint32_t payloadMax,
			HistoryQosPolicy& history,ResourceLimitsQosPolicy& resource, MemoryManagementPolicy_t mempolicy);
	virtual ~SubscriberHistory();

	/**
	* Called when a change is received by the Subscriber History. Will add the change to the history
   * if it wasn't already present
	* @param[in] change The received change
	* @param unknown_missing_changes_up_to Number of missing changes before this one
	* @return
	*/
	bool received_change(CacheChange_t* change, size_t unknown_missing_changes_up_to);

	/** @name Read or take data methods.
	 * Methods to read or take data from the History.
	 * @param data Pointer to the object where you want to read or take the information.
	 * @param info Pointer to a SampleInfo_t object where you want
	 * to store the information about the retrieved data
	 */
	///@{
	bool readNextData(void* data, SampleInfo_t* info);
	bool takeNextData(void* data, SampleInfo_t* info);
	///@}
	
	/**
	 * Method to know whether there are unread CacheChange_t.
	 * @return True if there are any unread changes.
	 */
	bool isUnreadCache();

	/**
	* This method is called to remove a change from the SubscriberHistory.
	* @param change Pointer to the CacheChange_t.
	* @param vit Pointer to the iterator of the key-ordered cacheChange vector.
	* @return True if removed.
	*/
	bool remove_change_sub(CacheChange_t* change,t_v_Inst_Caches::iterator* vit=nullptr);

	//!Increase the unread count.
	inline void increaseUnreadCount()
	{
		++m_unreadCacheCount;
	}
	
	//!Decrease the unread count.
	inline void decreaseUnreadCount()
	{
		if(m_unreadCacheCount>0)
			--m_unreadCacheCount;
	}
	
	/** Get the unread count.
	* @return Unread count
	*/
	inline uint64_t getUnreadCount()
	{
		return m_unreadCacheCount;
	}
private:
	//!Number of unread CacheChange_t.
	uint64_t m_unreadCacheCount;
	//!Vector of pointer to the CacheChange_t divided by key.
	t_v_Inst_Caches m_keyedChanges;
	//!HistoryQosPolicy values.
	HistoryQosPolicy m_historyQos;
	//!ResourceLimitsQosPolicy values.
	ResourceLimitsQosPolicy m_resourceLimitsQos;
	//!Publisher Pointer
	SubscriberImpl* mp_subImpl;

	//!Type object to deserialize Key
	void * mp_getKeyObject;


	bool find_Key(CacheChange_t* a_change,t_v_Inst_Caches::iterator* vecPairIterrator);

};

} /* namespace fastrtps */
} /* namespace eprosima */
#endif
#endif /* SUBSCRIBERHISTORY_H_ */
