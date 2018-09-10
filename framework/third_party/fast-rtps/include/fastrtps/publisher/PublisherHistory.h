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
 * @file PublisherHistory.h
 *
 */

#ifndef PUBLISHERHISTORY_H_
#define PUBLISHERHISTORY_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <fastrtps/rtps/resources/ResourceManagement.h>

#include "../rtps/history/WriterHistory.h"
#include "../qos/QosPolicies.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps {

class PublisherImpl;

/**
 * Class PublisherHistory, implementing a WriterHistory with support for keyed topics and HistoryQOS.
 * This class is created by the PublisherImpl and should not be used by the user directly.
 * @ingroup FASTRTPS_MODULE
 */
class PublisherHistory:public WriterHistory
{
public:
	typedef std::pair<InstanceHandle_t,std::vector<CacheChange_t*>> t_p_I_Change;
	typedef std::vector<t_p_I_Change> t_v_Inst_Caches;
	/**
	* Constructor of the PublisherHistory.
	* @param pimpl Pointer to the PublisherImpl.
	* @param payloadInitialSize Initial payload size.
	* @param mempolicy Set wether the payloads ccan dynamically resized or not.
	* @param history QOS of the associated History.
	* @param resource ResourceLimits for the History.
	*/
	PublisherHistory(PublisherImpl* pimpl,uint32_t payloadMax,
			HistoryQosPolicy& history,ResourceLimitsQosPolicy& resource,MemoryManagementPolicy_t mempolicy);
			
	virtual ~PublisherHistory();
	
	/**
	* Add a change comming from the Publisher.
	* @param change Pointer to the change
    * @param wparams Extra write parameters.
    * @param wparams
	* @return True if added.
	*/
	bool add_pub_change(CacheChange_t* change, WriteParams &wparams);

	/**
	* Remove all change from the associated history.
	* @param removed Number of elements removed.
	* @return True if all elements were removed.
	*/
	bool removeAllChange(size_t* removed);

	/**
	* Remove the change with the minimum sequence Number.
	* @return True if removed.
	*/
	bool removeMinChange();

	/**
	* Remove a change by the publisher History.
	* @param change Pointer to the CacheChange_t.
	* @param vit Pointer to the iterator of the Keyed history vector.
	* @return True if removed.
	*/
	bool remove_change_pub(CacheChange_t* change,t_v_Inst_Caches::iterator* vit=nullptr);

    virtual bool remove_change_g(CacheChange_t* a_change);

private:
	//!Vector of pointer to the CacheChange_t divided by key.
	t_v_Inst_Caches m_keyedChanges;
	//!HistoryQosPolicy values.
	HistoryQosPolicy m_historyQos;
	//!ResourceLimitsQosPolicy values.
	ResourceLimitsQosPolicy m_resourceLimitsQos;
	//!Publisher Pointer
	PublisherImpl* mp_pubImpl;

	bool find_Key(CacheChange_t* a_change,t_v_Inst_Caches::iterator* vecPairIterrator);
};

} /* namespace fastrtps */
} /* namespace eprosima */
#endif
#endif /* PUBLISHERHISTORY_H_ */
