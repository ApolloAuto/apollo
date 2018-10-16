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
 * @file SubscriberListener.h
 */

#ifndef SUBLISTENER_H_
#define SUBLISTENER_H_

#include "../rtps/common/MatchingInfo.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps {

class Subscriber;

/**
 * Class SubscriberListener, it should be used by the end user to implement specific callbacks to certain actions.
 * @ingroup FASTRTPS_MODULE
 * @snippet fastrtps_example.cpp ex_SubscriberListener
 */
class RTPS_DllAPI SubscriberListener {
public:
	SubscriberListener(){};
	virtual ~SubscriberListener(){};
	/**
	 * Virtual function to be implemented by the user containing the actions to be performed when a new  Data Message is received.
	 * @param sub Subscriber
	 */
	virtual void onNewDataMessage(Subscriber * sub){(void) sub;};
	/**
	 * Virtual method to be called when the subscriber is matched with a new Writer (or unmatched); i.e., when a writer publishing in the same topic is discovered.
	 * @param sub Subscriber
	 * @param info Matching information
	 */
	virtual void onSubscriptionMatched(Subscriber* sub, MatchingInfo& info){(void)sub; (void)info;};
};

} /* namespace pubsub */
} /* namespace eprosima */

#endif /* LISTENER_H_ */
