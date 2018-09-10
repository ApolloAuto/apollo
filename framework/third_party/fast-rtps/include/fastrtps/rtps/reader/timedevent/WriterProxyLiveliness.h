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
 * @file WriterProxyLiveliness.h
 *
 */

#ifndef WRITERPROXYLIVELINESS_H_
#define WRITERPROXYLIVELINESS_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../../resources/TimedEvent.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {

class WriterProxy;
/**
 * Class WriterProxyLiveliness, timed event to check the liveliness of a writer each leaseDuration.
 *  @ingroup READER_MODULE
 */
class WriterProxyLiveliness: public TimedEvent {
public:
	/**
	* @param wp
	* @param interval
	*/
	WriterProxyLiveliness(WriterProxy* wp,double interval);
	virtual ~WriterProxyLiveliness();
	/**
	* Method invoked when the event occurs
	*
	* @param code Code representing the status of the event
	* @param msg Message associated to the event
	*/
	void event(EventCode code, const char* msg= nullptr);
	//!Pointer to the WriterProxy associated with this specific event.
	WriterProxy* mp_WP;
};
}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* WRITERPROXYLIVELINESS_H_ */
