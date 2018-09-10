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
 * @file NackSupressionDuration.h
 *
 */

#ifndef NACKSUPRESSIONDURATION_H_
#define NACKSUPRESSIONDURATION_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "../../resources/TimedEvent.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {

class StatefulWriter;
class ReaderProxy;

/**
 * NackSupressionDuration class, used to avoid too "recent" NACK messages.
 * @ingroup WRITER_MODULE
 */
class NackSupressionDuration : public TimedEvent
{
public:
	virtual ~NackSupressionDuration();
	/**
	*
	* @param p_RP
	* @param intervalmillisec
	*/
	NackSupressionDuration(ReaderProxy* p_RP,double intervalmillisec);

	/**
	* Method invoked when the event occurs
	*
	* @param code Code representing the status of the event
	* @param msg Message associated to the event
	*/
	void event(EventCode code, const char* msg= nullptr);

	//!Reader proxy
	ReaderProxy* mp_RP;
};

}
}
} /* namespace eprosima */
#endif
#endif /* NACKSUPRESSIONDURATION_H_ */
