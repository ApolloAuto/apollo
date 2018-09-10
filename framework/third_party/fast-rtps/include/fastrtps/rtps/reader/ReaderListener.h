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
 * @file ReaderListener.h
 *
 */

#ifndef READERLISTENER_H_
#define READERLISTENER_H_

#include "../common/MatchingInfo.h"
#include <mutex>
namespace eprosima{
namespace fastrtps{
namespace rtps{

class RTPSReader;
struct CacheChange_t;

/**
* Class ReaderListener, to be used by the user to override some of is virtual method to program actions to certain events.
*  @ingroup READER_MODULE
*/
class RTPS_DllAPI ReaderListener
{
public:
	ReaderListener(){};
	virtual ~ReaderListener(){};
	
	/**
	* This method is invoked when a new reader matches
	* @param reader Matching reader
	* @param info Matching information of the reader
	*/
	virtual void onReaderMatched(RTPSReader* reader, MatchingInfo& info){(void)reader; (void)info;};
	
	/**
	* This method is called when a new CacheChange_t is added to the ReaderHistory.
	* @param reader Pointer to the reader.
	* @param change Pointer to the CacheChange_t. This is a const pointer to const data
	* to indicate that the user should not dispose of this data himself.
	* To remove the data call the remove_change method of the ReaderHistory.
	* reader->getHistory()->remove_change((CacheChange_t*)change).
	*/
	virtual void onNewCacheChangeAdded(RTPSReader* reader, const CacheChange_t* const change){(void)reader; (void)change;};
};

//Namespace enders
}
}
}

#endif /* READERLISTENER_H_ */
