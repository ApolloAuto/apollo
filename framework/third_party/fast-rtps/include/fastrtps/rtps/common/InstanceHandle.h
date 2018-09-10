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
 * @file InstanceHandle.h 	
 */

#ifndef INSTANCEHANDLE_H_
#define INSTANCEHANDLE_H_
#include "../../fastrtps_dll.h"
#include "Types.h"
#include "Guid.h"

namespace eprosima{
namespace fastrtps{
namespace rtps{

/**
 * Struct InstanceHandle_t, used to contain the key for WITH_KEY topics.
 * @ingroup COMMON_MODULE
 */
struct RTPS_DllAPI InstanceHandle_t{
	//!Value
	octet value[16];
	InstanceHandle_t()
	{
		for(uint8_t i=0;i<16;i++)
			value[i] = 0;
	}

	InstanceHandle_t(const InstanceHandle_t& ihandle)
	{
		for(uint8_t i = 0; i < 16; i++)
			value[i] = ihandle.value[i];
	}
	
	/**
	* Assingment operator
	* @param ihandle Instance handle to copy the data from
	*/
	InstanceHandle_t& operator=(const InstanceHandle_t& ihandle){

		for(uint8_t i =0;i<16;i++)
		{
			value[i] = ihandle.value[i];
		}
		return *this;
	}
	
	/**
	* Assingment operator
	* @param guid GUID to copy the data from
	*/
	InstanceHandle_t& operator=(const GUID_t& guid)
	{
		for(uint8_t i =0;i<16;i++)
		{
			if(i<12)
				value[i] = guid.guidPrefix.value[i];
			else
				value[i] = guid.entityId.value[i-12];
		}
		return *this;
	}

	/**
	* Know if the instance handle is defined
	* @return True if the values are not zero.
	*/
	bool isDefined()
	{
		for(uint8_t i=0;i<16;++i)
		{
			if(value[i]!=0)
				return true;
		}
		return false;
	}
};

const InstanceHandle_t c_InstanceHandle_Unknown;

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
* Comparison operator
* @param ihandle1 First InstanceHandle_t to compare
* @param ihandle2 Second InstanceHandle_t to compare
* @return True if equal
*/
inline bool operator==(const InstanceHandle_t & ihandle1, const InstanceHandle_t& ihandle2)
{
	for(uint8_t i =0;i<16;++i)
	{
		if(ihandle1.value[i] != ihandle2.value[i])
			return false;
	}
	return true;
}
#endif

/**
* Convert InstanceHandle_t to GUID
* @param guid GUID to store the results
* @param ihandle InstanceHandle_t to copy
*/
inline void iHandle2GUID(GUID_t& guid,const InstanceHandle_t& ihandle)
{
	for(uint8_t i = 0;i<16;++i)
	{
		if(i<12)
			guid.guidPrefix.value[i] = ihandle.value[i];
		else
			guid.entityId.value[i-12] = ihandle.value[i];
	}
	return;
}


/**
* Convert GUID to InstanceHandle_t
* @param ihandle InstanceHandle_t to store the results
* @return GUID_t
*/
inline GUID_t iHandle2GUID(const InstanceHandle_t& ihandle)
{
	GUID_t guid;
	for(uint8_t i = 0;i<16;++i)
	{
		if(i<12)
			guid.guidPrefix.value[i] = ihandle.value[i];
		else
			guid.entityId.value[i-12] = ihandle.value[i];
	}
	return guid;
}

inline bool operator<(const InstanceHandle_t& h1, const InstanceHandle_t& h2)
{
    return memcmp(h1.value, h2.value, 16) < 0;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
* 
* @param output 
* @param iHandle
*/
inline std::ostream& operator<<(std::ostream& output,const InstanceHandle_t& iHandle)
{
	output << std::hex;
	for(uint8_t i =0;i<15;++i)
		output << (int)iHandle.value[i] << ".";
	output << (int)iHandle.value[15] << std::dec;
	return output;
}

#endif

}
}
}

#endif /* INSTANCEHANDLE_H_ */
