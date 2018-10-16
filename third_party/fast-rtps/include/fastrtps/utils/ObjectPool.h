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
 * @file ObjectPool.h
 *
 */

#ifndef OBJECTPOOL_H_
#define OBJECTPOOL_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include <vector>
#include <cstdint>


namespace eprosima {
namespace fastrtps{
namespace rtps {


/**
 * ObjectPool class used to define an object pool of different types.
 * @ingroup UTILITIESMODULE
 */
template <typename T>
class ObjectPool {
public:
	/**
	* @param defaultGroupSize Default group size
	*/
	ObjectPool(uint16_t defaultGroupSize);
	virtual ~ObjectPool();
	T& reserve_Object();
	void release_Object(T& obj);
protected:
	std::vector<T*> m_free_objects;
	std::vector<T*> m_all_objects;
	uint16_t m_group_size;
	void allocateGroup();

};
}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* OBJECTPOOL_H_ */
