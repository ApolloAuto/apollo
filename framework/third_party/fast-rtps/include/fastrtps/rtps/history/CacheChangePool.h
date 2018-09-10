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
 * @file CacheChangePool.h
 *
 */



#ifndef CACHECHANGEPOOL_H_
#define CACHECHANGEPOOL_H_

#include "../resources/ResourceManagement.h"

#include <vector>
#include <functional>
#include <cstdint>
#include <cstddef>
#include <mutex>


namespace eprosima {
namespace fastrtps{
namespace rtps {

struct CacheChange_t;

/**
 * Class CacheChangePool, used by the HistoryCache to pre-reserve a number of CacheChange_t to avoid dynamically reserving memory in the middle of execution loops.
 * @ingroup COMMON_MODULE
 */
class CacheChangePool {
    public:
        virtual ~CacheChangePool();
        /**
         * Constructor.
         * @param pool_size The initial pool size
         * @param payload_size The initial payload size associated with the pool.
         * @param max_pool_size Maximum payload size. If set to 0 the pool will keep reserving until something breaks.
         * @param memoryPolicy Memory management policy.
         */
        CacheChangePool(int32_t pool_size, uint32_t payload_size, int32_t max_pool_size, MemoryManagementPolicy_t memoryPolicy);

        /*!
         * @brief Reserves a CacheChange from the pool.
         * @param chan Returned pointer to the reserved CacheChange.
         * @param calculateSizeFunc Function that returns the size of the data which will go into the CacheChange.
         * This function is executed depending on the memory management policy (DYNAMIC_RESERVE_MEMORY_MODE and
         * PREALLOCATED_WITH_REALLOC_MEMORY_MODE)
         * @return True whether the CacheChange could be allocated. In other case returns false.
         */
        bool reserve_Cache(CacheChange_t** chan, const std::function<uint32_t()>& calculateSizeFunc);

        /*!
         * @brief Reserves a CacheChange from the pool.
         * @param chan Returned pointer to the reserved CacheChange.
         * @param dataSize Size of the data which will go into the CacheChange if it is necessary (on memory management
         * policy DYNAMIC_RESERVE_MEMORY_MODE and PREALLOCATED_WITH_REALLOC_MEMORY_MODE). In other case this variable is not used.
         * @return True whether the CacheChange could be allocated. In other case returns false.
         */
        bool reserve_Cache(CacheChange_t** chan, uint32_t dataSize);

        //!Release a Cache back to the pool.
        void release_Cache(CacheChange_t*);
        //!Get the size of the cache vector; all of them (reserved and not reserved).
        size_t get_allCachesSize(){return m_allCaches.size();}
        //!Get the number of frre caches.
        size_t get_freeCachesSize(){return m_freeCaches.size();}
        //!Get the initial payload size associated with the Pool.
        inline uint32_t getInitialPayloadSize(){return m_initial_payload_size;};
    private:
        uint32_t m_initial_payload_size;
        uint32_t m_payload_size;
        uint32_t m_pool_size;
        uint32_t m_max_pool_size;
        std::vector<CacheChange_t*> m_freeCaches;
        std::vector<CacheChange_t*> m_allCaches;
        bool allocateGroup(uint32_t pool_size);
        CacheChange_t* allocateSingle(uint32_t dataSize);
        std::mutex* mp_mutex;
        MemoryManagementPolicy_t memoryMode;
};
}
} /* namespace rtps */
} /* namespace eprosima */


#endif /* CACHECHANGEPOOL_H_ */
