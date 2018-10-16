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
 * @file History.h
 *
 */

#ifndef HISTORY_H_
#define HISTORY_H_

#include <mutex>

#include "../../fastrtps_dll.h"

#include "CacheChangePool.h"

#include "../common/SequenceNumber.h"
#include "../common/Guid.h"
#include "../attributes/HistoryAttributes.h"

#include <cassert>

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
 * Class History, container of the different CacheChanges and the methods to access them.
 * @ingroup COMMON_MODULE
 */
class History
{
    protected:
        History(const HistoryAttributes&  att);
        virtual ~History();
    public:
        //!Attributes of the History
        HistoryAttributes m_att;
        /**
         * Reserve a CacheChange_t from the CacheChange pool.
         * @param[out] change Pointer to pointer to the CacheChange_t to reserve
         * @return True is reserved
         */
        RTPS_DllAPI inline bool reserve_Cache(CacheChange_t** change, const std::function<uint32_t()>& calculateSizeFunc)
        {
            return m_changePool.reserve_Cache(change, calculateSizeFunc);
        }

        RTPS_DllAPI inline bool reserve_Cache(CacheChange_t** change, uint32_t dataSize)
        {
            return m_changePool.reserve_Cache(change, dataSize);
        }

        /**
         * release a previously reserved CacheChange_t.
         * @param ch Pointer to the CacheChange_t.
         */
        RTPS_DllAPI inline void release_Cache(CacheChange_t* ch) { return m_changePool.release_Cache(ch); }

        /**
         * Check if the history is full
         * @return true if the History is full.
         */
        RTPS_DllAPI bool isFull()	{ return m_isHistoryFull; }
        /**
         * Get the History size.
         * @return Size of the history.
         */
        RTPS_DllAPI size_t getHistorySize(){ return m_changes.size(); }
        /**
         * Remove all changes from the History
         * @return True if everything was correctly removed.
         */
        RTPS_DllAPI bool remove_all_changes();
        /**
         * Update the maximum and minimum sequenceNumbers.
         */
        virtual void updateMaxMinSeqNum()=0;
        /**
         * Remove a specific change from the history.
         * @param ch Pointer to the CacheChange_t.
         * @return True if removed.
         */
        virtual bool remove_change(CacheChange_t* ch) = 0;

        /**
         * Get the beginning of the changes history iterator.
         * @return Iterator to the beginning of the vector.
         */
        RTPS_DllAPI std::vector<CacheChange_t*>::iterator changesBegin(){ return m_changes.begin(); }
        /**
         * Get the end of the changes history iterator.
         * @return Iterator to the end of the vector.
         */
        RTPS_DllAPI std::vector<CacheChange_t*>::iterator changesEnd(){ return m_changes.end(); }
        /**
         * Get the minimum CacheChange_t.
         * @param min_change Pointer to pointer to the minimum change.
         * @return True if correct.
         */
        RTPS_DllAPI bool get_min_change(CacheChange_t** min_change);

        /**
         * Get the maximum CacheChange_t.
         * @param max_change Pointer to pointer to the maximum change.
         * @return True if correct.
         */
        RTPS_DllAPI bool get_max_change(CacheChange_t** max_change);

        /**
         * Get the maximum serialized payload size
         * @return Maximum serialized payload size
         */
        RTPS_DllAPI inline uint32_t getTypeMaxSerialized(){ return m_changePool.getInitialPayloadSize(); }

        /*!
         * Get the mutex
         * @return Mutex
         */
        RTPS_DllAPI inline std::recursive_mutex* getMutex() { assert(mp_mutex != nullptr); return mp_mutex; }

        RTPS_DllAPI bool get_change(SequenceNumber_t& seq, GUID_t& guid,CacheChange_t** change);

    protected:
        //!Vector of pointers to the CacheChange_t.
        std::vector<CacheChange_t*> m_changes;
        //!Variable to know if the history is full without needing to block the History mutex.
        bool m_isHistoryFull;
        //!Pointer to and invalid cacheChange used to return the maximum and minimum when no changes are stored in the history.
        CacheChange_t* mp_invalidCache;
        //!Pool of cache changes reserved when the History is created.
        CacheChangePool m_changePool;
        //!Pointer to the minimum sequeceNumber CacheChange.
        CacheChange_t* mp_minSeqCacheChange;
        //!Pointer to the maximum sequeceNumber CacheChange.
        CacheChange_t* mp_maxSeqCacheChange;
        //!Print the seqNum of the changes in the History (for debugging purposes).
        void print_changes_seqNum2();
        //!Mutex for the History.
        std::recursive_mutex* mp_mutex;
};

}
} /* namespace rtps */
} /* namespace eprosima */

#endif /* HISTORY_H_ */
