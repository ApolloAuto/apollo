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
 * @file WriterHistory.h
 *
 */

#ifndef WRITERHISTORY_H_
#define WRITERHISTORY_H_

#include "History.h"

namespace eprosima {
namespace fastrtps{
namespace rtps {

class RTPSWriter;

/**
 * Class WriterHistory, container of the different CacheChanges of a writer
 * @ingroup WRITER_MODULE
 */
class WriterHistory : public History
{
    friend class RTPSWriter;

    public:

    /**
     * Constructor of the WriterHistory.
     */
    RTPS_DllAPI WriterHistory(const HistoryAttributes&  att);
    RTPS_DllAPI virtual ~WriterHistory();

    /**
     * Update the maximum and minimum sequenceNumber cacheChanges.
     */
    RTPS_DllAPI void updateMaxMinSeqNum();
    /**
     * Add a CacheChange_t to the ReaderHistory.
     * @param a_change Pointer to the CacheChange to add.
     * @return True if added.
     */
    RTPS_DllAPI bool add_change(CacheChange_t* a_change);
    /**
     * Remove a specific change from the history.
     * @param a_change Pointer to the CacheChange_t.
     * @return True if removed.
     */
    RTPS_DllAPI bool remove_change(CacheChange_t* a_change);

    virtual bool remove_change_g(CacheChange_t* a_change);

    RTPS_DllAPI bool remove_change(const SequenceNumber_t& sequence_number);

    RTPS_DllAPI CacheChange_t* remove_change_and_reuse(const SequenceNumber_t& sequence_number);

    /**
     * Remove the CacheChange_t with the minimum sequenceNumber.
     * @return True if correctly removed.
     */
    RTPS_DllAPI bool remove_min_change();

    RTPS_DllAPI SequenceNumber_t next_sequence_number() const { return m_lastCacheChangeSeqNum + 1; }

    protected:
    //!Last CacheChange Sequence Number added to the History.
    SequenceNumber_t m_lastCacheChangeSeqNum;
    //!Pointer to the associated RTPSWriter;
    RTPSWriter* mp_writer;
};

}
} /* namespace fastrtps */
} /* namespace eprosima */

#endif /* WRITERHISTORY_H_ */
