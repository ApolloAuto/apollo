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
 * @file StatelessReader.h
 */


#ifndef STATELESSREADER_H_
#define STATELESSREADER_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "RTPSReader.h"

#include <mutex>
#include <map>

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
 * Class StatelessReader, specialization of the RTPSReader for Best Effort Readers.
 * @ingroup READER_MODULE
 */
class StatelessReader: public RTPSReader
{
    friend class RTPSParticipantImpl;

public:
    virtual ~StatelessReader();
private:
    StatelessReader(RTPSParticipantImpl*,GUID_t& guid,
            ReaderAttributes& att,ReaderHistory* hist,ReaderListener* listen=nullptr);
public:
    /**
     * Add a matched writer represented by a WriterProxyData object.
     * @param wdata Pointer to the WPD object to add.
     * @return True if correctly added.
     */
    bool matched_writer_add(RemoteWriterAttributes& wdata);
    /**
     * Remove a WriterProxyData from the matached writers.
     * @param wdata Pointer to the WPD object.
     * @return True if correct.
     */
    bool matched_writer_remove(RemoteWriterAttributes& wdata);

    /**
     * Tells us if a specific Writer is matched against this reader
     * @param wdata Pointer to the WriterProxyData object
     * @return True if it is matched.
     */
    bool matched_writer_is_matched(RemoteWriterAttributes& wdata);

    /**
     * Method to indicate the reader that some change has been removed due to HistoryQos requirements.
     * @param change Pointer to the CacheChange_t.
     * @param prox Pointer to the WriterProxy.
     * @return True if correctly removed.
     */
    bool change_removed_by_history(CacheChange_t* change,WriterProxy* prox = nullptr);

    /**
     * Processes a new DATA message. Previously the message must have been accepted by function acceptMsgDirectedTo.
     *
     * @param change Pointer to the CacheChange_t.
     * @return true if the reader accepts messages from the.
     */
    bool processDataMsg(CacheChange_t *change);

    /**
     * Processes a new DATA FRAG message. Previously the message must have been accepted by function acceptMsgDirectedTo.
     * @param change Pointer to the CacheChange_t.
     * @param sampleSize Size of the complete assembled message.
     * @param fragmentStartingNum fragment number of this particular fragment.
     * @return true if the reader accepts message.
     */
    bool processDataFragMsg(CacheChange_t *change, uint32_t sampleSize, uint32_t fragmentStartingNum);

    /**
     * Processes a new HEARTBEAT message. Previously the message must have been accepted by function acceptMsgDirectedTo.
     *
     * @return true if the reader accepts messages from the.
     */
    bool processHeartbeatMsg(GUID_t &writerGUID, uint32_t hbCount, SequenceNumber_t &firstSN,
            SequenceNumber_t &lastSN, bool finalFlag, bool livelinessFlag);

    bool processGapMsg(GUID_t &writerGUID, SequenceNumber_t &gapStart, SequenceNumberSet_t &gapList);

    /**
     * This method is called when a new change is received. This method calls the received_change of the History
     * and depending on the implementation performs different actions.
     * @param a_change Pointer of the change to add.
     * @return True if added.
     */
    bool change_received(CacheChange_t* a_change, std::unique_lock<std::recursive_mutex> &lock);

    /**
     * Read the next unread CacheChange_t from the history
     * @param change Pointer to pointer of CacheChange_t
     * @param wpout Pointer to pointer of the matched writer proxy
     * @return True if read.
     */
    bool nextUnreadCache(CacheChange_t** change,WriterProxy** wpout=nullptr);
    /**
     * Take the next CacheChange_t from the history;
     * @param change Pointer to pointer of CacheChange_t
     * @param wpout Pointer to pointer of the matched writer proxy
     * @return True if read.
     */
    bool nextUntakenCache(CacheChange_t** change,WriterProxy** wpout=nullptr);

    /**
     * Get the number of matched writers
     * @return Number of matched writers
     */
    inline size_t getMatchedWritersSize() const {return m_matched_writers.size();};

    /*!
     * @brief Returns there is a clean state with all Writers.
     * StatelessReader allways return true;
     * @return true
     */
    bool isInCleanState() const { return true; }

    inline RTPSParticipantImpl* getRTPSParticipant() const {return mp_RTPSParticipant;}

private:

    bool acceptMsgFrom(GUID_t& entityId);

    bool thereIsUpperRecordOf(GUID_t& guid, SequenceNumber_t& seq);

    //!List of GUID_t os matched writers.
    //!Is only used in the Discovery, to correctly notify the user using SubscriptionListener::onSubscriptionMatched();
    std::vector<RemoteWriterAttributes> m_matched_writers;

	//!Information about changes already in History
	std::map<GUID_t, SequenceNumber_t> m_historyRecord;
};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* STATELESSREADER_H_ */
