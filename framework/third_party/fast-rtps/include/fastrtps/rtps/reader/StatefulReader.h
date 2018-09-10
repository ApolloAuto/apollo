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
 * @file StatefulReader.h
 */

#ifndef STATEFULREADER_H_
#define STATEFULREADER_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "RTPSReader.h"
#include <mutex>

namespace eprosima {
namespace fastrtps{
namespace rtps {

class WriterProxy;

/**
 * Class StatefulReader, specialization of RTPSReader than stores the state of the matched writers.
 * @ingroup READER_MODULE
 */
class StatefulReader:public RTPSReader
{
    public:

        friend class RTPSParticipantImpl;

        virtual ~StatefulReader();

    private:

        StatefulReader(RTPSParticipantImpl*,GUID_t& guid,
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
         * @param deleteWP If the Reader has to delete the associated WP object or not.
         * @return True if correct.
         */
        bool matched_writer_remove(RemoteWriterAttributes& wdata,bool deleteWP);
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
         * Look for a specific WriterProxy.
         * @param writerGUID GUID_t of the writer we are looking for.
         * @param WP Pointer to pointer to a WriterProxy.
         * @return True if found.
         */
        bool matched_writer_lookup(const GUID_t& writerGUID, WriterProxy** WP);

        /**
         * Processes a new DATA message. Previously the message must have been accepted by function acceptMsgDirectedTo.
         * @param change Pointer to the CacheChange_t.
         * @return true if the reader accepts messages.
         */
        bool processDataMsg(CacheChange_t *change);

        /**
         * Processes a new DATA FRAG message. Previously the message must have been accepted by function acceptMsgDirectedTo.
         * @param change Pointer to the CacheChange_t.
         * @param sampleSize Size of the complete assembled message.
         * @param fragmentStartingNum fragment number of this particular fragment.
         * @return true if the reader accepts messages.
         */
        bool processDataFragMsg(CacheChange_t *change, uint32_t sampleSize, uint32_t fragmentStartingNum);

        /**
         * Processes a new HEARTBEAT message. Previously the message must have been accepted by function acceptMsgDirectedTo.
         *
         * @return true if the reader accepts messages.
         */
        bool processHeartbeatMsg(GUID_t &writerGUID, uint32_t hbCount, SequenceNumber_t &firstSN,
                SequenceNumber_t &lastSN, bool finalFlag, bool livelinessFlag);

        bool processGapMsg(GUID_t &writerGUID, SequenceNumber_t &gapStart, SequenceNumberSet_t &gapList);

        /**
         * Method to indicate the reader that some change has been removed due to HistoryQos requirements.
         * @param change Pointer to the CacheChange_t.
         * @param prox Pointer to the WriterProxy.
         * @return True if correctly removed.
         */
        bool change_removed_by_history(CacheChange_t* change ,WriterProxy* prox = nullptr);

        /**
         * This method is called when a new change is received. This method calls the received_change of the History
         * and depending on the implementation performs different actions.
         * @param a_change Pointer of the change to add.
         * @param prox Pointer to the WriterProxy that adds the Change.
         * @param lock mutex protecting the StatefulReader.
         * @return True if added.
         */
        bool change_received(CacheChange_t* a_change, WriterProxy* prox, std::unique_lock<std::recursive_mutex> &lock);

        /**
         * Get the RTPS participant
         * @return Associated RTPS participant
         */
        inline RTPSParticipantImpl* getRTPSParticipant() const {return mp_RTPSParticipant;}

        /**
         * Read the next unread CacheChange_t from the history
         * @param change Pointer to pointer of CacheChange_t
         * @param wpout Pointer to pointer the matched writer proxy
         * @return True if read.
         */
        bool nextUnreadCache(CacheChange_t** change,WriterProxy** wpout=nullptr);

        /**
         * Take the next CacheChange_t from the history;
         * @param change Pointer to pointer of CacheChange_t
         * @param wpout Pointer to pointer the matched writer proxy
         * @return True if read.
         */
        bool nextUntakenCache(CacheChange_t** change,WriterProxy** wpout=nullptr);


        /**
         * Update the times parameters of the Reader.
         * @param times ReaderTimes reference.
         * @return True if correctly updated.
         */
        bool updateTimes(ReaderTimes& times);
        /**
         *
         * @return Reference to the ReaderTimes.
         */
        inline ReaderTimes& getTimes(){return m_times;};

        /**
         * Get the number of matched writers
         * @return Number of matched writers
         */
        inline size_t getMatchedWritersSize() const { return matched_writers.size(); }

        void sendFragAck(WriterProxy *mp_WP, CacheChange_t *cit);
        /*!
         * @brief Returns there is a clean state with all Writers.
         * It occurs when the Reader received all samples sent by Writers. In other words,
         * its WriterProxies are up to date.
         * @return There is a clean state with all Writers.
         */
        bool isInCleanState() const;

    private:

        bool acceptMsgFrom(GUID_t &entityGUID ,WriterProxy **wp, bool checkTrusted = true);

        /*!
         * @remarks Nn thread-safe.
         */
        bool findWriterProxy(const GUID_t& writerGUID, WriterProxy** WP);

        //!ReaderTimes of the StatefulReader.
        ReaderTimes m_times;
        //! Vector containing pointers to the matched writers.
        std::vector<WriterProxy*> matched_writers;
};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* STATEFULREADER_H_ */
