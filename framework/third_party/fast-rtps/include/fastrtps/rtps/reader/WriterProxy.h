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
 * @file WriterProxy.h
 */

#ifndef WRITERPROXY_H_
#define WRITERPROXY_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <mutex>

#include "../common/Types.h"
#include "../common/Locator.h"
#include "../common/CacheChange.h"
#include "../attributes/ReaderAttributes.h"

#include<set>

// Testing purpose
#ifndef TEST_FRIENDS
#define TEST_FRIENDS
#endif // TEST_FRIENDS

namespace eprosima
{
    namespace fastrtps
    {
        namespace rtps
        {

            class StatefulReader;
            class HeartbeatResponseDelay;
            class WriterProxyLiveliness;
            class InitialAckNack;

            /**
             * Class WriterProxy that contains the state of each matched writer for a specific reader.
             * @ingroup READER_MODULE
             */
            class WriterProxy
            {
                TEST_FRIENDS

                public:

                    ~WriterProxy();

                    /**
                     * Constructor.
                     * @param watt RemoteWriterAttributes.
                     * @param SR Pointer to the StatefulReader.
                     */
                    WriterProxy(RemoteWriterAttributes& watt, StatefulReader* SR);

                    /**
                     * Get the maximum sequenceNumber received from this Writer.
                     * @return the maximum sequence number.
                     */
                    const SequenceNumber_t available_changes_max() const;

                    /**
                     * Update the missing changes up to the provided sequenceNumber.
                     * All changes with status UNKNOWN with seqNum <= input seqNum are marked MISSING.
                     * @param[in] seqNum Pointer to the SequenceNumber.
                     */
                    void missing_changes_update(const SequenceNumber_t& seqNum);

                    /**
                     * Update the lost changes up to the provided sequenceNumber.
                     * All changes with status UNKNOWN or MISSING with seqNum < input seqNum are marked LOST.
                     * @param[in] seqNum Pointer to the SequenceNumber.
                     */
                    void lost_changes_update(const SequenceNumber_t& seqNum);

                    /**
                     * The provided change is marked as RECEIVED.
                     * @param seqNum Sequence number of the change
                     * @return True if correct.
                     */
                    bool received_change_set(const SequenceNumber_t& seqNum);

                    /**
                     * Set a change as RECEIVED and NOT RELEVANT.
                     * @param seqNum Sequence number of the change
                     * @return true on success
                     */
                    bool irrelevant_change_set(const SequenceNumber_t& seqNum);

                    void setNotValid(const SequenceNumber_t& seqNum);

                    bool areThereMissing();

                    /**
                     * The method returns a vector containing all missing changes.
                     * @return Vector of missing changes..
                     */
                    const std::vector<ChangeFromWriter_t>  missing_changes();

                    size_t unknown_missing_changes_up_to(const SequenceNumber_t& seqNum);

                    //! Pointer to associated StatefulReader.
                    StatefulReader* mp_SFR;
                    //! Parameters of the WriterProxy
                    RemoteWriterAttributes m_att;
                    //! Acknack Count
                    uint32_t m_acknackCount;
                    //! NACKFRAG Count
                    uint32_t m_nackfragCount;
                    //! LAst HEartbeatcount.
                    uint32_t m_lastHeartbeatCount;
                    //!Timed event to postpone the heartbeatResponse.
                    HeartbeatResponseDelay* mp_heartbeatResponse;
                    //!TO check the liveliness Status periodically.
                    WriterProxyLiveliness* mp_writerProxyLiveliness;
                    //! Timed event to send initial acknack.
                    InitialAckNack* mp_initialAcknack;
                    //!Indicates if the heartbeat has the final flag set.
                    bool m_heartbeatFinalFlag;

                    /**
                     * Check if the writer is alive
                     * @return true if the writer is alive
                     */
                    inline bool isAlive(){return m_isAlive;};

                    /**
                     * Set the writer as alive
                     */
                    void assertLiveliness();
                    /**
                     * Set the writer as not alive
                     * @return
                     */
                    inline void setNotAlive(){m_isAlive = false;};

                    /**
                     * Get the mutex
                     * @return Associated mutex
                     */
                    inline std::recursive_mutex* getMutex(){return mp_mutex;};

                    /*!
                     * @brief Returns number of ChangeFromWriter_t managed currently by the WriterProxy.
                     * @return Number of ChangeFromWriter_t managed currently by the WriterProxy.
                    */
                    size_t numberOfChangeFromWriter() const;

                    /*!
                     * @brief Returns next CacheChange_t to be notified.
                     * @return Next CacheChange_t to be nofified or invalid SequenceNumber_t
                     * if any CacheChange_t to be notified.
                     */
                    SequenceNumber_t nextCacheChangeToBeNotified();

                    bool change_was_received(const SequenceNumber_t& seq_num);

                private:

                    /*!
                     * @brief Add ChangeFromWriter_t up to the sequenceNumber passed, but not including this.
                     * Ex: If you have seqNums 1,2,3 and you receive seqNum 6, you need to add 4 and 5.
                     * @param sequence_number
                     * @param default_status ChangeFromWriter_t added will be created with this ChangeFromWriterStatus_t.
                     * @return True if sequence_number will be the next after last element in the m_changesFromW container.
                     * @remarks No thread-safe.
                     */
                    bool maybe_add_changes_from_writer_up_to(const SequenceNumber_t& sequence_number, const ChangeFromWriterStatus_t default_status = ChangeFromWriterStatus_t::UNKNOWN);

                    bool received_change_set(const SequenceNumber_t& seqNum, bool is_relevance);

                    void cleanup();

                    //!Is the writer alive
                    bool m_isAlive;
                    //Print Method for log purposes
                    void print_changes_fromWriter_test2();

                    //!Mutex Pointer
                    std::recursive_mutex* mp_mutex;

                    //!Vector containing the ChangeFromWriter_t objects.
                    std::set<ChangeFromWriter_t, ChangeFromWriterCmp> m_changesFromW;
                    SequenceNumber_t changesFromWLowMark_;

                    //! Store last ChacheChange_t notified.
                    SequenceNumber_t lastNotified_;

                    void for_each_set_status_from(decltype(m_changesFromW)::iterator first,
                            decltype(m_changesFromW)::iterator last,
                            ChangeFromWriterStatus_t status,
                            ChangeFromWriterStatus_t new_status);

                    void for_each_set_status_from_and_maybe_remove(decltype(m_changesFromW)::iterator first,
                            decltype(m_changesFromW)::iterator last,
                            ChangeFromWriterStatus_t status,
                            ChangeFromWriterStatus_t orstatus,
                            ChangeFromWriterStatus_t new_status);
            };

        } /* namespace rtps */
    } /* namespace fastrtps */
} /* namespace eprosima */
#endif
#endif /* WRITERPROXY_H_ */
