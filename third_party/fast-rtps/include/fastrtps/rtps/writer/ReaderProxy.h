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
 * @file ReaderProxy.h
 *
 */
#ifndef READERPROXY_H_
#define READERPROXY_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include <algorithm>
#include <mutex>
#include <set>
#include "../common/Types.h"
#include "../common/Locator.h"
#include "../common/SequenceNumber.h"
#include "../common/CacheChange.h"
#include "../common/FragmentNumber.h"
#include "../attributes/WriterAttributes.h"

#include <set>

namespace eprosima
{
    namespace fastrtps
    {
        namespace rtps
        {

            class StatefulWriter;
            class NackResponseDelay;
            class NackSupressionDuration;
            class InitialHeartbeat;


            /**
             * ReaderProxy class that helps to keep the state of a specific Reader with respect to the RTPSWriter.
             * @ingroup WRITER_MODULE
             */
            class ReaderProxy
            {
                public:
                    ~ReaderProxy();

                    /**
                     * Constructor.
                     * @param rdata RemoteWriterAttributes to use in the creation.
                     * @param times WriterTimes to use in the ReaderProxy.
                     * @param SW Pointer to the StatefulWriter.
                     */
                ReaderProxy(RemoteReaderAttributes& rdata,const WriterTimes& times,StatefulWriter* SW);

                void destroy_timers();

                void addChange(const ChangeForReader_t&);

                size_t countChangesForReader() const;

                bool change_is_acked(const SequenceNumber_t& sequence_number);

                /**
                 * Mark all changes up to the one indicated by the seqNum as Acknowledged.
                 * If seqNum == 30, changes 1-29 are marked as ack.
                 * @param seqNum Pointer to the seqNum
                 * @return True if all changes are acknowledge and anyone with other state.
                 */
                bool acked_changes_set(const SequenceNumber_t& seqNum);

                /**
                 * Mark all changes in the vector as requested.
                 * @param seqNumSet Vector of sequenceNumbers
                 * @return False if any change was set REQUESTED.
                 */
                bool requested_changes_set(std::vector<SequenceNumber_t>& seqNumSet);

                /*!
                 * @brief Lists all unsent changes. These changes are also relevants and valid.
                 * @return STL vector with the unsent change list.
                 */
                //TODO(Ricardo) Temporal
                //std::vector<const ChangeForReader_t*> get_unsent_changes() const;
                std::vector<ChangeForReader_t*> get_unsent_changes();
                /*!
                 * @brief Lists all requested changes.
                 * @return STL vector with the requested change list.
                 */
                std::vector<const ChangeForReader_t*> get_requested_changes() const;

                /*!
                 * @brief Sets a change to a particular status (if present in the ReaderProxy)
                 * @param change change to search and set.
                 * @param status Status to apply.
                 */
                void set_change_to_status(const SequenceNumber_t& seq_num, ChangeForReaderStatus_t status);

                bool mark_fragment_as_sent_for_change(const CacheChange_t* change, FragmentNumber_t fragment);

                /*
                 * Converts all changes with a given status to a different status.
                 * @param previous Status to change.
                 * @param next Status to adopt.
                 */
                void convert_status_on_all_changes(ChangeForReaderStatus_t previous, ChangeForReaderStatus_t next);

                //void setNotValid(const CacheChange_t* change);
                void setNotValid(CacheChange_t* change);

                /*!
                 * @brief Returns there is some UNACKNOWLEDGED change.
                 * @return There is some UNACKNOWLEDGED change.
                 */
                bool thereIsUnacknowledged() const;

                /**
                 * Get a vector of all unacked changes by this Reader.
                 * @param reqChanges Pointer to a vector of pointers.
                 * @return True if correct.
                 */
                bool unacked_changes(std::vector<const ChangeForReader_t*>* reqChanges);

                //!Attributes of the Remote Reader
                RemoteReaderAttributes m_att;

                //!Pointer to the associated StatefulWriter.
                StatefulWriter* mp_SFW;

                /**
                 * Return the minimum change in a vector of CacheChange_t.
                 * @param Changes Pointer to a vector of CacheChange_t.
                 * @param changeForReader Pointer to the CacheChange_t.
                 * @return True if correct.
                 */
                bool minChange(std::vector<ChangeForReader_t*>* Changes, ChangeForReader_t* changeForReader);

                /*!
                 * @brief Adds requested fragments. These fragments will be sent in next NackResponseDelay.
                 * @param[in] frag_set set containing the requested fragments to be sent.
                 * @param[in] sequence_number Sequence number to be paired with the requested fragments.
                 * @return True if there is at least one requested fragment. False in other case.
                 */
                bool requested_fragment_set(SequenceNumber_t sequence_number, const FragmentNumberSet_t& frag_set);

                /*!
                 * @brief Returns the last NACKFRAG count.
                 * @return Last NACKFRAG count.
                 */
                uint32_t getLastNackfragCount() const { return lastNackfragCount_; }

                /*!
                 * @brief Sets the last NACKFRAG count.
                 * @param lastNackfragCount New value for last NACKFRAG count.
                 */
                void setLastNackfragCount(uint32_t lastNackfragCount) { lastNackfragCount_ = lastNackfragCount; }

                //! Timed Event to manage the Acknack response delay.
                NackResponseDelay* mp_nackResponse;
                //! Timed Event to manage the delay to mark a change as UNACKED after sending it.
                NackSupressionDuration* mp_nackSupression;
                //! Timed Event to send initial heartbeat.
                InitialHeartbeat* mp_initialHeartbeat;
                //! Last ack/nack count
                uint32_t m_lastAcknackCount;

                /**
                 * Filter a CacheChange_t, in this version always returns true.
                 * @param change
                 * @return
                 */
                inline bool rtps_is_relevant(CacheChange_t* change){(void)change; return true;};

                //!Mutex
                std::recursive_mutex* mp_mutex;

                std::set<ChangeForReader_t, ChangeForReaderCmp> m_changesForReader;

                //!Set of the changes and its state.
                private:
                //! Last  NACKFRAG count.
                uint32_t lastNackfragCount_;

                SequenceNumber_t changesFromRLowMark_;
            };
        }
    } /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* READERPROXY_H_ */
