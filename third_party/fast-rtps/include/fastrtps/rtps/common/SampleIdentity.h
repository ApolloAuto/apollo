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
 * @file SampleIdentity.h
 */
#ifndef _FASTRTPS_RTPS_COMMON_SAMPLEIDENTITY_H_
#define _FASTRTPS_RTPS_COMMON_SAMPLEIDENTITY_H_

#include "Guid.h"
#include "SequenceNumber.h"

namespace eprosima
{
    namespace fastrtps
    {
        namespace rtps
        {
            /*!
             * @brief This class is used to specify a sample
             * @ingroup COMMON_MODULE
             */
            class RTPS_DllAPI SampleIdentity
            {
                public:

                    /*!
                     * @brief Default constructor. Constructs an unknown SampleIdentity.
                     */
                    SampleIdentity() : writer_guid_(GUID_t::unknown()), sequence_number_(SequenceNumber_t::unknown())
                    {
                    }

                    /*!
                     * @brief Copy constructor.
                     */
                    SampleIdentity(const SampleIdentity &sample_id) : writer_guid_(sample_id.writer_guid_),
                    sequence_number_(sample_id.sequence_number_)
                    {
                    }

                    /*!
                     * @brief Move constructor.
                     */
                    SampleIdentity(SampleIdentity &&sample_id) : writer_guid_(std::move(sample_id.writer_guid_)),
                    sequence_number_(std::move(sample_id.sequence_number_))
                    {
                    }

                    /*!
                     * @brief Assignment operator.
                     */
                    SampleIdentity& operator=(const SampleIdentity &sample_id)
                    {
                        writer_guid_ = sample_id.writer_guid_;
                        sequence_number_ = sample_id.sequence_number_;
                        return *this;
                    }

                    /*!
                     * @brief Move constructor.
                     */
                    SampleIdentity& operator=(SampleIdentity &&sample_id)
                    {
                        writer_guid_ = std::move(sample_id.writer_guid_);
                        sequence_number_ = std::move(sample_id.sequence_number_);
                        return *this;
                    }

                    /*!
                     * @brief
                     */
                    bool operator==(const SampleIdentity &sample_id) const
                    {
                        return (writer_guid_ == sample_id.writer_guid_) && (sequence_number_ == sample_id.sequence_number_);
                    }

                    /*!
                     * @brief
                     */
                    bool operator!=(const SampleIdentity &sample_id) const
                    {
                        return !(*this == sample_id);
                    }

                    SampleIdentity& writer_guid(const GUID_t &guid)
                    {
                        writer_guid_ = guid;
                        return *this;
                    }

                    SampleIdentity& writer_guid(GUID_t &&guid)
                    {
                        writer_guid_ = std::move(guid);
                        return *this;
                    }

                    const GUID_t& writer_guid() const
                    {
                        return writer_guid_;
                    }

                    GUID_t& writer_guid()
                    {
                        return writer_guid_;
                    }

                    SampleIdentity& sequence_number(const SequenceNumber_t &seq)
                    {
                        sequence_number_ = seq;
                        return *this;
                    }

                    SampleIdentity& sequence_number(SequenceNumber_t &&seq)
                    {
                        sequence_number_ = std::move(seq);
                        return *this;
                    }

                    const SequenceNumber_t& sequence_number() const
                    {
                        return sequence_number_;
                    }

                    SequenceNumber_t& sequence_number()
                    {
                        return sequence_number_;
                    }

                    static SampleIdentity unknown()
                    {
                        return SampleIdentity();
                    }

                private:

                    GUID_t writer_guid_;

                    SequenceNumber_t sequence_number_;
            };
        } //namespace rtps
    } //namespace fastrtps
} //namespace eprosima
#endif // _FASTRTPS_RTPS_COMMON_SAMPLEIDENTITY_H_
