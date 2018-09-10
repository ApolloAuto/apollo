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
 * @file WriteParams.h
 */
#ifndef _FASTRTPS_RTPS_COMMON_WRITEPARAMS_H_
#define _FASTRTPS_RTPS_COMMON_WRITEPARAMS_H_

#include "SampleIdentity.h"

namespace eprosima
{
    namespace fastrtps
    {
        namespace rtps
        {
            /*!
             * @brief This class contains additional information of a CacheChange.
             * @ingroup COMMON_MODULE
             */
            class RTPS_DllAPI WriteParams
            {
                public:

                    /*!
                     * @brief Default constructor.
                     */
                    WriteParams()
                    {
                    }

                    /*!
                     * @brief Copy constructor.
                     */
                    WriteParams(const WriteParams &wparam) : sample_identity_(wparam.sample_identity_),
                    related_sample_identity_(wparam.related_sample_identity_)
                    {
                    }

                    /*!
                     * @brief Move constructor.
                     */
                    WriteParams(WriteParams &&wparam) : sample_identity_(std::move(wparam.sample_identity_)),
                    related_sample_identity_(std::move(wparam.related_sample_identity_))
                    {
                    }

                    /*!
                     * @brief Assignment operator
                     */
                    WriteParams& operator=(const WriteParams &wparam)
                    {
                        sample_identity_ = wparam.sample_identity_;
                        related_sample_identity_ = wparam.related_sample_identity_;
                        return *this;
                    }

                    /*!
                     * @brief Assignment operator
                     */
                    WriteParams& operator=(WriteParams &&wparam)
                    {
                        sample_identity_ = std::move(wparam.sample_identity_);
                        related_sample_identity_ = std::move(wparam.related_sample_identity_);
                        return *this;
                    }

                    WriteParams& sample_identity(const SampleIdentity &sample_id)
                    {
                        sample_identity_ = sample_id;
                        return *this;
                    }

                    WriteParams& sample_identity(SampleIdentity &&sample_id)
                    {
                        sample_identity_ = std::move(sample_id);
                        return *this;
                    }

                    const SampleIdentity& sample_identity() const
                    {
                        return sample_identity_;
                    }

                    SampleIdentity& sample_identity()
                    {
                        return sample_identity_;
                    }

                    WriteParams& related_sample_identity(const SampleIdentity &sample_id)
                    {
                        related_sample_identity_ = sample_id;
                        return *this;
                    }

                    WriteParams& related_sample_identity(SampleIdentity &&sample_id)
                    {
                        related_sample_identity_ = std::move(sample_id);
                        return *this;
                    }

                    const SampleIdentity& related_sample_identity() const
                    {
                        return related_sample_identity_;
                    }

                    SampleIdentity& related_sample_identity()
                    {
                        return related_sample_identity_;
                    }

                private:

                    SampleIdentity sample_identity_;

                    SampleIdentity related_sample_identity_;
            };

        } //namespace rtps
    } //namespace fastrtps
} //namespace eprosima
#endif //_FASTRTPS_RTPS_COMMON_WRITEPARAMS_H_
