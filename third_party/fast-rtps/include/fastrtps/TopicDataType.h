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
 * @file TopicDataType.h
 */

#ifndef TOPICDATATYPE_H_
#define TOPICDATATYPE_H_

#include "rtps/common/Types.h"
#include "rtps/common/SerializedPayload.h"
#include "rtps/common/InstanceHandle.h"
#include "utils/md5.h"
#include <string>
#include <functional>

using namespace eprosima::fastrtps::rtps;



namespace eprosima {
namespace fastrtps {


/**
 * Class TopicDataType used to provide the DomainRTPSParticipant with the methods to serialize, deserialize and get the key of a specific data type.
 * The user should created a class that inherits from this one, where Serialize and deserialize methods MUST be implemented.
 * @ingroup FASTRTPS_MODULE
 * @snippet fastrtps_example.cpp ex_TopicDataType
 */
class  TopicDataType {
    public:
        RTPS_DllAPI TopicDataType()
            : m_typeSize(0), m_isGetKeyDefined(false)
        {}

        RTPS_DllAPI virtual ~TopicDataType(){};
        /**
         * Serialize method, it should be implemented by the user, since it is abstract.
         * It is VERY IMPORTANT that the user sets the serializedPaylaod length correctly.
         * @param[in] data Pointer to the data
         * @param[out] payload Pointer to the payload
         * @return True if correct.
         */
        RTPS_DllAPI virtual bool serialize(void* data, SerializedPayload_t* payload) = 0;

        /**
         * Deserialize method, it should be implemented by the user, since it is abstract.
         * @param[in] payload Pointer to the payload
         * @param[out] data Pointer to the data
         * @return True if correct.
         */
        RTPS_DllAPI virtual bool deserialize(SerializedPayload_t* payload, void* data) = 0;

        RTPS_DllAPI virtual std::function<uint32_t()> getSerializedSizeProvider(void* data) = 0;

        /**
         * Create a Data Type.
         * @return Void pointer to the created object.
         */
        RTPS_DllAPI virtual void * createData() = 0;
        /**
         * Remove a previously created object.
         * @param data Pointer to the created Data.
         */
        RTPS_DllAPI virtual void deleteData(void * data) = 0;

        /**
         * Get the key associated with the data.
         * @param[in] data Pointer to the data.
         * @param[out] ihandle Pointer to the Handle.
         * @return True if correct.
         */
        RTPS_DllAPI virtual bool getKey(void* data, InstanceHandle_t* ihandle){ (void) data; (void) ihandle; return false; }

        /**
         * Set topic data type name
         * @param nam Topic data type name
         */
        RTPS_DllAPI inline void setName(const char* nam) { m_topicDataTypeName = std::string(nam); }

        /**
         * Get topic data type name
         * @return Topic data type name
         */
        RTPS_DllAPI inline const char* getName() const { return m_topicDataTypeName.c_str(); }

        //! Maximum serialized size of the type in bytes.
        //! If the type has unbounded fields, and therefore cannot have a maximum size, use 0.
        uint32_t m_typeSize;

        //! Indicates whether the method to obtain the key has been implemented.
        bool m_isGetKeyDefined;
    private:
        //! Data Type Name.
        std::string m_topicDataTypeName;



};

} /* namespace pubsub */
} /* namespace eprosima */

#endif /* RTPSTOPICDATATYPE_H_ */
