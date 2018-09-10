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
 * @file TopicAttributes.h
 */

#ifndef TOPICPARAMETERS_H_
#define TOPICPARAMETERS_H_

#include <string>

#include "../rtps/common/Types.h"

#include "../qos/QosPolicies.h"
#include "../log/Log.h"

using namespace eprosima::fastrtps::rtps;

namespace eprosima {
namespace fastrtps{

/**
 * Class TopicAttributes, used by the user to define the attributes of the topic associated with a Publisher or Subscriber.
 * @ingroup FASTRTPS_ATTRIBUTES_MODULE
 */
class TopicAttributes
{
    public:
        /**
         * Default constructor
         */
        TopicAttributes()
        {
            topicKind = NO_KEY;
            topicName = "UNDEF";
            topicDataType = "UNDEF";
        }
        //!Constructor, you need to provide the topic name and the topic data type.
        TopicAttributes(const char* name, const char* dataType, TopicKind_t tKind= NO_KEY)
        {
            topicKind = tKind;
            topicName = std::string(name);
            topicDataType = std::string(dataType);
        }
        virtual ~TopicAttributes() {
        }

        /**
         * Get the topic data type
         * @return Topic data type
         */
        const std::string& getTopicDataType() const {
            return topicDataType;
        }

        /**
         * Get the topic kind
         * @return Topic kind
         */
        TopicKind_t getTopicKind() const {
            return topicKind;
        }

        /**
         * Get the topic name
         * @return Topic name
         */
        const std::string& getTopicName() const {
            return topicName;
        }

        //! TopicKind_t, default value NO_KEY.
        TopicKind_t topicKind;
        //! Topic Name.
        std::string topicName;
        //!Topic Data Type.
        std::string topicDataType;
        //!QOS Regarding the History to be saved.
        HistoryQosPolicy historyQos;
        //!QOS Regarding the resources to allocate.
        ResourceLimitsQosPolicy resourceLimitsQos;
        /**
         * Method to check whether the defined QOS are correct.
         * @return True if they are valid.
         */
        bool checkQos()
        {
            if(resourceLimitsQos.max_samples_per_instance > resourceLimitsQos.max_samples && topicKind == WITH_KEY)
            {

                logError(RTPS_QOS_CHECK,"INCORRECT TOPIC QOS ("<< topicName <<"):max_samples_per_instance must be <= than max_samples");
                return false;
            }
            if(resourceLimitsQos.max_samples_per_instance*resourceLimitsQos.max_instances > resourceLimitsQos.max_samples && topicKind == WITH_KEY)
                logWarning(RTPS_QOS_CHECK,"TOPIC QOS: max_samples < max_samples_per_instance*max_instances");
            if(historyQos.kind == KEEP_LAST_HISTORY_QOS)
            {
                if(historyQos.depth > resourceLimitsQos.max_samples)
                {
                    logError(RTPS_QOS_CHECK,"INCORRECT TOPIC QOS ("<< topicName <<"): depth must be <= max_samples");
                    return false;
                }
                if(historyQos.depth > resourceLimitsQos.max_samples_per_instance && topicKind == WITH_KEY)
                {
                    logError(RTPS_QOS_CHECK,"INCORRECT TOPIC QOS ("<< topicName <<"): depth must be <= max_samples_per_instance");
                    return false;
                }
                if(historyQos.depth <=0 )
                {
                    logError(RTPS_QOS_CHECK,"INCORRECT TOPIC QOS ("<< topicName <<"): depth must be > 0");
                    return false;
                }
            }
            if(resourceLimitsQos.max_samples != 0 && resourceLimitsQos.allocated_samples > resourceLimitsQos.max_samples)
            {
                logError(RTPS_QOS_CHECK,"INCORRECT TOPIC QOS ("<< topicName <<"): max_samples < allocated_samples");
                return false;
            }
            return true;
        }
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * Check if two topic attributes are not equal
 * @param t1 First instance of TopicAttributes to compare 
 * @param t2 Second instance of TopicAttributes to compare 
 * @return True if the instances are not equal. False if the instances are equal.
 */
bool inline operator!=(TopicAttributes& t1, TopicAttributes& t2)
{
    if(t1.topicKind != t2.topicKind)
    {
        return true;
    }
    if(t1.topicName != t2.topicName)
    {
        return true;
    }
    if(t1.topicDataType != t2.topicDataType)
    {
        return true;
    }
    if(t1.historyQos.kind != t2.historyQos.kind)
    {
        return true;
    }
    if(t1.historyQos.kind == KEEP_LAST_HISTORY_QOS && t1.historyQos.depth != t2.historyQos.depth)
    {
        return true;
    }
    return false;
}
#endif

} /* namespace fastrtps */
} /* namespace eprosima */

#endif /* TOPICPARAMETERS_H_ */
