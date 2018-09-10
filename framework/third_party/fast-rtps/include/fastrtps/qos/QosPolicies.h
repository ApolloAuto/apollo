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
 * @file QosPolicies.h
 *
 */

#ifndef QOS_POLICIES_H_
#define QOS_POLICIES_H_

#include <vector>
#include "../rtps/common/Types.h"
#include "../rtps/common/Time_t.h"
#include "ParameterTypes.h"
using namespace eprosima::fastrtps::rtps;


namespace eprosima{
namespace fastrtps{

namespace rtps{
class EDP;
}

/**
 * Class QosPolicy, base for all QoS policies defined for Writers and Readers.
 */
class QosPolicy{
    public:
        QosPolicy():hasChanged(false),m_sendAlways(false){};
        QosPolicy(bool b_sendAlways):hasChanged(false),m_sendAlways(b_sendAlways){};
        virtual ~ QosPolicy(){};
        bool hasChanged;
        /**
         * Whether it should always be sent.
         * @return True if it should always be sent.
         */
        virtual bool sendAlways(){return m_sendAlways;}
    protected:
        bool m_sendAlways;

};
/**
 * Enum DurabilityQosPolicyKind_t, different kinds of durability for DurabilityQosPolicy.
 */
typedef enum DurabilityQosPolicyKind: octet{
    VOLATILE_DURABILITY_QOS  ,      //!< Volatile Durability (default for Subscribers).
    TRANSIENT_LOCAL_DURABILITY_QOS ,//!< Transient Local Durability (default for Publishers).
    TRANSIENT_DURABILITY_QOS ,      //!< NOT IMPLEMENTED.
    PERSISTENT_DURABILITY_QOS       //!< NOT IMPLEMENTED.
}DurabilityQosPolicyKind_t;

#define PARAMETER_KIND_LENGTH 4

/**
 * Class DurabilityQosPolicy, to indicate the durability of the samples.
 * kind: Default value for Subscribers: VOLATILE_DURABILITY_QOS, for Publishers TRANSIENT_LOCAL_DURABILITY_QOS
 */
class DurabilityQosPolicy : private Parameter_t, public QosPolicy
{
    public:
        RTPS_DllAPI DurabilityQosPolicy():Parameter_t(PID_DURABILITY,PARAMETER_KIND_LENGTH),QosPolicy(true),kind(VOLATILE_DURABILITY_QOS){};
        virtual RTPS_DllAPI ~DurabilityQosPolicy(){};
        DurabilityQosPolicyKind_t kind;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Class DeadlineQosPolicy, to indicate the Deadline of the samples.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * period: Default value c_TimeInifinite.
 */
class DeadlineQosPolicy : private Parameter_t, public QosPolicy {
    public:
        RTPS_DllAPI DeadlineQosPolicy():Parameter_t(PID_DEADLINE,PARAMETER_TIME_LENGTH),QosPolicy(true),period(c_TimeInfinite){	};
        virtual RTPS_DllAPI ~DeadlineQosPolicy(){};
        Duration_t period;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Class LatencyBudgetQosPolicy, to indicate the LatencyBudget of the samples.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * period: Default value c_TimeZero.
 */
class LatencyBudgetQosPolicy : private Parameter_t, public QosPolicy {
    public:
        RTPS_DllAPI LatencyBudgetQosPolicy():Parameter_t(PID_LATENCY_BUDGET,PARAMETER_TIME_LENGTH),QosPolicy(true),duration(c_TimeZero){};
        virtual RTPS_DllAPI ~LatencyBudgetQosPolicy(){};
        Duration_t duration;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Enum LivelinessQosPolicyKind, different kinds of liveliness for LivelinessQosPolicy
 */
typedef enum LivelinessQosPolicyKind:octet {
    AUTOMATIC_LIVELINESS_QOS ,             //!< Automatic Liveliness, default value.
    MANUAL_BY_PARTICIPANT_LIVELINESS_QOS,//!< MANUAL_BY_PARTICIPANT_LIVELINESS_QOS
    MANUAL_BY_TOPIC_LIVELINESS_QOS       //!< MANUAL_BY_TOPIC_LIVELINESS_QOS
}LivelinessQosPolicyKind;

/**
 * Class LivelinessQosPolicy, to indicate the Liveliness of the Writers.
 * This QosPolicy can be defined for the Subscribers and is transmitted but only the Writer Liveliness protocol
 * is implemented in this version. The user should set the lease_duration and the announcement_period with values that differ
 * in at least 30%. Values too close to each other may cause the failure of the writer liveliness assertion in networks
 * with high latency or with lots of communication errors.
 * kind: Default value AUTOMATIC_LIVELINESS_QOS
 * lease_duration: Default value c_TimeInfinite.
 * announcement_period: Default value c_TimeInfinite (must be < lease_duration).
 */
class LivelinessQosPolicy : private Parameter_t, public QosPolicy {
    public:
        RTPS_DllAPI LivelinessQosPolicy():Parameter_t(PID_LIVELINESS,PARAMETER_KIND_LENGTH+PARAMETER_TIME_LENGTH),QosPolicy(true),
        kind(AUTOMATIC_LIVELINESS_QOS){lease_duration = c_TimeInfinite; announcement_period = c_TimeInfinite;};
        virtual RTPS_DllAPI ~LivelinessQosPolicy(){};
        LivelinessQosPolicyKind kind;
        Duration_t lease_duration;
        Duration_t announcement_period;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Enum ReliabilityQosPolicyKind, different kinds of reliability for ReliabilityQosPolicy.
 */
typedef enum ReliabilityQosPolicyKind:octet {
    BEST_EFFORT_RELIABILITY_QOS = 0x01, //!< Best Effort reliability (default for Subscribers).
    RELIABLE_RELIABILITY_QOS = 0x02 //!< Reliable reliability (default for Publishers).
}ReliabilityQosPolicyKind;

/**
 * Class ReliabilityQosPolicy, to indicate the reliability of the endpoints.
 * kind: Default value BEST_EFFORT_RELIABILITY_QOS for ReaderQos and RELIABLE_RELIABILITY_QOS for WriterQos.
 * max_blocking_time: Not Used in this version.
 */
class ReliabilityQosPolicy : private Parameter_t, public QosPolicy
{
    public:
        RTPS_DllAPI ReliabilityQosPolicy():	Parameter_t(PID_RELIABILITY,PARAMETER_KIND_LENGTH+PARAMETER_TIME_LENGTH),
        QosPolicy(true), //indicate send always
        kind(BEST_EFFORT_RELIABILITY_QOS){};
        virtual RTPS_DllAPI ~ReliabilityQosPolicy(){};
        ReliabilityQosPolicyKind kind;
        Duration_t max_blocking_time;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};



/**
 * Enum OwnershipQosPolicyKind, different kinds of ownership for OwnershipQosPolicy.
 */
enum OwnershipQosPolicyKind:octet {
    SHARED_OWNERSHIP_QOS, //!< Shared Ownership, default value.
    EXCLUSIVE_OWNERSHIP_QOS //!< Exclusive ownership
};

/**
 * Class OwnershipQosPolicy, to indicate the ownership kind of the endpoints.
 * kind: Default value SHARED_OWNERSHIP_QOS.
 */
class OwnershipQosPolicy : private Parameter_t, public QosPolicy {
    public:
        RTPS_DllAPI OwnershipQosPolicy():Parameter_t(PID_OWNERSHIP,PARAMETER_KIND_LENGTH),QosPolicy(true),
        kind(SHARED_OWNERSHIP_QOS){};
        virtual RTPS_DllAPI ~OwnershipQosPolicy(){};
        OwnershipQosPolicyKind kind;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Enum OwnershipQosPolicyKind, different kinds of destination order for DestinationOrderQosPolicy.
 */
enum DestinationOrderQosPolicyKind :octet{
    BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS, //!< By Reception Timestamp, default value.
    BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS //!< By Source Timestamp.
};



/**
 * Class DestinationOrderQosPolicy, to indicate the Destination Order Qos.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * kind: Default value BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS
 */
class DestinationOrderQosPolicy : private Parameter_t, public QosPolicy {
    public:
        DestinationOrderQosPolicyKind kind;
        RTPS_DllAPI DestinationOrderQosPolicy():Parameter_t(PID_DESTINATION_ORDER,PARAMETER_KIND_LENGTH),QosPolicy(true),
        kind(BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS){};
        virtual RTPS_DllAPI ~DestinationOrderQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};


/**
 * Class UserDataQosPolicy, to transmit user data during the discovery phase.
 */
class UserDataQosPolicy : private Parameter_t, public QosPolicy{
    friend class ParameterList;
    public:
    RTPS_DllAPI UserDataQosPolicy() :Parameter_t(PID_USER_DATA, 0), QosPolicy(false){};
    virtual RTPS_DllAPI ~UserDataQosPolicy(){};

    /**
     * Appends QoS to the specified CDR message.
     * @param msg Message to append the QoS Policy to.
     * @return True if the modified CDRMessage is valid.
     */
    bool addToCDRMessage(CDRMessage_t* msg) override;

    /**
     * Returns raw data vector.
     * @return raw data as vector of octets.
     * */
    RTPS_DllAPI inline std::vector<octet> getDataVec() const { return dataVec; };
    /**
     * Sets raw data vector.
     * @param vec raw data to set.
     * */
    RTPS_DllAPI inline void setDataVec(const std::vector<octet>& vec){ dataVec = vec; };

    private:

    std::vector<octet> dataVec;
};

/**
 * Class TimeBasedFilterQosPolicy, to indicate the Time Based Filter Qos.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * minimum_separation: Default value c_TimeZero
 */
class TimeBasedFilterQosPolicy : private Parameter_t, public QosPolicy {
    public:

        RTPS_DllAPI TimeBasedFilterQosPolicy():Parameter_t(PID_TIME_BASED_FILTER,PARAMETER_TIME_LENGTH),QosPolicy(false),minimum_separation(c_TimeZero){};
        virtual RTPS_DllAPI ~TimeBasedFilterQosPolicy(){};
        Duration_t minimum_separation;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Enum PresentationQosPolicyAccessScopeKind, different kinds of Presentation Policy order for PresentationQosPolicy.
 */
enum PresentationQosPolicyAccessScopeKind:octet
{
    INSTANCE_PRESENTATION_QOS, //!< Instance Presentation, default value.
    TOPIC_PRESENTATION_QOS, //!< Topic Presentation.
    GROUP_PRESENTATION_QOS //!< Group Presentation.
};

#define PARAMETER_PRESENTATION_LENGTH 8

/**
 * Class PresentationQosPolicy, to indicate the Presentation Qos Policy.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * access_scope: Default value INSTANCE_PRESENTATION_QOS
 * coherent_access: Default value false.
 * ordered_access: Default value false.
 */
class PresentationQosPolicy : private Parameter_t, public QosPolicy
{
    public:
        PresentationQosPolicyAccessScopeKind access_scope;
        bool coherent_access;
        bool ordered_access;
        RTPS_DllAPI PresentationQosPolicy():Parameter_t(PID_PRESENTATION,PARAMETER_PRESENTATION_LENGTH),QosPolicy(false),
        access_scope(INSTANCE_PRESENTATION_QOS),
        coherent_access(false),ordered_access(false){};
        virtual RTPS_DllAPI ~PresentationQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};


/**
 * Class PartitionQosPolicy, to indicate the Partition Qos.
 */
class  PartitionQosPolicy : private Parameter_t, public QosPolicy
{
    friend class ParameterList;
    friend class rtps::EDP;
    public:
    RTPS_DllAPI PartitionQosPolicy() :Parameter_t(PID_PARTITION, 0), QosPolicy(false){};
    virtual RTPS_DllAPI ~PartitionQosPolicy(){};
    /**
     * Appends QoS to the specified CDR message.
     * @param msg Message to append the QoS Policy to.
     * @return True if the modified CDRMessage is valid.
     */
    bool addToCDRMessage(CDRMessage_t* msg) override;

    /**
     * Appends a name to the list of partition names.
     * @param name Name to append.
     */
    RTPS_DllAPI inline void push_back(const char* name){ names.push_back(std::string(name)); hasChanged=true; };
    /**
     * Clears list of partition names
     */
    RTPS_DllAPI inline void clear(){ names.clear(); };
    /**
     * Returns partition names.
     * @return Vector of partition name strings.
     */
    RTPS_DllAPI inline std::vector<std::string> getNames() const { return names; };
    /**
     * Overrides partition names
     * @param nam Vector of partition name strings.
     */
    RTPS_DllAPI inline void setNames(std::vector<std::string>& nam){ names = nam; };

    private:

    std::vector<std::string> names;
};


/**
 * Class TopicDataQosPolicy, to indicate the Topic Data.
 */
class  TopicDataQosPolicy : private Parameter_t, public QosPolicy
{
    friend class ParameterList;
    public:
    RTPS_DllAPI TopicDataQosPolicy() :Parameter_t(PID_TOPIC_DATA, 0), QosPolicy(false){};
    virtual RTPS_DllAPI ~TopicDataQosPolicy(){};
    /**
     * Appends QoS to the specified CDR message.
     * @param msg Message to append the QoS Policy to.
     * @return True if the modified CDRMessage is valid.
     */
    bool addToCDRMessage(CDRMessage_t* msg) override;

    /**
     * Appends topic data.
     * @param oc Data octet.
     */
    RTPS_DllAPI inline void push_back(octet oc){ value.push_back(oc); };
    /**
     * Clears all topic data.
     */
    RTPS_DllAPI inline void clear(){ value.clear(); };
    /**
     * Overrides topic data vector.
     * @param ocv Topic data octet vector.
     */
    RTPS_DllAPI inline void setValue(std::vector<octet> ocv){ value = ocv; };
    /**
     * Returns topic data
     * @return Vector of data octets.
     */
    RTPS_DllAPI inline std::vector<octet> getValue() const { return value; };

    private:

    std::vector<octet> value;
};

/**
 * Class GroupDataQosPolicy, to indicate the Group Data.
 */
class  GroupDataQosPolicy : private Parameter_t, public QosPolicy
{
    friend class ParameterList;
    public:
    RTPS_DllAPI GroupDataQosPolicy() :Parameter_t(PID_GROUP_DATA, 0), QosPolicy(false){}
    virtual RTPS_DllAPI ~GroupDataQosPolicy(){};
    /**
     * Appends QoS to the specified CDR message.
     * @param msg Message to append the QoS Policy to.
     * @return True if the modified CDRMessage is valid.
     */
    bool addToCDRMessage(CDRMessage_t* msg) override;

    /**
     * Appends group data.
     * @param oc Data octet.
     */
    RTPS_DllAPI inline void push_back(octet oc){ value.push_back(oc); };
    /**
     * Clears all group data.
     */
    RTPS_DllAPI inline void clear(){ value.clear(); };
    /**
     * Overrides group data vector.
     * @param ocv Group data octet vector.
     */
    RTPS_DllAPI inline void setValue(std::vector<octet> ocv){ value = ocv; };
    /**
     * Returns group data
     * @return Vector of data octets.
     */
    RTPS_DllAPI inline std::vector<octet> getValue() const { return value; };

    private:

    std::vector<octet> value;
};

/**
 * Enum HistoryQosPolicyKind, different kinds of History Qos for HistoryQosPolicy.
 */
enum HistoryQosPolicyKind:octet {
    KEEP_LAST_HISTORY_QOS, //!< Keep only a number of samples, default value.
    KEEP_ALL_HISTORY_QOS //!< Keep all samples until the ResourceLimitsQosPolicy are exhausted.
};

/**
 * Class HistoryQosPolicy, defines the HistoryQos of the topic in the Writer or Reader side.
 * kind: Default value KEEP_LAST_HISTORY_QOS.
 * depth: Default value 1000.
 */
class HistoryQosPolicy : private Parameter_t, public QosPolicy {
    public:
        HistoryQosPolicyKind kind;
        int32_t depth;
        RTPS_DllAPI HistoryQosPolicy():Parameter_t(PID_HISTORY,PARAMETER_KIND_LENGTH+4),QosPolicy(true),
        kind(KEEP_LAST_HISTORY_QOS),depth(1){};
        virtual RTPS_DllAPI ~HistoryQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Class ResourceLimitsQosPolicy, defines the ResourceLimits for the Writer or the Reader.
 * max_samples: Default value 5000.
 * max_instances: Default value 10.
 * max_samples_per_instance: Default value 400.
 * allocated_samples: Default value 100.
 */
class ResourceLimitsQosPolicy : private Parameter_t, public QosPolicy {
    public:
        int32_t max_samples;
        int32_t max_instances;
        int32_t max_samples_per_instance;
        int32_t allocated_samples;
        RTPS_DllAPI ResourceLimitsQosPolicy():Parameter_t(PID_RESOURCE_LIMITS,4+4+4),QosPolicy(false),
        max_samples(5000),max_instances(10),max_samples_per_instance(400),allocated_samples(100){};
        virtual RTPS_DllAPI ~ResourceLimitsQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};



/**
 * Class DurabilityServiceQosPolicy, to indicate the Durability Service.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * service_cleanup_delay: Default value c_TimeZero.
 * history_kind: Default value KEEP_LAST_HISTORY_QOS.
 * history_depth: Default value 1.
 * max_samples: Default value -1.
 * max_instances: Default value -1.
 * max_samples_per_instance: Default value -1.
 */
class DurabilityServiceQosPolicy : private Parameter_t, public QosPolicy {
    public:
        Duration_t service_cleanup_delay;
        HistoryQosPolicyKind history_kind;
        int32_t history_depth;
        int32_t max_samples;
        int32_t max_instances;
        int32_t max_samples_per_instance;
        RTPS_DllAPI DurabilityServiceQosPolicy():Parameter_t(PID_DURABILITY_SERVICE,PARAMETER_TIME_LENGTH+PARAMETER_KIND_LENGTH+4+4+4+4),QosPolicy(false),
        history_kind(KEEP_LAST_HISTORY_QOS),
        history_depth(1),max_samples(-1),max_instances(-1),max_samples_per_instance(-1){};
        virtual RTPS_DllAPI ~DurabilityServiceQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Class LifespanQosPolicy, currently unimplemented.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * duration: Default value c_TimeInfinite.
 */
class LifespanQosPolicy : private Parameter_t, public QosPolicy {
    public:
        RTPS_DllAPI LifespanQosPolicy():Parameter_t(PID_LIFESPAN,PARAMETER_TIME_LENGTH),QosPolicy(true),duration(c_TimeInfinite){};
        virtual RTPS_DllAPI ~LifespanQosPolicy(){};
        Duration_t duration;
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Class OwnershipStrengthQosPolicy, to indicate the strength of the ownership.
 * value: Default value 0.
 */
class OwnershipStrengthQosPolicy : private Parameter_t, public QosPolicy {
    public:
        uint32_t value;
        RTPS_DllAPI OwnershipStrengthQosPolicy():Parameter_t(PID_OWNERSHIP_STRENGTH,4),QosPolicy(false),value(0){};
        virtual RTPS_DllAPI ~OwnershipStrengthQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};



/**
 * Class TransportPriorityQosPolicy, currently unimplemented.
 * This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.
 * value: Default value 0.
 */
class TransportPriorityQosPolicy : private Parameter_t , public QosPolicy{
    public:
        uint32_t value;
        RTPS_DllAPI TransportPriorityQosPolicy():Parameter_t(PID_TRANSPORT_PRIORITY,4),QosPolicy(false),value(0){};
        virtual RTPS_DllAPI ~TransportPriorityQosPolicy(){};
        /**
         * Appends QoS to the specified CDR message.
         * @param msg Message to append the QoS Policy to.
         * @return True if the modified CDRMessage is valid.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 * Enum PublishModeQosPolicyKind, different kinds of publication synchronism
 */
typedef enum PublishModeQosPolicyKind : octet{
    SYNCHRONOUS_PUBLISH_MODE,	//!< Synchronous publication mode (default for writers).
    ASYNCHRONOUS_PUBLISH_MODE	//!< Asynchronous publication mode.
}PublishModeQosPolicyKind_t;

/**
 * Class PublishModeQosPolicy, defines the publication mode for a specific writer.
 * kind: Default value SYNCHRONOUS_PUBLISH_MODE.
 */
class PublishModeQosPolicy : public QosPolicy {
    public:
        PublishModeQosPolicyKind kind;
        RTPS_DllAPI PublishModeQosPolicy() : kind(SYNCHRONOUS_PUBLISH_MODE){};
        virtual RTPS_DllAPI ~PublishModeQosPolicy(){};
};

}
}

#endif /* QOS_POLICIES_H_ */
