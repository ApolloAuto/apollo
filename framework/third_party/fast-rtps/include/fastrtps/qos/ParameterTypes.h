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
 * @file ParameterTypes.h
*/

#ifndef PARAMETERTYPES_H_
#define PARAMETERTYPES_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../rtps/common/all_common.h"
#include "../rtps/common/Token.h"


#include <string>
#include <vector>

using namespace eprosima::fastrtps::rtps;


namespace eprosima {
namespace fastrtps {

namespace rtps{
struct CDRMessage_t;
}


using namespace rtps;




#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * @addtogroup PARAMETER_MODULE
 * @{
 */

enum ParameterId_t	: uint16_t
{
    PID_PAD = 0x0000,
    PID_SENTINEL = 0x0001,
    PID_USER_DATA = 0x002c,
    PID_TOPIC_NAME = 0x0005,
    PID_TYPE_NAME = 0x0007,
    PID_GROUP_DATA =0x002d,
    PID_TOPIC_DATA =0x002e,
    PID_DURABILITY =0x001d,
    PID_DURABILITY_SERVICE =0x001e,
    PID_DEADLINE =0x0023,
    PID_LATENCY_BUDGET =0x0027,
    PID_LIVELINESS =0x001b,
    PID_RELIABILITY =0x001A,
    PID_LIFESPAN =0x002b,
    PID_DESTINATION_ORDER =0x0025,
    PID_HISTORY =0x0040,
    PID_RESOURCE_LIMITS =0x0041,
    PID_OWNERSHIP =0x001f,
    PID_OWNERSHIP_STRENGTH =0x0006,
    PID_PRESENTATION =0x0021,
    PID_PARTITION =0x0029,
    PID_TIME_BASED_FILTER =0x0004,
    PID_TRANSPORT_PRIORITY =0x0049,
    PID_PROTOCOL_VERSION = 0x0015,
    PID_VENDORID = 0x0016,
    PID_UNICAST_LOCATOR = 0x002f,
    PID_MULTICAST_LOCATOR = 0x0030,
    PID_MULTICAST_IPADDRESS =0x0011,
    PID_DEFAULT_UNICAST_LOCATOR = 0x0031,
    PID_DEFAULT_MULTICAST_LOCATOR = 0x0048,
    PID_METATRAFFIC_UNICAST_LOCATOR = 0x0032,
    PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033,
    PID_DEFAULT_UNICAST_IPADDRESS =0x000c,
    PID_DEFAULT_UNICAST_PORT = 0x000e,
    PID_METATRAFFIC_UNICAST_IPADDRESS =0x0045,
    PID_METATRAFFIC_UNICAST_PORT = 0x000d,
    PID_METATRAFFIC_MULTICAST_IPADDRESS =0x000b,
    PID_METATRAFFIC_MULTICAST_PORT = 0x0046,
    PID_EXPECTS_INLINE_QOS =0x0043,
    PID_PARTICIPANT_MANUAL_LIVELINESS_COUNT =0x0034,
    PID_PARTICIPANT_BUILTIN_ENDPOINTS = 0x0044,
    PID_PARTICIPANT_LEASE_DURATION = 0x0002,
    PID_CONTENT_FILTER_PROPERTY =0x0035,
    PID_PARTICIPANT_GUID = 0x0050,
    PID_PARTICIPANT_ENTITYID =0x0051,
    PID_GROUP_GUID =0x0052,
    PID_GROUP_ENTITYID =0x0053,
    PID_BUILTIN_ENDPOINT_SET = 0x0058,
    PID_PROPERTY_LIST = 0x0059,
    PID_TYPE_MAX_SIZE_SERIALIZED =0x0060,
    PID_ENTITY_NAME = 0x0062,
    PID_KEY_HASH = 0x0070,
    PID_STATUS_INFO = 0x0071,
    PID_ENDPOINT_GUID = 0x005a,
    //PID_RELATED_SAMPLE_IDENTITY = 0x0083
    PID_IDENTITY_TOKEN = 0x1001,
    PID_RELATED_SAMPLE_IDENTITY = 0x800f
};









//!Base Parameter class with parameter PID and parameter length in bytes.
//!@ingroup PARAMETER_MODULE
class Parameter_t {
    public:
        //!Parameter ID
        ParameterId_t Pid;
        //!Parameter length
        uint16_t length;
        RTPS_DllAPI Parameter_t();
        virtual RTPS_DllAPI ~Parameter_t();
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param length Its associated length
         */
        RTPS_DllAPI Parameter_t(ParameterId_t pid,uint16_t length);
        /**
         * Virtual method used to add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        virtual bool addToCDRMessage(CDRMessage_t* msg) = 0;
};

/**
 *@ingroup PARAMETER_MODULE
 */
class ParameterKey_t:public Parameter_t{
    public:
        InstanceHandle_t key;
        ParameterKey_t(){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterKey_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){};
        ParameterKey_t(ParameterId_t pid,uint16_t in_length,InstanceHandle_t& ke):Parameter_t(pid,in_length),key(ke){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 *
 */
class ParameterLocator_t: public Parameter_t {
    public:
        Locator_t locator;
        ParameterLocator_t(){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterLocator_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){};
        ParameterLocator_t(ParameterId_t pid,uint16_t in_length,Locator_t& loc):Parameter_t(pid,in_length),locator(loc){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};
#define PARAMETER_LOCATOR_LENGTH 24


/**
 *
 */
class ParameterString_t: public Parameter_t {
    public:
        ParameterString_t(){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterString_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){};
        ParameterString_t(ParameterId_t pid,uint16_t in_length,std::string& strin):Parameter_t(pid,in_length),m_string(strin){}
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
        inline const char* getName()const { return m_string.c_str(); };
        inline void setName(const char* name){ m_string = std::string(name); };
    private:
        std::string m_string;
};

/**
 *
 */
class ParameterPort_t: public Parameter_t {
    public:
        uint32_t port;
        ParameterPort_t():port(0){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterPort_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length),port(0){};
        ParameterPort_t(ParameterId_t pid,uint16_t in_length,uint32_t po):Parameter_t(pid,in_length),port(po){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_PORT_LENGTH 4

/**
 *
 */
class ParameterGuid_t: public Parameter_t {
    public:
        GUID_t guid;
        ParameterGuid_t(){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterGuid_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){};
        ParameterGuid_t(ParameterId_t pid,uint16_t in_length,GUID_t guidin):Parameter_t(pid,in_length),guid(guidin){};
        ParameterGuid_t(ParameterId_t pid,uint16_t in_length,InstanceHandle_t& iH):Parameter_t(pid,in_length)
        {
            for(uint8_t i =0;i<16;++i)
            {
                if(i<12)
                    guid.guidPrefix.value[i] = iH.value[i];
                else
                    guid.entityId.value[i-12] = iH.value[i];
            }
        };
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_GUID_LENGTH 16

/**
 *
 */
class ParameterProtocolVersion_t: public Parameter_t {
    public:
        ProtocolVersion_t protocolVersion;
        ParameterProtocolVersion_t(){protocolVersion = c_ProtocolVersion;};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterProtocolVersion_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){protocolVersion = c_ProtocolVersion;};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_PROTOCOL_LENGTH 4

/**
 *
 */
class ParameterVendorId_t:public Parameter_t{
    public:
        VendorId_t vendorId;
        ParameterVendorId_t(){set_VendorId_eProsima(vendorId);};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterVendorId_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){set_VendorId_eProsima(vendorId);};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_VENDOR_LENGTH 4

/**
 *
 */
class ParameterIP4Address_t :public Parameter_t{
    public:
        octet address[4];
        ParameterIP4Address_t(){this->setIP4Address(0,0,0,0);};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterIP4Address_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){this->setIP4Address(0,0,0,0);};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
        void setIP4Address(octet o1,octet o2,octet o3,octet o4);
};

#define PARAMETER_IP4_LENGTH 4

/**
 *
 */
class ParameterBool_t:public Parameter_t{
    public:
        bool value;
        ParameterBool_t():value(false){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterBool_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length),value(false){};
        ParameterBool_t(ParameterId_t pid,uint16_t in_length,bool inbool):Parameter_t(pid,in_length),value(inbool){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_BOOL_LENGTH 4

/**
 *
 */
class ParameterCount_t:public Parameter_t{
    public:
        Count_t count;
        ParameterCount_t():count(0){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterCount_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length),count(0){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_COUNT_LENGTH 4

/**
 *
 */
class ParameterEntityId_t:public Parameter_t{
    public:
        EntityId_t entityId;
        ParameterEntityId_t():entityId(ENTITYID_UNKNOWN){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterEntityId_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length),entityId(ENTITYID_UNKNOWN){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_ENTITYID_LENGTH 4

/**
 *
 */
class ParameterTime_t:public Parameter_t{
    public:
        Time_t time;
        ParameterTime_t(){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterTime_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_TIME_LENGTH 8

/**
 *
 */
class ParameterBuiltinEndpointSet_t:public Parameter_t{
    public:
        BuiltinEndpointSet_t endpointSet;
        ParameterBuiltinEndpointSet_t():endpointSet(0){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterBuiltinEndpointSet_t(ParameterId_t pid,uint16_t in_length):Parameter_t(pid,in_length),endpointSet(0){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

#define PARAMETER_BUILTINENDPOINTSET_LENGTH 4


/**
 *
 */
class ParameterPropertyList_t:public Parameter_t{
    public:
        std::vector<std::pair<std::string,std::string>> properties;
        ParameterPropertyList_t():Parameter_t(PID_PROPERTY_LIST,0){};
        /**
         * Constructor using a parameter PID and the parameter length
         * @param in_length Its associated length
         */
        ParameterPropertyList_t(ParameterId_t /*pid*/, uint16_t in_length) : Parameter_t(PID_PROPERTY_LIST,in_length){};
        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 *
 */
class ParameterSampleIdentity_t : public Parameter_t
{
    public:
        SampleIdentity sample_id;

        ParameterSampleIdentity_t() : sample_id(SampleIdentity::unknown()) {}

        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterSampleIdentity_t(ParameterId_t pid, uint16_t in_length) : Parameter_t(pid,in_length), sample_id(SampleIdentity::unknown()) {}

        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};

/**
 *
 */
class ParameterToken_t : public Parameter_t
{
    public:
        Token token;

        ParameterToken_t() {}

        /**
         * Constructor using a parameter PID and the parameter length
         * @param pid Pid of the parameter
         * @param in_length Its associated length
         */
        ParameterToken_t(ParameterId_t pid, uint16_t in_length) : Parameter_t(pid,in_length) {}

        /**
         * Add the parameter to a CDRMessage_t message.
         * @param[in,out] msg Pointer to the message where the parameter should be added.
         * @return True if the parameter was correctly added.
         */
        bool addToCDRMessage(CDRMessage_t* msg) override;
};



///@}

#endif

} //end of namespace
} //end of namespace eprosima

#endif
#endif /* PARAMETERTYPES_H_ */
