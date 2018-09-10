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
 * @file RTPSParticipantAttributes.h
 */

#ifndef _RTPSPARTICIPANTPARAMETERS_H_
#define _RTPSPARTICIPANTPARAMETERS_H_

#include "../common/Time_t.h"
#include "../common/Locator.h"
#include "PropertyPolicy.h"
#include "../flowcontrol/ThroughputControllerDescriptor.h"
#include "../../transport/TransportInterface.h"

#include <memory>

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
 * Class SimpleEDPAttributes, to define the attributes of the Simple Endpoint Discovery Protocol.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class SimpleEDPAttributes
{
    public:

        //!Default value true.
        bool use_PublicationWriterANDSubscriptionReader;
        //!Default value true.
        bool use_PublicationReaderANDSubscriptionWriter;
        SimpleEDPAttributes():
            use_PublicationWriterANDSubscriptionReader(true),
            use_PublicationReaderANDSubscriptionWriter(true)
        {

        }
};

/**
 * Class PortParameters, to define the port parameters and gains related with the RTPS protocol.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class PortParameters
{
    public:
        PortParameters()
        {
            portBase = 7400;
            participantIDGain = 2;
            domainIDGain = 250;
            offsetd0 = 0;
            offsetd1 = 10;
            offsetd2 = 1;
            offsetd3 = 11;
        };
        virtual ~PortParameters(){}
        /**
         * Get a multicast port based on the domain ID.
         *
         * @param domainId Domain ID.
         * @return Multicast port
         */
        inline uint32_t getMulticastPort(uint32_t domainId)
        {
            return portBase+ domainIDGain * domainId+ offsetd0;
        }
        /**
         * Get a unicast port baes on the domain ID and the participant ID.
         *
         * @param domainId Domain ID.
         * @param RTPSParticipantID Participant ID.
         * @return Unicast port
         */
        inline uint32_t getUnicastPort(uint32_t domainId,uint32_t RTPSParticipantID)
        {
            return portBase+ domainIDGain * domainId	+ offsetd1	+ participantIDGain * RTPSParticipantID;
        }
        //!PortBase, default value 7400.
        uint16_t portBase;
        //!DomainID gain, default value 250.
        uint16_t domainIDGain;
        //!ParticipantID gain, default value 2.
        uint16_t participantIDGain;
        //!Offset d0, default value 0.
        uint16_t offsetd0;
        //!Offset d1, default value 10.
        uint16_t offsetd1;
        //!Offset d2, default value 1.
        uint16_t offsetd2;
        //!Offset d3, default value 11.
        uint16_t offsetd3;
};

/**
 * Class BuiltinAttributes, to define the behavior of the RTPSParticipant builtin protocols.
 * @ingroup RTPS_ATTRIBUTES_MODULE
 */
class BuiltinAttributes{
    public:
        /**
         * If set to false, NO discovery whatsoever would be used.
         * Publisher and Subscriber defined with the same topic name would NOT be linked. All matching must be done
         * manually through the addReaderLocator, addReaderProxy, addWriterProxy methods.
         */
        bool use_SIMPLE_RTPSParticipantDiscoveryProtocol;

        //!Indicates to use the WriterLiveliness protocol.
        bool use_WriterLivelinessProtocol;
        /**
         * If set to true, SimpleEDP would be used.
         */
        bool use_SIMPLE_EndpointDiscoveryProtocol;
        /**
         * If set to true, StaticEDP based on an XML file would be implemented.
         * The XML filename must be provided.
         */
        bool use_STATIC_EndpointDiscoveryProtocol;

        /**
         * DomainId to be used by the RTPSParticipant (80 by default).
         */
        uint32_t domainId;
        //!Lease Duration of the RTPSParticipant, indicating how much time remote RTPSParticipants should consider this RTPSParticipant alive.
        Duration_t leaseDuration;
        /**
         * The period for the RTPSParticipant to send its Discovery Message to all other discovered RTPSParticipants
         * as well as to all Multicast ports.
         */
        Duration_t leaseDuration_announcementperiod;
        //!Attributes of the SimpleEDP protocol
        SimpleEDPAttributes m_simpleEDP;
        //!Metatraffic Unicast Locator List
        LocatorList_t metatrafficUnicastLocatorList;
        //!Metatraffic Multicast Locator List.
        LocatorList_t metatrafficMulticastLocatorList;
        //! Initial peers.
        LocatorList_t initialPeersList;

        BuiltinAttributes()
        {
            use_SIMPLE_RTPSParticipantDiscoveryProtocol = true;
            use_SIMPLE_EndpointDiscoveryProtocol = true;
            use_STATIC_EndpointDiscoveryProtocol = false;
            m_staticEndpointXMLFilename = "";
            domainId = 0;
            leaseDuration.seconds = 500;
            leaseDuration_announcementperiod.seconds = 250;
            use_WriterLivelinessProtocol = true;

        };
        virtual ~BuiltinAttributes(){};
        /**
         * Get the static endpoint XML filename
         * @return Static endpoint XML filename
         */
        const char* getStaticEndpointXMLFilename(){ return m_staticEndpointXMLFilename.c_str(); };
        /**
         * Set the static endpoint XML filename
         * @param str Static endpoint XML filename
         */
        void setStaticEndpointXMLFilename(const char* str){ m_staticEndpointXMLFilename = std::string(str); };
    private:
        //! StaticEDP XML filename, only necessary if use_STATIC_EndpointDiscoveryProtocol=true
        std::string m_staticEndpointXMLFilename;
};



/**
 * Class RTPSParticipantAttributes used to define different aspects of a RTPSParticipant.
 *@ingroup RTPS_ATTRIBUTES_MODULE
 */
class RTPSParticipantAttributes
{
    public:

        RTPSParticipantAttributes()
        {
            defaultSendPort = 10040;
            setName("RTPSParticipant");
            sendSocketBufferSize = 0;
            listenSocketBufferSize = 0;
            use_IP4_to_send = true;
            use_IP6_to_send = false;
            participantID = -1;
            useBuiltinTransports = true;
        }

        virtual ~RTPSParticipantAttributes(){};

        /**
         * Default list of Unicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the case
         * that it was defined with NO UnicastLocators. At least ONE locator should be included in this list.
         */
        LocatorList_t defaultUnicastLocatorList;

        /**
         * Default list of Multicast Locators to be used for any Endpoint defined inside this RTPSParticipant in the case
         * that it was defined with NO UnicastLocators. This is usually left empty.
         */
        LocatorList_t defaultMulticastLocatorList;

        /**
         * Default list of Locators used to send messages through. Used to link with SenderResources in the case and 
         * Endpoint is created with NO outLocators. This list contains the default outLocators for the Transports implemented
         * by eProsima.
         */
        LocatorList_t defaultOutLocatorList;

        /**
         * Default send port that all Endpoints in the RTPSParticipant would use to send messages, default value 10040.
         * In this release all Endpoints use the same resource (socket) to send messages.
         */
        uint32_t defaultSendPort;

        /*!
         * @brief Send socket buffer size for the send resource. Zero value indicates to use default system buffer size.
         * Default value: 0.
         */
        uint32_t sendSocketBufferSize;

        /*! Listen socket buffer for all listen resources. Zero value indicates to use default system buffer size.
         * Default value: 0.
         */
        uint32_t listenSocketBufferSize;

        //! Builtin parameters.
        BuiltinAttributes builtin;
        //!Port Parameters
        PortParameters port;
        //!User Data of the participant
        std::vector<octet> userData;
        //!Participant ID
        int32_t participantID;
        //!Use IP4 to send messages.
        bool use_IP4_to_send;
        //!Use IP6 to send messages.
        bool use_IP6_to_send;
        //!Set the name of the participant.
        inline void setName(const char* nam){name = nam;}
        //!Get the name of the participant.
        inline const char* getName() const {return name.c_str();}
        //!Throughput controller parameters. Leave default for uncontrolled flow.
        ThroughputControllerDescriptor throughputController; 
        //!User defined transports to use alongside or in place of builtins.
        std::vector<std::shared_ptr<TransportDescriptorInterface> > userTransports;
        //!Set as false to disable the default UDPv4 implementation.
        bool useBuiltinTransports;

        //! Property policies
        PropertyPolicy properties;

    private:
        //!Name of the participant.
        std::string name;
};

}
} /* namespace rtps */
} /* namespace eprosima */

#endif /* _RTPSPARTICIPANTPARAMETERS_H_ */
