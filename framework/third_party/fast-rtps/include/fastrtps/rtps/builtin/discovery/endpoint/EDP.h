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
 * @file EDP.h
 *
 */

#ifndef EDP_H_
#define EDP_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include "../../../attributes/RTPSParticipantAttributes.h"
#include "../../../common/Guid.h"

namespace eprosima {
namespace fastrtps{

class TopicAttributes;
class ReaderQos;
class WriterQos;

namespace rtps {

class PDPSimple;
class ParticipantProxyData;
class RTPSWriter;
class RTPSReader;
class ReaderProxyData;
class WriterProxyData;
class RTPSParticipantImpl;

/**
 * Class EDP, base class for Endpoint Discovery Protocols. It contains generic methods used by the two EDP implemented (EDPSimple and EDPStatic), as well as abstract methods
 * definitions required by the specific implementations.
 * @ingroup DISCOVERY_MODULE
 */
class EDP
{
    public:
        /**
         * Constructor.
         * @param p Pointer to the PDPSimple
         * @param part Pointer to the RTPSParticipantImpl
         */
        EDP(PDPSimple* p,RTPSParticipantImpl* part);
        virtual ~EDP();

        /**
         * Abstract method to initialize the EDP.
         * @param attributes DiscoveryAttributes structure.
         * @return True if correct.
         */
        virtual bool initEDP(BuiltinAttributes& attributes)=0;
        /**
         * Abstract method that assigns remote endpoints when a new RTPSParticipantProxyData is discovered.
         * @param pdata Discovered ParticipantProxyData
         */
        virtual void assignRemoteEndpoints(ParticipantProxyData* pdata)=0;
        /**
         * Remove remote endpoints from the endpoint discovery protocol
         * @param pdata Pointer to the ParticipantProxyData to remove
         */
        virtual void removeRemoteEndpoints(ParticipantProxyData* pdata){(void) pdata;};

        /**
         * Abstract method that removes a local Reader from the discovery method
         * @param R Pointer to the Reader to remove.
         * @return True if correctly removed.
         */
        virtual bool removeLocalReader(RTPSReader* R)=0;
        /**
         * Abstract method that removes a local Writer from the discovery method
         * @param W Pointer to the Writer to remove.
         * @return True if correctly removed.
         */
        virtual bool removeLocalWriter(RTPSWriter*W)=0;

        /**
         * After a new local ReaderProxyData has been created some processing is needed (depends on the implementation).
         * @param rdata Pointer to the ReaderProxyData object.
         * @return True if correct.
         */
        virtual bool processLocalReaderProxyData(ReaderProxyData* rdata)= 0;
        /**
         * After a new local WriterProxyData has been created some processing is needed (depends on the implementation).
         * @param wdata Pointer to the Writer ProxyData object.
         * @return True if correct.
         */
        virtual bool processLocalWriterProxyData(WriterProxyData* wdata)= 0;

        /**
         * Create a new ReaderPD for a local Reader.
         * @param R Pointer to the RTPSReader.
         * @param att Attributes of the associated topic
         * @param qos QoS policies dictated by the subscriber
         * @return True if correct.
         */
        bool newLocalReaderProxyData(RTPSReader* R,TopicAttributes& att, ReaderQos& qos);
        /**
         * Create a new ReaderPD for a local Writer.
         * @param W Pointer to the RTPSWriter.
         * @param att Attributes of the associated topic
         * @param qos QoS policies dictated by the publisher
         * @return True if correct.
         */
        bool newLocalWriterProxyData(RTPSWriter* W,TopicAttributes& att, WriterQos& qos);
        /**
         * A previously created Reader has been updated
         * @param R Pointer to the reader;
         * @param qos QoS policies dictated by the subscriber
         * @return True if correctly updated
         */
        bool updatedLocalReader(RTPSReader* R,ReaderQos& qos);
        /**
         * A previously created Writer has been updated
         * @param W Pointer to the Writer
         * @param qos QoS policies dictated by the publisher
         * @return True if correctly updated
         */
        bool updatedLocalWriter(RTPSWriter* W,WriterQos& qos);
        /**
         * Check the validity of a matching between a RTPSWriter and a ReaderProxyData object.
         * @param wdata Pointer to the WriterProxyData object.
         * @param rdata Pointer to the ReaderProxyData object.
         * @return True if the two can be matched.
         */
        bool validMatching(WriterProxyData* wdata,ReaderProxyData* rdata);
        /**
         * Check the validity of a matching between a RTPSReader and a WriterProxyData object.
         * @param rdata Pointer to the ReaderProxyData object.
         * @param wdata Pointer to the WriterProxyData object.
         * @return True if the two can be matched.
         */
        bool validMatching(ReaderProxyData* rdata,WriterProxyData* wdata);
        /**
         * Remove a WriterProxyDataObject based on its GUID_t.
         * @param writer Reference to the writer GUID.
         * @return True if correct.
         */
        bool removeWriterProxy(const GUID_t& writer);
        /**
         * Remove a ReaderProxyDataObject based on its GUID_t.
         * @param reader Reference to the reader GUID.
         * @return True if correct.
         */
        bool removeReaderProxy(const GUID_t& reader);
        /**
         * Unpair a WriterProxyData object from all local readers.
         * @param pdata Pointer to the participant proxy data.
         * @param wdata Pointer to the WriterProxyData object.
         * @return True if correct.
         */
        bool unpairWriterProxy(ParticipantProxyData* pdata, WriterProxyData* wdata);
        /**
         * Unpair a ReaderProxyData object from all local writers.
         * @param rdata Pointer to the ReaderProxyData object.
         * @param pdata Pointer to the participant proxy data.
         * @return True if correct.
         */
        bool unpairReaderProxy(ParticipantProxyData* pdata, ReaderProxyData* rdata);

        /**
         * Try to pair/unpair ReaderProxyData.
         * @param pdata Pointer to the participant proxy data.
         * @param rdata Pointer to the ReaderProxyData object.
         * @return True.
         */
        bool pairingReaderProxy(ParticipantProxyData* pdata, ReaderProxyData* rdata);

#if HAVE_SECURITY
        bool pairingLaterReaderProxy(const GUID_t local_writer, ParticipantProxyData& pdata, ReaderProxyData& rdata);
#endif

        /**
         * Try to pair/unpair WriterProxyData.
         * @param pdata Pointer to the participant proxy data.
         * @param wdata Pointer to the WriterProxyData.
         * @return True.
         */
        bool pairingWriterProxy(ParticipantProxyData* pdata, WriterProxyData* wdata);

#if HAVE_SECURITY
        bool pairingLaterWriterProxy(const GUID_t local_reader, ParticipantProxyData& pdata, WriterProxyData& wdata);
#endif

        //! Pointer to the PDPSimple object that contains the endpoint discovery protocol.
        PDPSimple* mp_PDP;
        //! Pointer to the RTPSParticipant.
        RTPSParticipantImpl* mp_RTPSParticipant;

    private:

        /**
         * Try to pair/unpair a local Reader against all possible writerProxy Data.
         * @param R Pointer to the Reader
         * @return True
         */
        bool pairingReader(RTPSReader* R);
        /**l
         * Try to pair/unpair a local Writer against all possible readerProxy Data.
         * @param W Pointer to the Writer
         * @return True
         */
        bool pairingWriter(RTPSWriter* W);

};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* EDP_H_ */
