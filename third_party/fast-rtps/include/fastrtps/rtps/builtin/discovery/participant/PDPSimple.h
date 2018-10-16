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
 * @file PDPSimple.h
 *
 */

#ifndef PDPSIMPLE_H_
#define PDPSIMPLE_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

#include <mutex>
#include "../../../common/Guid.h"
#include "../../../attributes/RTPSParticipantAttributes.h"

#include "../../../../qos/QosPolicies.h"

using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{
namespace rtps {

class StatelessWriter;
class StatelessReader;
class WriterHistory;
class ReaderHistory;
class RTPSParticipantImpl;
class BuiltinProtocols;
class EDP;
class ResendParticipantProxyDataPeriod;
class RemoteParticipantLeaseDuration;
class ReaderProxyData;
class WriterProxyData;
class ParticipantProxyData;
class PDPSimpleListener;


/**
 * Class PDPSimple that implements the SimpleRTPSParticipantDiscoveryProtocol as defined in the RTPS specification.
 *@ingroup DISCOVERY_MODULE
 */
class PDPSimple 
{
    friend class ResendRTPSParticipantProxyDataPeriod;
    friend class RemoteRTPSParticipantLeaseDuration;
    friend class PDPSimpleListener;
    public:
    /**
     * Constructor
     * @param builtin Pointer to the BuiltinProcols object.
     */
    PDPSimple(BuiltinProtocols* builtin);
    virtual ~PDPSimple();

    void initializeParticipantProxyData(ParticipantProxyData* participant_data);

    /**
     * Initialize the PDP.
     * @param part Pointer to the RTPSParticipant.
     * @return True on success
     */
    bool initPDP(RTPSParticipantImpl* part);

    /**
     * Force the sending of our local DPD to all remote RTPSParticipants and multicast Locators.
     * @param new_change If true a new change (with new seqNum) is created and sent; if false the last change is re-sent
     * @param dispose Sets change kind to NOT_ALIVE_DISPOSED_UNREGISTERED 
     */
    void announceParticipantState(bool new_change, bool dispose = false);
    //!Stop the RTPSParticipantAnnouncement (only used in tests).
    void stopParticipantAnnouncement();
    //!Reset the RTPSParticipantAnnouncement (only used in tests).
    void resetParticipantAnnouncement();

    /**
     * Add a ReaderProxyData to the correct ParticipantProxyData.
     * @param rdata Pointer to the ReaderProxyData objectr to add.
     * @param copydata Boolean variable indicating the need to copy the passed object.
     * @param returnReaderProxyData Pointer to pointer in case you wanted the data copied.
     * @param pdata Pointer to the associated ParticipantProxyData.
     * @return True if correct.
     */
    bool addReaderProxyData(ReaderProxyData* rdata,bool copydata=false,
            ReaderProxyData** returnReaderProxyData=nullptr,
            ParticipantProxyData** pdata = nullptr);

    /**
     * Add a WriterProxyData to the correct ParticipantProxyData.
     * @param wdata Pointer to the WriterProxyData objectr to add.
     * @param copydata Boolean variable indicating the need to copy the passed object.
     * @param returnWriterProxyData Pointer to pointer in case you wanted the data copied.
     * @param pdata Pointer to the associated ParticipantProxyData.
     * @return True if correct.
     */
    bool addWriterProxyData(WriterProxyData* wdata,bool copydata=false,
            WriterProxyData** returnWriterProxyData=nullptr,
            ParticipantProxyData** pdata = nullptr);

    /**
     * This method returns a pointer to a ReaderProxyData object if it is found among the registered RTPSParticipants (including the local RTPSParticipant).
     * @param[in] reader GUID_t of the reader we are looking for.
     * @param rdata Pointer to pointer of the ReaderProxyData object.
     * @param pdata Pointer to pointer of the ParticipantProxyData object.
     * @return True if found.
     */
    bool lookupReaderProxyData(const GUID_t& reader, ReaderProxyData** rdata, ParticipantProxyData** pdata);
    /**
     * This method returns a pointer to a WriterProxyData object if it is found among the registered RTPSParticipants (including the local RTPSParticipant).
     * @param[in] writer GUID_t of the writer we are looking for.
     * @param wdata Pointer to pointer of the WriterProxyData object.
     * @param pdata Pointer to pointer of the ParticipantProxyData object.
     * @return True if found.
     */
    bool lookupWriterProxyData(const GUID_t& writer, WriterProxyData** wdata, ParticipantProxyData** pdata);
    /**
     * This method returns a pointer to a RTPSParticipantProxyData object if it is found among the registered RTPSParticipants.
     * @param[in] pguid GUID_t of the RTPSParticipant we are looking for.
     * @param pdata Pointer to pointer of the ParticipantProxyData object.
     * @return True if found.
     */
    bool lookupParticipantProxyData(const GUID_t& pguid,ParticipantProxyData** pdata);
    /**
     * This method removes and deletes a ReaderProxyData object from its corresponding RTPSParticipant.
     * @param rdata Pointer to the ReaderProxyData object.
     * @param pdata Pointer to pointer of the ParticipantProxyData object.
     * @return true if found and deleted.
     */
    bool removeReaderProxyData(ParticipantProxyData* pdata, ReaderProxyData* rdata);
    /**
     * This method removes and deletes a WriterProxyData object from its corresponding RTPSParticipant.
     * @param wdata Pointer to the WriterProxyData object.
     * @param pdata Pointer to pointer of the ParticipantProxyData object.
     * @return true if found and deleted.
     */
    bool removeWriterProxyData(ParticipantProxyData* pdata, WriterProxyData* wdata);

    /**
     * This method assigns remtoe endpoints to the builtin endpoints defined in this protocol. It also calls the corresponding methods in EDP and WLP.
     * @param pdata Pointer to the RTPSParticipantProxyData object.
     */
    void assignRemoteEndpoints(ParticipantProxyData* pdata);

    void notifyAboveRemoteEndpoints(ParticipantProxyData* pdata);

    /**
     * Remove remote endpoints from the participant discovery protocol
     * @param pdata Pointer to the ParticipantProxyData to remove
     */
    void removeRemoteEndpoints(ParticipantProxyData* pdata);

    /**
     * This method removes a remote RTPSParticipant and all its writers and readers.
     * @param partGUID GUID_t of the remote RTPSParticipant.
     * @return true if correct.
     */
    bool removeRemoteParticipant(GUID_t& partGUID);
    //!Pointer to the builtin protocols object.
    BuiltinProtocols* mp_builtin;
    /**
     * Get a pointer to the local RTPSParticipant RTPSParticipantProxyData object.
     * @return Pointer to the local RTPSParticipant RTPSParticipantProxyData object.
     */
    ParticipantProxyData* getLocalParticipantProxyData()
    {
        return m_participantProxies.front();
    }
    /**
     * Get a pointer to the EDP object.
     * @return pointer to the EDP object.
     */
    inline EDP* getEDP(){return mp_EDP;}
    /**
     * Get a cons_iterator to the beginning of the RTPSParticipant Proxies.
     * @return const_iterator.
     */
    std::vector<ParticipantProxyData*>::const_iterator ParticipantProxiesBegin(){return m_participantProxies.begin();};
    /**
     * Get a cons_iterator to the end RTPSParticipant Proxies.
     * @return const_iterator.
     */
    std::vector<ParticipantProxyData*>::const_iterator ParticipantProxiesEnd(){return m_participantProxies.end();};

    /**
     * Assert the liveliness of a Remote Participant.
     * @param guidP GuidPrefix_t of the participant whose liveliness is being asserted.
     */
    void assertRemoteParticipantLiveliness(const GuidPrefix_t& guidP);

    /**
     * Assert the liveliness of a Local Writer.
     * @param kind LivilinessQosPolicyKind to be asserted.
     */
    void assertLocalWritersLiveliness(LivelinessQosPolicyKind kind);

    /**
     * Assert the liveliness of remote writers.
     * @param guidP GuidPrefix_t of the participant whose writers liveliness is begin asserted.
     * @param kind LivelinessQosPolicyKind of the writers.
     */
    void assertRemoteWritersLiveliness(GuidPrefix_t& guidP,LivelinessQosPolicyKind kind);

    /**
     * Activate a new Remote Endpoint that has been statically discovered.
     * @param pguid GUID_t of the participant.
     * @param userDefinedId User Defined ID.
     * @param kind Kind of endpoint.
     */
    bool newRemoteEndpointStaticallyDiscovered(const GUID_t& pguid, int16_t userDefinedId,EndpointKind_t kind);

    /**
     * Get the RTPS participant
     * @return RTPS participant
     */
    inline RTPSParticipantImpl* getRTPSParticipant() const {return mp_RTPSParticipant;};
    /**
     * Get the mutex.
     * @return Pointer to the Mutex
     */
    inline std::recursive_mutex* getMutex() const {return mp_mutex;}

    CDRMessage_t get_participant_proxy_data_serialized(Endianness_t endian);

    private:
    //!Pointer to the local RTPSParticipant.
    RTPSParticipantImpl* mp_RTPSParticipant;
    //!Discovery attributes.
    BuiltinAttributes m_discovery;
    //!Pointer to the SPDPWriter.
    StatelessWriter* mp_SPDPWriter;
    //!Pointer to the SPDPReader.
    StatelessReader* mp_SPDPReader;
    //!Pointer to the EDP object.
    EDP* mp_EDP;
    //!Registered RTPSParticipants (including the local one, that is the first one.)
    std::vector<ParticipantProxyData*> m_participantProxies;
    //!Variable to indicate if any parameter has changed.
    bool m_hasChangedLocalPDP;
    //!TimedEvent to periodically resend the local RTPSParticipant information.
    ResendParticipantProxyDataPeriod* mp_resendParticipantTimer;
    //!Listener for the SPDP messages.
    PDPSimpleListener* mp_listener;
    //!WriterHistory
    WriterHistory* mp_SPDPWriterHistory;
    //!Reader History
    ReaderHistory* mp_SPDPReaderHistory;

    /**
     * Create the SPDP Writer and Reader
     * @return True if correct.
     */
    bool createSPDPEndpoints();
    std::recursive_mutex* mp_mutex;



};

}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* PDPSIMPLE_H_ */
