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

/*!
 * @file Authentication.h
 */
#ifndef _RTPS_SECURITY_AUTHENTICATION_AUTHENTICATION_H_
#define _RTPS_SECURITY_AUTHENTICATION_AUTHENTICATION_H_

#include "../common/Handle.h"
#include "../common/SharedSecretHandle.h"
#include "../../common/Guid.h"
#include "../../attributes/RTPSParticipantAttributes.h"
#include "../exceptions/SecurityException.h"
#include "../../common/Token.h"
#include "../../common/CDRMessage_t.h"
#include "Handshake.h"

#include <cstdint>

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

enum ValidationResult_t : uint32_t
{
    VALIDATION_OK = 0,
    VALIDATION_FAILED,
    VALIDATION_PENDING_RETRY,
    VALIDATION_PENDING_HANDSHAKE_REQUEST,
    VALIDATION_PENDING_HANDSHAKE_MESSAGE,
    VALIDATION_OK_WITH_FINAL_MESSAGE
};

class Authentication;

class AuthenticationListener
{
    virtual bool on_revoke_identity(Authentication& plugin,
            const IdentityHandle& handle,
            SecurityException& exception) = 0;
};

class Authentication
{
    public:

        virtual ~Authentication() {}

        /*!
         * @brief Validates the identity of the local RTPSParticipant.
         * @param local_identity_handle (out) A handle that can be used to locally refer to the Authenticated
         * Participant in subsequent interactions with the Authentication plugin.
         * @param adjusted_participant_key (out) The GUID_t that the implementation shall use to uniquely identify the
         * RTPSParticipant on the network.
         * @param domain_id The Domain Id of the RTPSParticipant.
         * @param participant_attr The RTPSParticipantAttributes of the RTPSParticipant.
         * @param candidate_participant_key The GUID_t that the DDS implementation would have used to uniquely identify
         * the RTPSParticipant if the Security plugins were not enabled.
         * @param exception (out) A SecurityException object.
         * @return Validation status.
         */
        virtual ValidationResult_t validate_local_identity(IdentityHandle** local_identity_handle,
                GUID_t& adjusted_participant_key,
                const uint32_t domain_id,
                const RTPSParticipantAttributes& participant_attr,
                const GUID_t& candidate_participant_key,
                SecurityException& exception) = 0;

        /*!
         * @brief Initiates the process of validating the identity of the discovered remote RTPSParticipant, represented
         * as an IdentityToken object.
         * @param remote_identity_handle (out) A handle that can be used to locally refer to the remote Authenticated
         * Participant in subsequent interactions with the AuthenticationPlugin.
         * @param local_identity_handle A handle to the local RTPSParticipant requesting the remote participant to be
         * validate.
         * @param remote_identity_token A token received as part of ParticipantProxyData, representing the
         * identity of the remote DomainParticipant.
         * @param remote_participant_key
         * @param exception (out) A SecurityException object.
         * @result Validation status.
         */
        virtual ValidationResult_t validate_remote_identity(IdentityHandle** remote_identity_handle,
                const IdentityHandle& local_identity_handle,
                IdentityToken&& remote_identity_token,
                const GUID_t& remote_participant_key,
                SecurityException& exception) = 0;

        /*!
         * @brief This operation is used to initiate a handshake.
         * @param handshake_handle (out) A handle returned by the Authentication plugin used to keep the state of the
         * handshake.
         * @param handshake_message_token (out) A HandshakeMessageToken to be sent using the BuiltinParticipantMessageWriter.
         * @param initiator_identity_handle Handle to the local participant that originated the handshake.
         * @param replier_identity_handle Handle to the remote participant whose identity is being validated.
         * @param exception (out) A SecurityException object.
         * @result Validation status.
         */
        virtual ValidationResult_t begin_handshake_request(HandshakeHandle** handshake_handle,
                HandshakeMessageToken** handshake_message,
                const IdentityHandle& initiator_identity_handle,
                IdentityHandle& replier_identity_handle,
                const CDRMessage_t& cdr_participant_data,
                SecurityException& exception) = 0;

        /*!
         * @brief This operation shall be invoked by the implementation in reaction to the reception of the initial
         * handshake message that originated on a RTPSParticipant that called the begin_handshake_request operation.
         * @param handshake_handle (out) A handle returned by the Authentication Plugin used to keep the state of the
         * handshake.
         * @param handshake_message_out (out) A HandshakeMessageToken containing a message to be sent using the
         * BuiltinParticipantMessageWriter.
         * @param handshake_message_in A HandshakeMessageToken containing a message received from the
         * BuiltinParticipantMessageReader.
         * @param initiator_identity_handle Handle to the remote participant that originated the handshake.
         * @param replier_identity_handle Handle to the local participant that is initiaing the handshake.
         * @param exception (out) A SecurityException object.
         * @result Validation status.
         */
        virtual ValidationResult_t begin_handshake_reply(HandshakeHandle** handshake_handle,
                HandshakeMessageToken** handshake_message_out,
                HandshakeMessageToken&& handshake_message_in,
                IdentityHandle& initiator_identity_handle,
                const IdentityHandle& replier_identity_handle,
                const CDRMessage_t& cdr_participant_data,
                SecurityException& exception) = 0;

        /*!
         * @brief This operation is used to continue a handshake.
         * @handshake_message_out (out) A HandshakeMessageToken containing the message_data that should be place in a
         * ParticipantStatelessMessage to be sent using the BuiltinParticipantMessageWriter.
         * @param handshake_message_in The HandshakeMessageToken contained in the message_data attribute of the
         * ParticipantStatelessMessage received.
         * @param handshake_handle Handle returned by a correspoing previous call to begin_handshake_request or
         * begin_handshake_reply.
         * @exception (out) A SecurityException object.
         * @return Validation status.
         */
        virtual ValidationResult_t process_handshake(HandshakeMessageToken** handshake_message_out,
                HandshakeMessageToken&& handshake_message_in,
                HandshakeHandle& handshake_handle,
                SecurityException& exception) = 0;

        /*!
         * @brief Retrieve the SharedSecretHandle resulting with a successfully completed handshake.
         * @param handshake_handle Handle returned bu a corresponding previous call to begin_handshake_request or
         * begin_handshake_reply, which has successfully complete the handshake operations.
         * @exception (out) A SecurityException object.
         * @return SharedSecretHandle.
         */
        virtual SharedSecretHandle* get_shared_secret(const HandshakeHandle& handshake_handle,
                SecurityException& exception) = 0;

        /*!
         * @brief Sets the AuthenticationListener that the Authentication plugin will use to notify the infrastructure
         * of events relevant to the Authentication of RTPSParticipants.
         * @param listener An AuthenticationListener object to be attached to the Authentication object.
         * @param exception (out) A SecurityException object.
         */
        virtual bool set_listener(AuthenticationListener* listener,
                SecurityException& exception) = 0;

        virtual bool get_identity_token(IdentityToken** identity_token,
                const IdentityHandle& handle,
                SecurityException& exception) = 0;

        /*!
         * @brief Returns the IdentityToken object to the plugin so it can be disposed of.
         * @param token An IdentityToken issued by the plugin on a prior call to get_identity_token.
         * @param exception (out) A SecurityException object.
         */
        virtual bool return_identity_token(IdentityToken* token,
                SecurityException& exception) = 0;

        /*!
         * @brief Returns the Handshakehandle object to the plugin so it can be disposed of.
         * @param handshake_handle A HandshakeHandle issued by the plugin on a prior call to begin_handshake_request or
         * begin_handshake_reply.
         * @param exception (out) A SecurityException object.
         */
        virtual bool return_handshake_handle(HandshakeHandle* handshake_handle,
                SecurityException& exception) = 0;

        /*!
         * @brief Returns the IdentityHandle object to the plugin so it can be disposed of.
         * @param identity_handle An IdentityHandle issued by the plugin on a prior call to validate_local_identity or
         * validate_remote_identity.
         * @param exception (out) A SecurityException object.
         */
        virtual bool return_identity_handle(IdentityHandle* identity_handle,
                SecurityException& exception) = 0;

        /*!
         * @brief Returns the SharedSecretHandle object to the plugin so it can be disposed of.
         * @param sharedsecret_handle An SharedSecretHandle issued by the plugin on a prior call to get_shared_secret.
         * @param exception (out) A SecurityException object.
         */
        virtual bool return_sharedsecret_handle(SharedSecretHandle* sharedsecret_handle,
                SecurityException& exception) = 0;

};

} //namespace security
} //namespace rtps
} //namespace fastrtps
} //namespace eprosima

#endif //  _RTPS_SECURITY_AUTHENTICATION_AUTHENTICATION_H_
