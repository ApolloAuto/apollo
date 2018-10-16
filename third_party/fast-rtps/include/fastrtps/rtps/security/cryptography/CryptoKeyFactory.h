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
 * @file CryptoKeyFactory.h
 */
#ifndef _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYFACTORY_H_
#define _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYFACTORY_H_

#include "CryptoTypes.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

class CryptoKeyFactory
{
    public:

        virtual ~CryptoKeyFactory(){}
        /**
         * Register a local, already authenticated Participant with the Cryptographic Plugin.
         * Creates Crypto material needed to encrypt messages directed to other Participants
         * @param participant_identity Made by a prior call to validate_local_identity
         * @param participant_permissions Made by a prior call to validate_local_permissions
         * @param participant_properties Combination of PropertyQoSPolicy and contents of AccessControl
         * @param exception (out) Security exception
         * @return ParticipantCryptoHandle with generated key material  
         */
        virtual ParticipantCryptoHandle * register_local_participant(
                const IdentityHandle &participant_identity, 
                const PermissionsHandle &participant_permissions, 
                const PropertySeq &participant_properties, 
                SecurityException &exception) = 0;

        /**
         * Register a remote, already authenticated Participant with the Cryptographic Plugin.
         * Creates key material to decrypt messages coming from and aimed at it.
         * @param local_participant_crypto_handle Returned by a prior call to register_local_participant
         * @param remote_participant_identity Returned by a prior call to validate_remote_identity
         * @param remote_participant_permissions Returned by a prior call to validate_remote_permissions
         * @param shared_secret Returned by a prior call to get_shared_secret (Auth Handshake)
         * @param exception (out) Security exception
         * @return ParticipantCryptoHandle with generated key material
         */
        virtual ParticipantCryptoHandle * register_matched_remote_participant(
                const ParticipantCryptoHandle& local_participant_crypto_handle,
                const IdentityHandle& remote_participant_identity,
                const PermissionsHandle& remote_participant_permissions,
                const SharedSecretHandle& shared_secret,
                SecurityException &exception) = 0;

        /**
         * Register a local DataWriter belonging to an authenticated Pariticipant.
         * Creates cryptomaterial for use with incoming/outgoing messages
         * @param participant_crypto returned by a prior call to register_local_participant
         * @param datawriter_prop Combination of PropertyWosPolicy and contents of AccessControl
         * @param exception (out) Security exception 
         * @return CryptoHandle to be used with operations related to the DataWriter
         */
        virtual DatawriterCryptoHandle * register_local_datawriter(
                ParticipantCryptoHandle &participant_crypto,
                const PropertySeq &datawriter_prop,
                SecurityException &exception) = 0;

        /**
         * Register a remote DataReader that has been granted permission to match with the local DataWriter. 
         * Creates cryptographic material to encript/decrypt messages from and towards that DataReader.
         * @param local_datawriter_crypto_handle Returned by a prior call to register_local_datawriter
         * @param remote_participant_crypto Returned by a prior call to register_matched_remote_participant.
         * @param shared_secret Obtained as a result of the Authentication Handshake.
         * @param relay_only If FALSE it generates material for both a submessage and serialized payload. Submessages only if TRUE.
         * @param exception (out) Security exception.
         * @return Crypto Handle to the generated key material.
         */
        virtual DatareaderCryptoHandle * register_matched_remote_datareader(
                DatawriterCryptoHandle &local_datawriter_crypto_handle,
                ParticipantCryptoHandle &remote_participant_crypto,
                const SharedSecretHandle &shared_secret,
                const bool relay_only,
                SecurityException &exception) = 0;

        /**
         * Register a local DataReader (belonging to an authenticated and authorized Participant) with the Cryptographic Plugin.
         * Creates crypto material to encode messages when the encryption is independent of the targeted DataWriter
         * @param participant_crypto Returned by a prior call to register_local_participant
         * @param datareader_properties Combination of PropertyQosPolicy and the contents of AccessControl
         * @param exception (out) Security exception
         * @return Crypto Handle to the generated key material
         */
        virtual DatareaderCryptoHandle * register_local_datareader(
                ParticipantCryptoHandle &participant_crypto,
                const PropertySeq &datareader_properties,
                SecurityException &exception) = 0;

        /**
         * Register a remote DataWriter that has been granted permission to match with a local DataReader.
         * Creates crypto material to decrypt messages coming from and encode messages going towards that datareader
         * @param local_datareader_crypto_handle
         * @param remote_participant_crypt
         * @param shared_secret
         * @param exception (out) Security exception
         * @return Crypto handle to the generated key material
         */
        virtual DatawriterCryptoHandle * register_matched_remote_datawriter(
                DatareaderCryptoHandle &local_datareader_crypto_handle,
                ParticipantCryptoHandle &remote_participant_crypt,
                const SharedSecretHandle &shared_secret,
                SecurityException &exception) = 0;

        /**
         * Releases resources associated with a Participant. The Crypto Handle becomes unusable after this 
         * @param participant_crypto_handle Belonging to the Participant that awaits termination
         * @param exception (out) Security exception
         * @return TRUE is succesful 
         */
        virtual bool unregister_participant(
                ParticipantCryptoHandle* participant_crypto_handle,
                SecurityException &exception) = 0;

        /**
         * Releases resources associated with a DataWriter. The Crypto Handle becomes unusable after this 
         * @param datawriter_crypto_handle Belonging to the DataWriter that awaits termination
         * @param exception (out) Security exception
         * @return TRUE is succesful 
         */
        virtual bool unregister_datawriter(
                DatawriterCryptoHandle *datawriter_crypto_handle,
                SecurityException &exception) = 0;

        /**
         * Releases resources associated with a DataReader. The Crypto Handle becomes unusable after this 
         * @param datareader_crypto_handle Belonging to the DataReader that awaits termination
         * @param exception (out) Security exception
         * @return TRUE is succesful 
         */
        virtual bool unregister_datareader(
                DatareaderCryptoHandle *datareader_crypto_handle,
                SecurityException &exception) = 0;


};

} //namespace security
} //namespace rtps
} //namespace fastrtps
} //namespace eprosima

#endif //_RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYFACTORY_H_
