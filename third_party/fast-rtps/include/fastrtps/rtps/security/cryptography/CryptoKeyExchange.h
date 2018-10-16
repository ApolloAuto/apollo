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
#ifndef _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYEXCHANGE_H_
#define _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYEXCHANGE_H_

#include "CryptoTypes.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

class CryptoKeyExchange
{
    public:

    virtual ~CryptoKeyExchange(){}
    /**
     * Creates Crypto Tokens containing the info to decrypt text encoded by the local Participant. 
     * To be sent to the remote participant.
     * @param local_participant_crypto_tokens (out) Returned CryptoTokenSeq.
     * @param local_participant_crypto CryptoHandle returned by a previous call to register_local_participant.
     * @param remote_participant_crypto CryptoHangle returned by a previous call to register_remote_participant.
     * @param exception (out) Security exception
     * @return TRUE is successful.
     */
    virtual bool create_local_participant_crypto_tokens(
            ParticipantCryptoTokenSeq& local_participant_crypto_tokens,
            const ParticipantCryptoHandle& local_participant_crypto,
            ParticipantCryptoHandle& remote_participant_crypto,
            SecurityException& exception) = 0;

    /**
     * Configures the Cryptographic Plugin with the material needed to interpret messages coming from the remote crypto.
     * @param local_participant_crypto CryptoHandle returned by a previous call to register_local_participant.
     * @param remote_participant_crypto CryptoHandle returned by a previous call to register_matched_remote_participant.
     * @param remote_participant_tokens CryptoToken sequence received from the remote Participant
     * @param exception (out) Security exception 
     * @return TRUE if successful
     */
    virtual bool set_remote_participant_crypto_tokens(
            const ParticipantCryptoHandle &local_participant_crypto,
            ParticipantCryptoHandle &remote_participant_crypto,
            const ParticipantCryptoTokenSeq &remote_participant_tokens,
            SecurityException &exception) = 0;

    /**
     * Creates CryptoTokens containing the info to decrypt text encoded by the local DataWriter.
     * @param local_datawriter_crypto_tokens (out) Returned CryptoSeq
     * @param local_datawriter_crypto CryptoHandle returned by a previous call to register_local_datawriter.
     * @param remote_datawriter_crypto CryptoHandle returned by a previous call to register_matched_remote_datareader
     * @param exception (out) Security exception
     * @return TRUE if successful
     */
    virtual bool create_local_datawriter_crypto_tokens(
            DatawriterCryptoTokenSeq &local_datawriter_crypto_tokens,
            DatawriterCryptoHandle &local_datawriter_crypto,
            DatareaderCryptoHandle &remote_datareader_crypto,
            SecurityException &exception) = 0;

    /**
     * Creates CryptoTokens containing the info to decrypt text encoded by the local DataReader.
     * @param local_datareader_crypto_tokens (out)
     * @param local_datareader_crypto
     * @param remote_datawriter_crypto
     * @param exception (out) Security exception
     * @return TRUE if successful
     */
    virtual bool create_local_datareader_crypto_tokens(
            DatareaderCryptoTokenSeq &local_datareader_crypto_tokens,
            DatareaderCryptoHandle &local_datareader_crypto,
            DatawriterCryptoHandle &remote_datawriter_crypto,
            SecurityException &exception) = 0;

    /**
     * Configures the Cryptographic Plugin with the material needed to interpret messages coming from the remote DataReader.
     * @param local_datawriter_crypto
     * @param remote_datareader_crypto
     * @param remote_datareader_tokens
     * @param exception (out) Security exception
     * @return TRUE if successful
     */
    virtual bool set_remote_datareader_crypto_tokens(
            DatawriterCryptoHandle &local_datawriter_crypto,
            DatareaderCryptoHandle &remote_datareader_crypto,
            const DatareaderCryptoTokenSeq &remote_datareader_tokens,
            SecurityException &exception) = 0;

    /**
     * Configures the Cryptographic Plugin with the material needed to interpret messages coming from the remote DataWriter.
     * @param local_datareader_crypto
     * @param remote_datawriter_crypto
     * @param remote_datawriter_tokens
     * @param exception (out) Security exception
     * @return TRUE if successful
     */

    virtual bool set_remote_datawriter_crypto_tokens(
             DatareaderCryptoHandle &local_datareader_crypto,
             DatawriterCryptoHandle &remote_datawriter_crypto,
             const DatawriterCryptoTokenSeq &remote_datawriter_tokens,
             SecurityException &exception) = 0;

    /**
     * Release resources associated with a CryptoTokenSeq
     * @param crypto_tokens
     * @param exception (out) Security exception
     * @return TRUE if successful
     */
    virtual bool return_crypto_tokens(
            const CryptoTokenSeq &crypto_tokens,
            SecurityException &exception) = 0;

};

} //namespace eprosima
} //namespace fastrtps
} //namespace rtps
} //namespace security

#endif //_RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOKEYEXCHANGE_H_
