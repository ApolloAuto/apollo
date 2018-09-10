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
#ifndef _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOTRANSFORM_H_
#define _RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOTRANSFORM_H_

#include "CryptoTypes.h"
#include "../../common/CDRMessage_t.h"

namespace eprosima {
namespace fastrtps {
namespace rtps {
namespace security {

class CryptoTransform
{
    public:

        virtual ~CryptoTransform(){}
        /**
         * Serializes the payload sent by the user with a Datawriter.
         * @param encoded_buffer (out) Result of the encryption
         * @param extra_inline_qos (out) Contains additional parameters to be added to the inlineQos of the submessage
         * @param plain_buffer Plain input buffer
         * @param sending_datawriter_crypto Returned by a prior call to register_local_datawriter
         * @param exception (out) Security exception 
         * @return TRUE if successful
         */
        virtual bool encode_serialized_payload(
                std::vector<uint8_t> &encoded_buffer,
                std::vector<uint8_t> &extra_inline_qos,
                const std::vector<uint8_t> &plain_buffer,
                DatawriterCryptoHandle &sending_datawriter_crypto,
                SecurityException &exception) = 0;
        /**
         * Encodes a Data, DataFrag, Gap, Heartbeat or HeartBeatFrag
         * @param encoded_rtps_submessage (out) Result of the encryption
         * @param plain_rtps_submessage Plain input buffer
         * @param sending_datawriter_crypto Crypto of the datawriter that sends the message
         * @param receiving_datareader_crypto_list Crypto of the datareaders the message is aimed at
         * @param exception (out) Security exception
         * @return TRUE is successful
         */
        virtual bool encode_datawriter_submessage(
                std::vector<uint8_t> &encoded_rtps_submessage,
                const std::vector<uint8_t> &plain_rtps_submessage,
                DatawriterCryptoHandle &sending_datawriter_crypto,
                std::vector<DatareaderCryptoHandle*>& receiving_datareader_crypto_list,
                SecurityException &exception) = 0;

        /**
         * Encodes an AckNack or NackFrag
         * @param encoded_rtps_submessage (out) Result of the encryption
         * @param plain_rtps_submessage Plain input buffer
         * @param sending_datareader_crypto Crypto of the sending datareader
         * @param receiving_datawriter_crypto_list List with Crypto of the intended datawriter recipients
         * @param exception (out) Security exception
         * @return TRUE if successful
         */
        virtual bool encode_datareader_submessage(
                std::vector<uint8_t> &encoded_rtps_submessage,
                const std::vector<uint8_t> &plain_rtps_submessage,
                DatareaderCryptoHandle &sending_datareader_crypto,
                std::vector<DatawriterCryptoHandle*> &receiving_datawriter_crypto_list,
                SecurityException &exception) = 0;

        /**
         * Encodes a full rtps message
         * @param encoded_rtps_message (out) Result of the encryption
         * @param plain_rtps_message Plain input buffer
         * @param sending_crypto Crypto of the Participant where the message originates from
         * @param receiving_crypto_list Crypto of the Partipants the message is intended towards
         * @param exception (out) Security expcetion
         * @return TRUE if successful
         */
        virtual bool encode_rtps_message(
                std::vector<uint8_t> &encoded_rtps_message,
                const std::vector<uint8_t> &plain_rtps_message,
                ParticipantCryptoHandle &sending_crypto,
                const std::vector<ParticipantCryptoHandle*> &receiving_crypto_list,
                SecurityException &exception) = 0;

        /**
         * Reverses the transformation performed by encode_rtps_message. Decrypts the contents and veryfies MACs or digital signatures.
         * @param plain_buffer (out) Decoded message
         * @param encoded_buffer Encoded message
         * @param receiving_crypto Crypto of the Participant that receives the message
         * @param sending_crypto Crypto of the Participant that wrote the message
         * @param exception (out) Security exception
         * @return TRUE is successful
         */
        virtual bool decode_rtps_message(
                std::vector<uint8_t> &plain_buffer,
                const std::vector<uint8_t> &encoded_buffer,
                const ParticipantCryptoHandle &receiving_crypto,
                const ParticipantCryptoHandle &sending_crypto,
                SecurityException &exception) = 0;

        /**
         * Determines whether the secure submessage comes from a datawriter or a data reader and extracts the required CryptoHandle to decode it.
         * @param datawriter_crypt (out) Crypto of the sending datawriter, if applicable
         * @param datareader_crypto (out) Crypto of the sending datareader, if applicable
         * @param secure_submessage_category (out) Specifies wether the message comes from a datawriter or from a datareader
         * @param encoded_rtps_submessage encoded input submessage
         * @param receiving_crypto Crypto of the Participant that receives the message
         * @param sending_crypto Crypto of the Participant that sent the message
         * @param exception (out) Security exception
         * @return TRUE if successful
         */
        virtual bool preprocess_secure_submsg(
                DatawriterCryptoHandle **datawriter_crypto,
                DatareaderCryptoHandle **datareader_crypto,
                SecureSubmessageCategory_t &secure_submessage_category,
                const CDRMessage_t& encoded_rtps_submessage,
                ParticipantCryptoHandle &receiving_crypto,
                ParticipantCryptoHandle &sending_crypto,
                SecurityException &exception) = 0;

        /**
         * Called after prprocess_secure_submessage when the submessage category is DATAWRITER_SUBMESSAGE
         * @param plain_rtps_message (out) Result of the decryption
         * @param encoded_rtps_submessage Encoded message
         * @param receiving_datareader_crypto Crypto of the target datareader
         * @param sending_datawriter_crypto Crypto of the datawriter that sent the message
         * @param exception (out) Security exception
         * @return TRUE if successful
         */
        virtual bool decode_datawriter_submessage(
                CDRMessage_t& plain_rtps_submessage,
                CDRMessage_t& encoded_rtps_submessage,
                DatareaderCryptoHandle &receiving_datareader_crypto,
                DatawriterCryptoHandle &sending_datawriter_crypto,
                SecurityException &exception) = 0;

        /**
         * Called after preprocess_secure_submessage when the submessage category is DATAREADER_SUBMESSAGE
         * @param plain_rtps_submessage (out) Result of the decryption
         * @param encoded_rtps_submessage Encoded message
         * @param receiving_datawriter_crypto Crypto of the target datawriter
         * @param sending_datareader_crypto Crypto of the datareader that sent the message
         * @param exception (out) Security exception
         * @return TRUE if successful
         */
        virtual bool decode_datareader_submessage(
                CDRMessage_t& plain_rtps_submessage,
                CDRMessage_t& encoded_rtps_submessage,
                DatawriterCryptoHandle &receiving_datawriter_crypto,
                DatareaderCryptoHandle &sending_datareader_crypto,
                SecurityException &exception) = 0;

        /**
         * Undoes the decryption transformation made on the writer side.
         * @param plain_buffer (out) Result of the decryption
         * @param encoded_buffer Encoded input buffer
         * @param inline_qos Coming from the data message that carries the target payload 
         * @param receiving_datareader_crypto Crypto of the target datareader
         * @param sending_datawriter_crypto Crypto of the datawriter that sent the message
         * @param exception (out) Security exception
         * @return TRUE if successful
         */
        virtual bool decode_serialized_payload(
                std::vector<uint8_t> &plain_buffer,
                const std::vector<uint8_t> &encoded_buffer,
                const std::vector<uint8_t> &inline_qos,
                DatareaderCryptoHandle &receiving_datareader_crypto,
                DatawriterCryptoHandle &sending_datawriter_crypto,
                SecurityException &exception) = 0;

        virtual uint32_t calculate_extra_size_for_rtps_message(uint32_t number_discovered_participants) const = 0;

        virtual uint32_t calculate_extra_size_for_rtps_submessage(uint32_t number_discovered_readers) const = 0;

        virtual uint32_t calculate_extra_size_for_encoded_payload(uint32_t number_discovered_readers) const = 0;
};

} //namespace eprosima
} //namespace fastrtps
} //namespace rtps
} //namespace security

#endif //_RTPS_SECURITY_CRYPTOGRAPHY_CRYPTOTRANSFORM_H_
