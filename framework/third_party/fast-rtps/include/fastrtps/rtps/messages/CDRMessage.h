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
 * @file CDRMessage.h
 */



#ifndef CDRMESSAGE_H_
#define CDRMESSAGE_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
#include "../common/CDRMessage_t.h"
#include "../../qos/ParameterTypes.h"
#include "../common/Property.h"
#include "../common/BinaryProperty.h"

#include "../security/common/ParticipantGenericMessage.h"

using namespace eprosima::fastrtps;

namespace eprosima {
namespace fastrtps{
namespace rtps {

/**
 * Namespace CDRMessage, contains inline methods to initialize CDRMessage_t and add or read different data types.
 @ingroup COMMON_MODULE
 */
namespace CDRMessage{

	/** @name Read from a CDRMessage_t.
	 * Methods to read different data types from a CDR message. Pointers to the message and to the data types are provided.
	 * The read position is updated in the message. It fails if you attempt to read outside the
	 * boundaries of the message.
	 * @param[in] msg Pointer to message.
	 * @param[out] data_ptr Pointer to data.
	 * @param[in] size Number of bytes (if necessary).
	 * @return True if correct.
	 */
	/// @{
	  inline bool readEntityId(CDRMessage_t* msg,const EntityId_t*id);
	  inline bool readData(CDRMessage_t* msg, octet* o, uint32_t length);
	  inline bool readDataReversed(CDRMessage_t* msg, octet* o, uint32_t length);
	  inline bool readInt32(CDRMessage_t* msg,int32_t* lo);
	  inline bool readUInt32(CDRMessage_t* msg, uint32_t* ulo);
	  inline bool readInt64(CDRMessage_t* msg, int64_t* lolo);
	  inline bool readSequenceNumber(CDRMessage_t* msg, SequenceNumber_t* sn);
	  inline bool readInt16(CDRMessage_t* msg,int16_t* i16);
	  inline bool readUInt16(CDRMessage_t* msg,uint16_t* i16);
	  inline bool readLocator(CDRMessage_t* msg,Locator_t* loc);
	  inline bool readOctet(CDRMessage_t* msg, octet* o);
	  inline bool readSequenceNumberSet(CDRMessage_t* msg, SequenceNumberSet_t* snset);
	  inline bool readFragmentNumberSet(CDRMessage_t* msg, FragmentNumberSet_t* snset);
	  inline bool readTimestamp(CDRMessage_t*msg,Time_t* ts);
	  inline bool readString(CDRMessage_t*msg,std::string* p_str);
	  inline bool readOctetVector(CDRMessage_t*msg,std::vector<octet>* ocvec);

      inline bool readProperty(CDRMessage_t* msg, Property& property);
      inline bool readBinaryProperty(CDRMessage_t* msg, BinaryProperty& binary_property);
      inline bool readPropertySeq(CDRMessage_t* msg, PropertySeq& properties);
      inline bool readBinaryPropertySeq(CDRMessage_t* msg, BinaryPropertySeq& binary_properties);
      inline bool readDataHolder(CDRMessage_t* msg, DataHolder& data_holder);
      inline bool readDataHolderSeq(CDRMessage_t* msg, DataHolderSeq& data_holders);
      inline bool readMessageIdentity(CDRMessage_t* msg, ::security::MessageIdentity& message_identity);
      inline bool readParticipantGenericMessage(CDRMessage_t* msg, ::security::ParticipantGenericMessage& message);
	///@}


	/**
	 * Initialize given CDR message with default size. It frees the memory already allocated and reserves new one.
	 * @param[in,out] msg Pointer to the message to initialize.
	 * @param data_size Size of the data the message is suppose to carry
	 * @return True if correct.
	 */
	  inline bool initCDRMsg(CDRMessage_t* msg,uint32_t data_size=RTPSMESSAGE_COMMON_DATA_PAYLOAD_SIZE);

     inline bool wrapVector(CDRMessage_t* msg, std::vector<octet>& vectorToWrap);
	/**
	 * Append given CDRMessage to existing CDR Message. Joins two messages into the first one if it has space.
	 * @param[out] first Pointer to first message.
	 * @param[in] second Pointer to second message.
	 ** @return True if correct.
	 */
	  inline bool appendMsg(CDRMessage_t* first,CDRMessage_t* second);


	/** @name Add to a CDRMessage_t.
	 * Methods to add different data types to a CDR message. Pointers to the message and to the data types are provided.
	 * The write position is updated in the message. It fails if you attempt to write outside the
	 * boundaries of the message.
	 * @param[in,out] Pointer to message.
	 * @param[in] data Data to add (might be a pointer).
	 * @param[in] byteSize Number of bytes (if necessary).
	 * @return True if correct.
	 */
	/// @{

      inline bool addData(CDRMessage_t*, const octet*, const uint32_t number_bytes);
      inline bool addDataReversed(CDRMessage_t*, const octet*, const uint32_t byte_number);
	  inline bool addOctet(CDRMessage_t*msg,octet o);
	  inline bool addUInt16(CDRMessage_t*msg,uint16_t us);
	  inline bool addInt32(CDRMessage_t*msg,int32_t lo);
	  inline bool addUInt32(CDRMessage_t*msg,uint32_t lo);
	  inline bool addInt64(CDRMessage_t*msg,int64_t lo);
	  inline bool addEntityId(CDRMessage_t*msg,const EntityId_t* id);
	  inline bool addSequenceNumber(CDRMessage_t*msg, const SequenceNumber_t* sn);
	  inline bool addSequenceNumberSet(CDRMessage_t*msg, const SequenceNumberSet_t* sns);
	  inline bool addFragmentNumberSet(CDRMessage_t*msg, FragmentNumberSet_t* fns);
	  inline bool addLocator(CDRMessage_t*msg,Locator_t*loc);
	  inline bool addParameterStatus(CDRMessage_t*msg,octet status);
	  inline bool addParameterKey(CDRMessage_t*msg, const InstanceHandle_t* iHandle);
	  inline bool addParameterSentinel(CDRMessage_t*msg);
	  inline bool addParameterId(CDRMessage_t*msg,ParameterId_t pid);
	  inline bool addString(CDRMessage_t*msg, const std::string& in_str);
	  inline bool addOctetVector(CDRMessage_t*msg, const std::vector<octet>* ocvec);
      inline bool addParameterSampleIdentity(CDRMessage_t *msg, const SampleIdentity &sample_id);
	///@}
    
      inline bool addProperty(CDRMessage_t* msg, const Property& property);
      inline bool addBinaryProperty(CDRMessage_t* msg, const BinaryProperty& binary_property);
      inline bool addPropertySeq(CDRMessage_t* msg, const PropertySeq& properties);
      inline bool addBinaryPropertySeq(CDRMessage_t* msg, const BinaryPropertySeq& binary_properties);
      inline bool addBinaryPropertySeq(CDRMessage_t* msg, const BinaryPropertySeq& binary_properties, const std::string& property_limit);
      inline bool addDataHolder(CDRMessage_t* msg, const DataHolder& data_holder);
      inline bool addDataHolderSeq(CDRMessage_t* msg, const DataHolderSeq& data_holders);
      inline bool addMessageIdentity(CDRMessage_t* msg, const ::security::MessageIdentity& message_identity);
      inline bool addParticipantGenericMessage(CDRMessage_t* msg, const ::security::ParticipantGenericMessage& message);
}

}
} /* namespace rtps */
} /* namespace eprosima */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "CDRMessage.hpp"

#endif /* DOXYGEN_SHOULD_SKIP_THIS */
#endif
#endif /* CDRMESSAGE_H_ */
