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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/**
 * @file CDRMessage.hpp
 *
 */

#include <cassert>
#include <algorithm>
#include <vector>

namespace eprosima {
namespace fastrtps{
namespace rtps {

inline bool CDRMessage::initCDRMsg(CDRMessage_t*msg,uint32_t payload_size)
{
    if(msg->buffer==NULL)
    {
        msg->buffer = (octet*)malloc(payload_size+RTPSMESSAGE_COMMON_RTPS_PAYLOAD_SIZE);
        msg->max_size = payload_size+RTPSMESSAGE_COMMON_RTPS_PAYLOAD_SIZE;
    }
    msg->pos = 0;
    msg->length = 0;
#if EPROSIMA_BIG_ENDIAN
    msg->msg_endian = BIGEND;
#else
    msg->msg_endian = LITTLEEND;
#endif
    return true;
}

inline bool CDRMessage::wrapVector(CDRMessage_t* msg, std::vector<octet>& vectorToWrap)
{
    if (msg->buffer && !msg->wraps)
        free(msg->buffer);

    msg->wraps = true;
    msg->buffer = vectorToWrap.data();
    msg->length = (uint32_t)vectorToWrap.size();
    msg->max_size = (uint32_t)vectorToWrap.capacity();
#if EPROSIMA_BIG_ENDIAN
    msg->msg_endian = BIGEND;
#else
    msg->msg_endian = LITTLEEND;
#endif
    return true;
}

inline bool CDRMessage::appendMsg(CDRMessage_t*first, CDRMessage_t*second) {
    return(CDRMessage::addData(first,second->buffer,second->length));
}


inline bool CDRMessage::readEntityId(CDRMessage_t* msg,const EntityId_t* id) {
    if(msg->pos+4>msg->length)
        return false;
    uint32_t* aux1 = (uint32_t*) id->value;
    uint32_t* aux2 = (uint32_t*) &msg->buffer[msg->pos];
    *aux1 = *aux2;
    msg->pos+=4;
    return true;
}

inline bool CDRMessage::readData(CDRMessage_t* msg, octet* o, uint32_t length) {
    if(msg->pos+length > msg->length){
        return false;
    }
    memcpy(o,&msg->buffer[msg->pos],length);
    msg->pos+=length;
    return true;
}

inline bool CDRMessage::readDataReversed(CDRMessage_t* msg, octet* o, uint32_t length) {
    for(uint16_t i=0;i<length;i++)
    {
        *(o+i)=*(msg->buffer+msg->pos+length-1-i);
    }
    msg->pos+=length;
    return true;
}

inline bool CDRMessage::readInt32(CDRMessage_t* msg,int32_t* lo) {
    if(msg->pos+4>msg->length)
        return false;
    octet* dest = (octet*)lo;
    if(msg->msg_endian == DEFAULT_ENDIAN){
        for(uint8_t i =0;i<4;i++)
            dest[i] = msg->buffer[msg->pos+i];
        msg->pos+=4;
    }
    else{
        readDataReversed(msg,dest,4);
    }
    return true;
}

inline bool CDRMessage::readUInt32(CDRMessage_t* msg,uint32_t* ulo) {
    if(msg->pos+4>msg->length)
        return false;
    octet* dest = (octet*)ulo;
    if(msg->msg_endian == DEFAULT_ENDIAN){
        for(uint8_t i =0;i<4;i++)
            dest[i] = msg->buffer[msg->pos+i];
        msg->pos+=4;
    }
    else{
        readDataReversed(msg,dest,4);
    }
    return true;
}

inline bool CDRMessage::readInt64(CDRMessage_t* msg, int64_t* lolo)
{
    if(msg->pos+8 > msg->length)
        return false;

    octet* dest = (octet*)lolo;
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        for(uint8_t i = 0; i < 8; ++i)
            dest[i] = msg->buffer[msg->pos+i];

        msg->pos+=8;
    }
    else
    {
        readDataReversed(msg, dest, 8);
    }

    return true;
}

inline bool CDRMessage::readSequenceNumber(CDRMessage_t* msg,SequenceNumber_t* sn) {
    if(msg->pos+8>msg->length)
        return false;
    bool valid=readInt32(msg,&sn->high);
    valid&=readUInt32(msg,&sn->low);
    return true;
}

inline bool CDRMessage::readSequenceNumberSet(CDRMessage_t* msg,SequenceNumberSet_t* sns)
{
    bool valid = true;
    valid &=CDRMessage::readSequenceNumber(msg,&sns->base);
    uint32_t numBits;
    valid &=CDRMessage::readUInt32(msg,&numBits);
    int32_t bitmap;
    SequenceNumber_t seqNum;
    for(uint32_t i=0;i<(numBits+31)/32;++i)
    {
        valid &= CDRMessage::readInt32(msg,&bitmap);
        for(uint8_t bit=0;bit<32;++bit)
        {
            if((bitmap & (1<<(31-bit%32)))==(1<<(31-bit%32)))
            {
                seqNum = sns->base+(i*32+bit);
                if(!sns->add(seqNum))
                {
                    return false;
                }
            }
        }
    }
    return valid;
}

inline bool CDRMessage::readFragmentNumberSet(CDRMessage_t* msg, FragmentNumberSet_t* fns)
{
    bool valid = true;
    valid &= CDRMessage::readUInt32(msg, &fns->base);
    uint32_t numBits;
    valid &= CDRMessage::readUInt32(msg, &numBits);
    int32_t bitmap;
    FragmentNumber_t fragNum;
    for (uint32_t i = 0; i<(numBits + 31) / 32; ++i)
    {
        valid &= CDRMessage::readInt32(msg, &bitmap);
        for (uint8_t bit = 0; bit<32; ++bit)
        {
            if ((bitmap & (1 << (31 - bit % 32))) == (1 << (31 - bit % 32)))
            {
                fragNum = fns->base + (i * 32 + bit);
                if (!fns->add(fragNum))
                {
                    return false;
                }
            }
        }
    }
    return valid;
}

inline bool CDRMessage::readTimestamp(CDRMessage_t* msg, Time_t* ts)
{
    bool valid = true;
    valid &=CDRMessage::readInt32(msg,&ts->seconds);
    valid &=CDRMessage::readUInt32(msg,&ts->fraction);
    return valid;
}


inline bool CDRMessage::readLocator(CDRMessage_t* msg,Locator_t* loc)
{
    if(msg->pos+24>msg->length)
        return false;
    bool valid = readInt32(msg,&loc->kind);
    valid&=readUInt32(msg,&loc->port);

    valid&=readData(msg,loc->address,16);

    return valid;
}

inline bool CDRMessage::readInt16(CDRMessage_t* msg,int16_t* i16)
{
    if(msg->pos+2>msg->length)
        return false;
    octet* o = (octet*)i16;
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        *o = msg->buffer[msg->pos];
        *(o+1) = msg->buffer[msg->pos+1];
    }
    else{
        *o = msg->buffer[msg->pos+1];
        *(o+1) = msg->buffer[msg->pos];
    }
    msg->pos+=2;
    return true;
}

inline bool CDRMessage::readUInt16(CDRMessage_t* msg,uint16_t* i16)
{
    if(msg->pos+2>msg->length)
        return false;
    octet* o = (octet*)i16;
    if(msg->msg_endian == DEFAULT_ENDIAN){
        *o = msg->buffer[msg->pos];
        *(o+1) = msg->buffer[msg->pos+1];
    }
    else{
        *o = msg->buffer[msg->pos+1];
        *(o+1) = msg->buffer[msg->pos];
    }
    msg->pos+=2;
    return true;
}



inline bool CDRMessage::readOctet(CDRMessage_t* msg, octet* o) {
    if(msg->pos+1>msg->length)
        return false;
    *o = msg->buffer[msg->pos];
    msg->pos++;
    return true;
}

inline bool CDRMessage::readOctetVector(CDRMessage_t*msg,std::vector<octet>* ocvec)
{
    if(msg->pos+4>msg->length)
        return false;
    uint32_t vecsize;
    bool valid = CDRMessage::readUInt32(msg,&vecsize);
    ocvec->resize(vecsize);
    valid &= CDRMessage::readData(msg,ocvec->data(),vecsize);
    int rest = (vecsize) % 4;
    rest = rest== 0 ? 0 : 4-rest;
    msg->pos+=rest;
    return valid;
}


inline bool CDRMessage::readString(CDRMessage_t*msg, std::string* stri)
{
    uint32_t str_size = 1;
    bool valid = true;
    valid&=CDRMessage::readUInt32(msg,&str_size);
    if(msg->pos+str_size > msg->length){
        return false;
    }
    if(str_size>1)
    {
        *stri = std::string();stri->resize(str_size-1);
        octet* oc1 = (octet*)malloc(str_size);
        valid &= CDRMessage::readData(msg,oc1,str_size);
        for(uint32_t i =0;i<str_size-1;i++)
            stri->at(i) = oc1[i];
        free((void*)oc1);
    }
    else
    {
        msg->pos+=str_size;
    }
    int rest = (str_size) % 4;
    rest = rest==0 ? 0 : 4-rest;
    msg->pos+=rest;

    return valid;
}


inline bool CDRMessage::addData(CDRMessage_t*msg, const octet *data, const uint32_t length)
{
    if(msg->pos + length > msg->max_size)
    {
        return false;
    }

    memcpy(&msg->buffer[msg->pos],data,length);
    msg->pos +=length;
    msg->length+=length;
    return true;
}

inline bool CDRMessage::addDataReversed(CDRMessage_t*msg, const octet *data, const uint32_t length)
{
    if(msg->pos + length > msg->max_size)
    {
        return false;
    }
    for(uint32_t i = 0;i<length;i++)
    {
        msg->buffer[msg->pos+i] = *(data+length-1-i);
    }
    msg->pos +=length;
    msg->length+=length;
    return true;
}

inline bool CDRMessage::addOctet(CDRMessage_t*msg, octet O)
{
    if(msg->pos + 1 > msg->max_size)
    {
        return false;
    }
    //const void* d = (void*)&O;
    msg->buffer[msg->pos] = O;
    msg->pos++;
    msg->length++;
    return true;
}

inline bool CDRMessage::addUInt16(CDRMessage_t*msg,uint16_t us)
{
    if(msg->pos + 2 > msg->max_size)
    {
        return false;
    }
    octet* o= (octet*)&us;
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        msg->buffer[msg->pos] = *(o);
        msg->buffer[msg->pos+1] = *(o+1);
    }
    else
    {
        msg->buffer[msg->pos] = *(o+1);
        msg->buffer[msg->pos+1] = *(o);
    }
    msg->pos+=2;
    msg->length+=2;
    return true;
}


inline bool CDRMessage::addParameterId(CDRMessage_t* msg, ParameterId_t pid)
{
    return CDRMessage::addUInt16(msg,(uint16_t)pid);
}



inline bool CDRMessage::addInt32(CDRMessage_t* msg, int32_t lo) {
    octet* o= (octet*)&lo;
    if(msg->pos + 4 > msg->max_size)
    {
        return false;
    }
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        for(uint8_t i=0;i<4;i++)
        {
            msg->buffer[msg->pos+i] = *(o+i);
        }
    }
    else
    {
        for(uint8_t i=0;i<4;i++)
        {
            msg->buffer[msg->pos+i] = *(o+3-i);
        }
    }
    msg->pos+=4;
    msg->length+=4;
    return true;
}



inline bool CDRMessage::addUInt32(CDRMessage_t* msg, uint32_t ulo) {
    octet* o= (octet*)&ulo;
    if(msg->pos + 4 > msg->max_size)
    {
        return false;
    }
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        for(uint8_t i=0;i<4;i++)
        {
            msg->buffer[msg->pos+i] = *(o+i);
        }
    }
    else
    {
        for(uint8_t i=0;i<4;i++)
        {
            msg->buffer[msg->pos+i] = *(o+3-i);
        }
    }
    msg->pos+=4;
    msg->length+=4;
    return true;
}

inline bool CDRMessage::addInt64(CDRMessage_t* msg, int64_t lolo) {
    octet* o= (octet*)&lolo;
    if(msg->pos + 8 > msg->max_size)
    {
        return false;
    }
    if(msg->msg_endian == DEFAULT_ENDIAN)
    {
        for(uint8_t i=0;i<8;i++)
        {
            msg->buffer[msg->pos+i] = *(o+i);
        }
    }
    else
    {
        for(uint8_t i=0;i<8;i++)
        {
            msg->buffer[msg->pos+i] = *(o+7-i);
        }
    }
    msg->pos+=8;
    msg->length+=8;
    return true;
}

inline bool CDRMessage::addOctetVector(CDRMessage_t*msg, const std::vector<octet>* ocvec)
{
    // TODO Calculate without padding
    if(msg->pos+4+ocvec->size()>=msg->max_size)
    {
        return false;
    }
    bool valid = CDRMessage::addUInt32(msg,(uint32_t)ocvec->size());
    valid &= CDRMessage::addData(msg,(octet*)ocvec->data(),(uint32_t)ocvec->size());

    int rest = ocvec->size()% 4;
    if (rest != 0)
    {
        rest = 4 - rest; //how many you have to add

        octet oc = '\0';
        for (int i = 0; i < rest; i++) {
            valid &= CDRMessage::addOctet(msg, oc);
        }
    }

    return valid;
}


inline bool CDRMessage::addEntityId(CDRMessage_t* msg, const EntityId_t*ID) {
    if(msg->pos+4>=msg->max_size)
    {
        return false;
    }
    int* aux1;
    int* aux2;
    aux1 = (int*)(&msg->buffer[msg->pos]);
    aux2 = (int*) ID->value;
    *aux1 = *aux2;
    msg->pos +=4;
    msg->length+=4;
    return true;
}





inline bool CDRMessage::addSequenceNumber(CDRMessage_t* msg,
        const SequenceNumber_t* sn)
{
    addInt32(msg,sn->high);
    addUInt32(msg,sn->low);

    return true;
}

inline bool CDRMessage::addSequenceNumberSet(CDRMessage_t* msg,
        const SequenceNumberSet_t* sns)
{
    CDRMessage::addSequenceNumber(msg, &sns->base);

    //Add set
    if(sns->isSetEmpty())
    {
        addUInt32(msg,0); //numbits 0
        return true;
    }

    SequenceNumber_t maxseqNum = sns->get_maxSeqNum();

    uint32_t numBits = (maxseqNum - sns->base + 1).low;
    assert((maxseqNum - sns->base + 1).high == 0);

    if(numBits >= 256)
        numBits = 255;

    addUInt32(msg, numBits);
    uint8_t n_longs = (uint8_t)((numBits + 31) / 32);
    int32_t* bitmap = new int32_t[n_longs];

    for(uint32_t i = 0; i < n_longs; i++)
        bitmap[i] = 0;

    uint32_t deltaN = 0;
    for(auto it = sns->get_begin();
            it != sns->get_end(); ++it)
    {
        deltaN = (*it - sns->base).low;
        assert((*it - sns->base).high == 0);
        if(deltaN < 256)
            bitmap[(uint32_t)(deltaN/32)] = (bitmap[(uint32_t)(deltaN/32)] | (1<<(31-deltaN%32)));
        else
            break;
    }

    for(uint32_t i= 0;i<n_longs;i++)
        addInt32(msg,bitmap[i]);

    delete[] bitmap;
    return true;
}

inline bool CDRMessage::addFragmentNumberSet(CDRMessage_t* msg,
        FragmentNumberSet_t* fns) {

    if (fns->base == 0)
        return false;

    CDRMessage::addUInt32(msg, fns->base);

    //Add set
    if (fns->isSetEmpty())
    {
        addUInt32(msg, 0); //numbits 0
        return true;
    }

    FragmentNumber_t maxfragNum = *(std::prev(fns->set.end()));

    uint32_t numBits = (uint32_t)(maxfragNum - fns->base + 1);

    if (numBits > 256)
        return false;

    addUInt32(msg, numBits);
    uint8_t n_longs = (uint8_t)((numBits + 31) / 32);
    int32_t* bitmap = new int32_t[n_longs];

    for (uint32_t i = 0; i<n_longs; i++)
        bitmap[i] = 0;

    uint32_t deltaN = 0;

    for (auto it = fns->get_begin();
            it != fns->get_end(); ++it)
    {
        deltaN = (uint32_t)(*it - fns->base);
        bitmap[(uint32_t)(deltaN / 32)] = (bitmap[(uint32_t)(deltaN / 32)] | (1 << (31 - deltaN % 32)));
    }

    for (uint32_t i = 0; i<n_longs; i++)
        addInt32(msg, bitmap[i]);

    delete[] bitmap;

    return true;
}

inline bool CDRMessage::addLocator(CDRMessage_t* msg, Locator_t* loc) {
    addInt32(msg,loc->kind);
    addUInt32(msg,loc->port);

    addData(msg,loc->address,16);

    return true;
}

inline bool CDRMessage::addParameterStatus(CDRMessage_t* msg, octet status)
{
    if(msg->pos+8>=msg->max_size)
    {
        return false;
    }
    CDRMessage::addUInt16(msg,PID_STATUS_INFO);
    CDRMessage::addUInt16(msg,4);
    CDRMessage::addOctet(msg,0);
    CDRMessage::addOctet(msg,0);
    CDRMessage::addOctet(msg,0);
    CDRMessage::addOctet(msg,status);
    return true;
}


inline bool CDRMessage::addParameterKey(CDRMessage_t* msg, const InstanceHandle_t* iHandle)
{
    if(msg->pos+20>=msg->max_size)
    {
        return false;
    }
    CDRMessage::addUInt16(msg,PID_KEY_HASH);
    CDRMessage::addUInt16(msg,16);
    for(uint8_t i=0;i<16;i++)
        msg->buffer[msg->pos+i] = iHandle->value[i];
    msg->pos+=16;
    msg->length+=16;
    return true;
}

inline bool CDRMessage::addParameterSentinel(CDRMessage_t* msg)
{
    if(msg->pos+4>=msg->max_size)
    {
        return false;
    }
    CDRMessage::addUInt16(msg, PID_SENTINEL);
    CDRMessage::addUInt16(msg, 0);

    return true;
}

inline bool CDRMessage::addString(CDRMessage_t*msg, const std::string& in_str)
{
    uint32_t str_siz = (uint32_t)in_str.size();
    int rest = (str_siz+1) % 4;
    if (rest != 0)
        rest = 4 - rest; //how many you have to add

    bool valid = CDRMessage::addUInt32(msg, str_siz+1);
    valid &= CDRMessage::addData(msg,
            (unsigned char*) in_str.c_str(), str_siz+1);
    if (rest != 0) {
        octet oc = '\0';
        for (int i = 0; i < rest; i++) {
            valid &= CDRMessage::addOctet(msg, oc);
        }
    }
    return valid;
}

inline bool CDRMessage::addParameterSampleIdentity(CDRMessage_t *msg, const SampleIdentity &sample_id)
{
    if(msg->pos + 28 > msg->max_size)
    {
        return false;
    }

    CDRMessage::addUInt16(msg, PID_RELATED_SAMPLE_IDENTITY);
    CDRMessage::addUInt16(msg, 24);
    CDRMessage::addData(msg, sample_id.writer_guid().guidPrefix.value, GuidPrefix_t::size);
    CDRMessage::addData(msg, sample_id.writer_guid().entityId.value, EntityId_t::size);
    CDRMessage::addInt32(msg, sample_id.sequence_number().high);
    CDRMessage::addUInt32(msg, sample_id.sequence_number().low);
    return true;
}

inline bool CDRMessage::addProperty(CDRMessage_t* msg, const Property& property)
{
    assert(msg);

    if(property.propagate())
    {
        if(!CDRMessage::addString(msg, property.name()))
            return false;
        if(!CDRMessage::addString(msg, property.value()))
            return false;
    }

    return true;
}

inline bool CDRMessage::readProperty(CDRMessage_t* msg, Property& property)
{
    assert(msg);

    if(!CDRMessage::readString(msg, &property.name()))
        return false;
    if(!CDRMessage::readString(msg, &property.value()))
        return false;

    return true;
}

inline bool CDRMessage::addBinaryProperty(CDRMessage_t* msg, const BinaryProperty& binary_property)
{
    assert(msg);

    if(binary_property.propagate())
    {
        if(!CDRMessage::addString(msg, binary_property.name()))
            return false;
        if(!CDRMessage::addOctetVector(msg, &binary_property.value()))
            return false;
    }

    return true;
}

inline bool CDRMessage::readBinaryProperty(CDRMessage_t* msg, BinaryProperty& binary_property)
{
    assert(msg);

    if(!CDRMessage::readString(msg, &binary_property.name()))
        return false;
    if(!CDRMessage::readOctetVector(msg, &binary_property.value()))
        return false;
    binary_property.propagate(true);

    return true;
}

inline bool CDRMessage::addPropertySeq(CDRMessage_t* msg, const PropertySeq& properties)
{
    assert(msg);

    bool returnedValue = false;

    if(msg->pos + 4 <=  msg->max_size)
    {
        uint32_t number_to_serialize = 0;
        for(auto it = properties.begin(); it != properties.end(); ++it)
            if(it->propagate())
                ++number_to_serialize;

        if(CDRMessage::addUInt32(msg, number_to_serialize))
        {
            returnedValue = true;
            for(auto it = properties.begin(); returnedValue && it != properties.end(); ++it)
                if(it->propagate())
                    returnedValue = CDRMessage::addProperty(msg, *it);
        }
    }

    return returnedValue;
}

inline bool CDRMessage::readPropertySeq(CDRMessage_t* msg, PropertySeq& properties)
{
    assert(msg);

    uint32_t length = 0;
    if(!CDRMessage::readUInt32(msg, &length))
        return false;

    properties.resize(length);
    bool returnedValue = true;
    for(uint32_t i = 0; returnedValue && i < length; ++i)
        returnedValue = CDRMessage::readProperty(msg, properties.at(i));

    return returnedValue;

}

inline bool CDRMessage::addBinaryPropertySeq(CDRMessage_t* msg, const BinaryPropertySeq& binary_properties)
{
    assert(msg);

    bool returnedValue = false;

    if(msg->pos + 4 <=  msg->max_size)
    {
        uint32_t number_to_serialize = 0;
        for(auto it = binary_properties.begin(); it != binary_properties.end(); ++it)
            if(it->propagate())
                ++number_to_serialize;

        if(CDRMessage::addUInt32(msg, number_to_serialize))
        {
            returnedValue = true;
            for(auto it = binary_properties.begin(); returnedValue && it != binary_properties.end(); ++it)
                if(it->propagate())
                    returnedValue = CDRMessage::addBinaryProperty(msg, *it);
        }
    }

    return returnedValue;
}

inline bool CDRMessage::addBinaryPropertySeq(CDRMessage_t* msg, const BinaryPropertySeq& binary_properties, const std::string& property_limit)
{
    assert(msg);

    bool returnedValue = false;

    if(msg->pos + 4 <=  msg->max_size)
    {
        uint32_t position = 0;
        uint32_t number_to_serialize = 0;
        for(auto it = binary_properties.begin(); it != binary_properties.end() &&
                it->name().compare(property_limit) != 0; ++it)
        {
            if(it->propagate())
                ++number_to_serialize;
            ++position;
        }

        if(CDRMessage::addUInt32(msg, number_to_serialize))
        {
            returnedValue = true;
            for(uint32_t i = 0; returnedValue && i < position; ++i)
                if(binary_properties.at(i).propagate())
                    returnedValue = CDRMessage::addBinaryProperty(msg, binary_properties.at(i));
        }
    }

    return returnedValue;
}

inline bool CDRMessage::readBinaryPropertySeq(CDRMessage_t* msg, BinaryPropertySeq& binary_properties)
{
    assert(msg);

    uint32_t length = 0;
    if(!CDRMessage::readUInt32(msg, &length))
        return false;

    binary_properties.resize(length);
    bool returnedValue = true;
    for(uint32_t i = 0; returnedValue && i < length; ++i)
        returnedValue = CDRMessage::readBinaryProperty(msg, binary_properties.at(i));

    return returnedValue;
}

inline bool CDRMessage::addDataHolder(CDRMessage_t* msg, const DataHolder& data_holder)
{
    assert(msg);

    if(!CDRMessage::addString(msg, data_holder.class_id()))
        return false;
    if(!CDRMessage::addPropertySeq(msg, data_holder.properties()))
        return false;
    if(!CDRMessage::addBinaryPropertySeq(msg, data_holder.binary_properties()))
        return false;

    return true;
}

inline bool CDRMessage::readDataHolder(CDRMessage_t* msg, DataHolder& data_holder)
{
    assert(msg);

    if(!CDRMessage::readString(msg, &data_holder.class_id()))
        return false;
    if(!CDRMessage::readPropertySeq(msg, data_holder.properties()))
        return false;
    if(!CDRMessage::readBinaryPropertySeq(msg, data_holder.binary_properties()))
        return false;

    return true;

}

inline bool CDRMessage::addDataHolderSeq(CDRMessage_t* msg, const DataHolderSeq& data_holders)
{
    assert(msg);

    bool returnedValue = false;

    if(msg->pos + 4 <=  msg->max_size)
    {
        if(CDRMessage::addUInt32(msg, static_cast<uint32_t>(data_holders.size())))
        {
            returnedValue = true;
            for(auto data_holder = data_holders.begin(); returnedValue && data_holder != data_holders.end(); ++data_holder)
                returnedValue = CDRMessage::addDataHolder(msg, *data_holder);
        }
    }

    return returnedValue;
}

inline bool CDRMessage::readDataHolderSeq(CDRMessage_t* msg, DataHolderSeq& data_holders)
{
    assert(msg);

    uint32_t length = 0;
    if(!CDRMessage::readUInt32(msg, &length))
        return false;

    data_holders.resize(length);
    bool returnedValue = true;
    for(uint32_t i = 0; returnedValue && i < length; ++i)
        returnedValue = CDRMessage::readDataHolder(msg, data_holders.at(i));

    return returnedValue;
}

inline bool CDRMessage::addMessageIdentity(CDRMessage_t* msg, const ::security::MessageIdentity& message_identity)
{
    assert(msg);

    if(!CDRMessage::addData(msg, message_identity.source_guid().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::addData(msg, message_identity.source_guid().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::addInt64(msg, message_identity.sequence_number()))
        return false;

    return true;
}

inline bool CDRMessage::readMessageIdentity(CDRMessage_t* msg, ::security::MessageIdentity& message_identity)
{
    assert(msg);

    if(!CDRMessage::readData(msg, message_identity.source_guid().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::readData(msg, message_identity.source_guid().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::readInt64(msg, &message_identity.sequence_number()))
        return false;

    return true;
}

inline bool CDRMessage::addParticipantGenericMessage(CDRMessage_t* msg, const ::security::ParticipantGenericMessage& message)
{
    assert(msg);

    if(!CDRMessage::addMessageIdentity(msg, message.message_identity()))
        return false;
    if(!CDRMessage::addMessageIdentity(msg, message.related_message_identity()))
        return false;
    if(!CDRMessage::addData(msg, message.destination_participant_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::addData(msg, message.destination_participant_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::addData(msg, message.destination_endpoint_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::addData(msg, message.destination_endpoint_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::addData(msg, message.source_endpoint_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::addData(msg, message.source_endpoint_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::addString(msg, message.message_class_id()))
        return false;
    if(!CDRMessage::addDataHolderSeq(msg, message.message_data()))
        return false;

    return true;
}

inline bool CDRMessage::readParticipantGenericMessage(CDRMessage_t* msg, ::security::ParticipantGenericMessage& message)
{
    assert(msg);

    if(!CDRMessage::readMessageIdentity(msg, message.message_identity()))
        return false;
    if(!CDRMessage::readMessageIdentity(msg, message.related_message_identity()))
        return false;
    if(!CDRMessage::readData(msg, message.destination_participant_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::readData(msg, message.destination_participant_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::readData(msg, message.destination_endpoint_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::readData(msg, message.destination_endpoint_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::readData(msg, message.source_endpoint_key().guidPrefix.value, GuidPrefix_t::size))
        return false;
    if(!CDRMessage::readData(msg, message.source_endpoint_key().entityId.value, EntityId_t::size))
        return false;
    if(!CDRMessage::readString(msg, &message.message_class_id()))
        return false;
    if(!CDRMessage::readDataHolderSeq(msg, message.message_data()))
        return false;

    return true;
}

}
} /* namespace rtps */
} /* namespace eprosima */

#endif /* DOXYGEN_SHOULD_SKIP_THIS */
