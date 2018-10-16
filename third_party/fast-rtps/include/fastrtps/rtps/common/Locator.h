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
 * @file Locator.h
 */

#ifndef RTPS_ELEM_LOCATOR_H_
#define RTPS_ELEM_LOCATOR_H_
#include "../../fastrtps_dll.h"
#include "Types.h"
#include <sstream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <algorithm>
namespace eprosima{
namespace fastrtps{
namespace rtps{

#define LOCATOR_INVALID(loc)  {loc.kind=LOCATOR_KIND_INVALID;loc.port= LOCATOR_PORT_INVALID;LOCATOR_ADDRESS_INVALID(loc.address);}
#define LOCATOR_KIND_INVALID -1

#define LOCATOR_ADDRESS_INVALID(a) {std::memset(a,0x00,16*sizeof(octet));}
#define LOCATOR_PORT_INVALID 0
#define LOCATOR_KIND_RESERVED 0
#define LOCATOR_KIND_UDPv4 1
#define LOCATOR_KIND_UDPv6 2


//!@brief Class Locator_t, uniquely identifies a communication channel for a particular transport. 
//For example, an address+port combination in the case of UDP.
//!@ingroup COMMON_MODULE
class RTPS_DllAPI Locator_t
{
    public:

        /*!
         * @brief Specifies the locator type. Valid values are:
         * LOCATOR_KIND_UDPv4
         * LOCATOR_KIND_UDPv6
         */
        int32_t kind;
        uint32_t port;
        octet address[16];

        //!Default constructor
        Locator_t():kind(1),port(0)
    {
        LOCATOR_ADDRESS_INVALID(address);
    }

        Locator_t(Locator_t&& loc):
            kind(loc.kind),
            port(loc.port)
    {
        std::memcpy(address,loc.address,16*sizeof(octet));
    }

        Locator_t(const Locator_t& loc) :
            kind(loc.kind),
            port(loc.port)
    {
        std::memcpy(address,loc.address,16*sizeof(octet));
    }

        Locator_t(uint32_t portin):kind(1),port(portin)
    {
        LOCATOR_ADDRESS_INVALID(address);
    }

        Locator_t& operator=(const Locator_t& loc)
        {
            kind = loc.kind;
            port = loc.port;
            std::memcpy(address,loc.address,16*sizeof(octet));
            return *this;
        }

        bool set_IP4_address(octet o1,octet o2,octet o3,octet o4){
            LOCATOR_ADDRESS_INVALID(address);
            address[12] = o1;
            address[13] = o2;
            address[14] = o3;
            address[15] = o4;
            return true;
        }
        bool set_IP4_address(const std::string& in_address)
        {
            std::stringstream ss(in_address);
            int a,b,c,d; //to store the 4 ints
            char ch; //to temporarily store the '.'
            ss >> a >> ch >> b >> ch >> c >> ch >> d;
            LOCATOR_ADDRESS_INVALID(address);
            address[12] = (octet)a;
            address[13] = (octet)b;
            address[14] = (octet)c;
            address[15] = (octet)d;
            return true;
        }
        std::string to_IP4_string() const {
            std::stringstream ss;
            ss << (int)address[12] << "." << (int)address[13] << "." << (int)address[14]<< "." << (int)address[15];
            return ss.str();
        }
        uint32_t to_IP4_long()
        {
            uint32_t addr;
            octet* oaddr = (octet*)&addr;
#if __BIG_ENDIAN__
            std::memcpy(oaddr,address+12,4*sizeof(octet));
#else
            // TODO (Santi) - Are we sure we want to flip this?
            oaddr[0] = address[15];
            oaddr[1] = address[14];
            oaddr[2] = address[13];
            oaddr[3] = address[12];
#endif

            return addr;
        }

        bool set_IP6_address(uint16_t group0, uint16_t group1, uint16_t group2, uint16_t group3,
                uint16_t group4, uint16_t group5, uint16_t group6, uint16_t group7)
        {
            address[0]  = (octet) (group0 >> 8);
            address[1]  = (octet) group0;
            address[2]  = (octet) (group1 >> 8);
            address[3]  = (octet) group1;
            address[4]  = (octet) (group2 >> 8);
            address[5]  = (octet) group2;
            address[6]  = (octet) (group3 >> 8);
            address[7]  = (octet) group3;
            address[8]  = (octet) (group4 >> 8);
            address[9]  = (octet) group4;
            address[10] = (octet) (group5 >> 8);
            address[11] = (octet) group5;
            address[12] = (octet) (group6 >> 8);
            address[13] = (octet) group6;
            address[14] = (octet) (group7 >> 8);
            address[15] = (octet) group7;
            return true;
        }

        std::string to_IP6_string() const{
            std::stringstream ss;
            ss << std::hex;
            for (int i = 0; i != 14; i+= 2) 
            {
                auto field = (address[i] << 8) + address[i+1];
                ss << field << ":";
            }
            auto field = address[14] + (address[15] << 8);
            ss << field;
            return ss.str();
        }
};


inline bool IsAddressDefined(const Locator_t& loc)
{
    if(loc.kind == LOCATOR_KIND_UDPv4)
    {
        for(uint8_t i = 12; i < 16; ++i)
        {
            if(loc.address[i] != 0)
                return true;
        }
    }
    else if (loc.kind == LOCATOR_KIND_UDPv6)
    {
        for(uint8_t i = 0; i < 16; ++i)
        {
            if(loc.address[i] != 0)
                return true;
        }
    }
    return false;
}

inline bool IsLocatorValid(const Locator_t&loc)
{
    if(loc.kind<0)
        return false;
    return true;
}

inline bool operator==(const Locator_t&loc1,const Locator_t& loc2)
{
    if(loc1.kind!=loc2.kind)
        return false;
    if(loc1.port !=loc2.port)
        return false;
    //for(uint8_t i =0;i<16;i++){
    //	if(loc1.address[i] !=loc2.address[i])
    //		return false;
    //}
    if(!std::equal(loc1.address,loc1.address+16,loc2.address))
        return false;
    return true;
}

inline std::ostream& operator<<(std::ostream& output,const Locator_t& loc)
{
    if(loc.kind == LOCATOR_KIND_UDPv4)
    {
        output<<(int)loc.address[12] << "." << (int)loc.address[13] << "." << (int)loc.address[14]<< "." << (int)loc.address[15]<<":"<<loc.port;
    }
    else if(loc.kind == LOCATOR_KIND_UDPv6)
    {
        for(uint8_t i =0;i<16;++i)
        {
            output<<(int)loc.address[i];
            if(i<15)
                output<<".";
        }
        output<<":"<<loc.port;
    }
    return output;
}



typedef std::vector<Locator_t>::iterator LocatorListIterator;
typedef std::vector<Locator_t>::const_iterator LocatorListConstIterator;


/**
 * Class LocatorList_t, a Locator_t vector that doesn't avoid duplicates.
 * @ingroup COMMON_MODULE
 */
class LocatorList_t
{
    public:
        RTPS_DllAPI LocatorList_t(){};

        RTPS_DllAPI ~LocatorList_t(){};

        RTPS_DllAPI LocatorList_t(const LocatorList_t& list) : m_locators(list.m_locators) {}

        RTPS_DllAPI LocatorList_t(LocatorList_t&& list) : m_locators(std::move(list.m_locators)) {}

        RTPS_DllAPI LocatorList_t& operator=(const LocatorList_t& list)
        {
            m_locators = list.m_locators;
            return *this;
        }

        RTPS_DllAPI LocatorList_t& operator=(LocatorList_t&& list)
        {
            m_locators = std::move(list.m_locators);
            return *this;
        }

        RTPS_DllAPI bool operator==(const LocatorList_t& locator_list) const
        {
            if(locator_list.m_locators.size() == m_locators.size())
            {
                 bool returnedValue = true;

                 for(auto it = locator_list.m_locators.begin(); returnedValue &&
                         it != locator_list.m_locators.end(); ++it)
                 {
                     returnedValue = false;

                     for(auto it2 = m_locators.begin();  !returnedValue && it2 != m_locators.end(); ++it2)
                     {
                         if(*it == *it2)
                             returnedValue = true;
                     }
                 }

                 return returnedValue;
            }

            return false;
        }

        RTPS_DllAPI LocatorListIterator begin(){
            return m_locators.begin();
        }

        RTPS_DllAPI LocatorListIterator end(){
            return m_locators.end();
        }

        RTPS_DllAPI LocatorListConstIterator begin() const {
            return m_locators.begin();
        }

        RTPS_DllAPI LocatorListConstIterator end() const {
            return m_locators.end();
        }

        RTPS_DllAPI size_t size(){
            return m_locators.size();
        }

        RTPS_DllAPI void clear(){ return m_locators.clear();}

        RTPS_DllAPI void reserve(size_t num){ return m_locators.reserve(num);}

        RTPS_DllAPI void resize(size_t num) { return m_locators.resize(num);}

        RTPS_DllAPI void push_back(const Locator_t& loc)
        {
            bool already = false;
            for(LocatorListIterator it=this->begin(); it!=this->end(); ++it)
            {
                if(loc == *it)
                {
                    already = true;
                    break;
                }
            }
            if(!already)
                m_locators.push_back(loc);
        }

        RTPS_DllAPI void push_back(const LocatorList_t& locList)
        {
            for(auto it = locList.m_locators.begin(); it!=locList.m_locators.end(); ++it)
            {
                this->push_back(*it);
            }
        }

        RTPS_DllAPI bool empty(){
            return m_locators.empty();
        }

        RTPS_DllAPI void erase(const Locator_t& loc)
        {
            std::remove(m_locators.begin(), m_locators.end(), loc);
        }

        RTPS_DllAPI bool contains(const Locator_t& loc)
        {
            for(LocatorListIterator it=this->begin();it!=this->end();++it)
            {
                if(IsAddressDefined(*it))
                {
                    if(loc == *it)
                        return true;
                }
                else
                {
                    if(loc.kind == (*it).kind && loc.port == (*it).port)
                        return true;
                }
            }

            return false;
        }

        RTPS_DllAPI bool isValid()
        {
            for(LocatorListIterator it=this->begin();it!=this->end();++it)
            {
                if(!IsLocatorValid(*it))
                    return false;
            }
            return true;
        }


        RTPS_DllAPI void swap(LocatorList_t& locatorList)
        {
            this->m_locators.swap(locatorList.m_locators);
        }

        friend std::ostream& operator <<(std::ostream& output,const LocatorList_t& loc);

    private:

        std::vector<Locator_t> m_locators;
};

inline std::ostream& operator<<(std::ostream& output,const LocatorList_t& locList)
{
    for(auto it = locList.m_locators.begin();it!=locList.m_locators.end();++it)
    {
        output << *it << ",";
    }
    return output;
}

}
}
}

#endif /* RTPS_ELEM_LOCATOR_H_ */
