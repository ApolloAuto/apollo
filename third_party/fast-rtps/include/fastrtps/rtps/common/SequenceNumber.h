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
 * @file SequenceNumber.h
 */

#ifndef RPTS_ELEM_SEQNUM_H_
#define RPTS_ELEM_SEQNUM_H_
#include "../../fastrtps_dll.h"
#include "Types.h"

#include <vector>
#include <algorithm>
#include <sstream>
#include <limits.h>
#include <cassert>

namespace eprosima{
namespace fastrtps{
namespace rtps{


//!@brief Structure SequenceNumber_t, different for each change in the same writer.
//!@ingroup COMMON_MODULE
struct RTPS_DllAPI SequenceNumber_t
{
    //!
    int32_t high;
    //!
    uint32_t low;

    //!Default constructor
    SequenceNumber_t()
    {
        high = 0;
        low = 0;
    }

    /*!
     * @brief Copy constructor.
     */
    SequenceNumber_t(const SequenceNumber_t& seq) : high(seq.high), low(seq.low)
    {
    }

    /*!
     * @param hi
     * @param lo
     */
    SequenceNumber_t(int32_t hi, uint32_t lo): high(hi),low(lo)
    {
    }

    // Check the target support 64bits.
#ifdef LLONG_MAX
    /*! Convert the number to 64 bit.
     * @return 64 bit representation of the SequenceNumber
     */
    uint64_t to64long() const
    {
        return (((uint64_t)high) << 32) + low;
    }
#endif

    /*!
     * Assignment operator
     * @param seq SequenceNumber_t to copy the data from
     */
    SequenceNumber_t& operator=(const SequenceNumber_t& seq)
    {
        high = seq.high;
        low = seq.low;
        return *this;
    }


    //! Increase SequenceNumber in 1.
    SequenceNumber_t& operator++()
    {
        if(low == UINT32_MAX)
        { ++high; low = 0; }
        else
            ++low;

        return *this;
    }

    SequenceNumber_t operator++(int)
    {
        SequenceNumber_t result(*this);
        ++(*this);
        return result;
    }

    /**
     * Increase SequenceNumber.
     * @param inc Number to add to the SequenceNumber
     */
    SequenceNumber_t& operator+=(int inc)
    {
        uint32_t aux_low = low;
        low += inc;

        if(low < aux_low)
        {
            // Being the type of the parameter an 'int', the increment of 'high' will be as much as 1.
            ++high;
        }

        return *this;
    }

    static SequenceNumber_t unknown()
    {
        return SequenceNumber_t(-1, 0);
    }

};

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * Compares two SequenceNumber_t.
 * @param sn1 First SequenceNumber_t to compare
 * @param sn2 Second SequenceNumber_t to compare
 * @return True if equal
 */
inline bool operator==(const SequenceNumber_t& sn1, const SequenceNumber_t& sn2)
{
    if(sn1.high != sn2.high || sn1.low != sn2.low)
        return false;

    return true;
}

/**
 * Compares two SequenceNumber_t.
 * @param sn1 First SequenceNumber_t to compare
 * @param sn2 Second SequenceNumber_t to compare
 * @return True if not equal
 */
inline bool operator!=(const SequenceNumber_t& sn1, const SequenceNumber_t& sn2)
{
    if(sn1.high == sn2.high && sn1.low == sn2.low)
        return false;

    return true;
}

/**
 * Checks if a SequenceNumber_t is greater than other.
 * @param seq1 First SequenceNumber_t to compare
 * @param seq2 Second SequenceNumber_t to compare
 * @return True if the first SequenceNumber_t is greater than the second
 */
inline bool operator>(const SequenceNumber_t& seq1, const SequenceNumber_t& seq2)
{
    if(seq1.high > seq2.high)
        return true;
    else if(seq1.high < seq2.high)
        return false;
    else
    {
        if(seq1.low > seq2.low)
            return true;
    }
    return false;
}

/**
 * Checks if a SequenceNumber_t is less than other.
 * @param seq1 First SequenceNumber_t to compare
 * @param seq2 Second SequenceNumber_t to compare
 * @return True if the first SequenceNumber_t is less than the second
 */
inline bool operator<(const SequenceNumber_t& seq1, const SequenceNumber_t& seq2)
{
    if(seq1.high > seq2.high)
        return false;
    else if(seq1.high < seq2.high)
        return true;
    else
    {
        if(seq1.low < seq2.low)
            return true;
    }
    return false;
}

/**
 * Checks if a SequenceNumber_t is greater or equal than other.
 * @param seq1 First SequenceNumber_t to compare
 * @param seq2 Second SequenceNumber_t to compare
 * @return True if the first SequenceNumber_t is greater or equal than the second
 */
inline bool operator>=(const SequenceNumber_t& seq1, const SequenceNumber_t& seq2)
{
    if(seq1.high > seq2.high)
        return true;
    else if(seq1.high < seq2.high)
        return false;
    else
    {
        if(seq1.low >= seq2.low)
            return true;
    }
    return false;
}

/**
 * Checks if a SequenceNumber_t is less or equal than other.
 * @param seq1 First SequenceNumber_t to compare
 * @param seq2 Second SequenceNumber_t to compare
 * @return True if the first SequenceNumber_t is less or equal than the second
 */
inline bool operator<=( const SequenceNumber_t& seq1, const  SequenceNumber_t& seq2)
{
    if(seq1.high > seq2.high)
        return false;
    else if(seq1.high < seq2.high)
        return true;
    else
    {
        if(seq1.low <= seq2.low)
            return true;
    }
    return false;
}

/**
 * Subtract one SequenceNumber_t from another
 * @param seq Base SequenceNumber_t
 * @param inc SequenceNumber_t to substract
 * @return Result of the substraction
 */
inline SequenceNumber_t operator-(const SequenceNumber_t& seq, const uint32_t inc)
{
    SequenceNumber_t res(seq.high, seq.low - inc);

    if(inc > seq.low)
    {
        // Being the type of the parameter an 'uint32_t', the decrement of 'high' will be as much as 1.
        --res.high;
    }

    return res;
}

/**
 * Add one SequenceNumber_t to another
 * @param[in] seq Base sequence number
 * @param inc value to add to the base
 * @return Result of the addition
 */
inline SequenceNumber_t operator+(const SequenceNumber_t& seq, const uint32_t inc)
{
    SequenceNumber_t res(seq.high, seq.low + inc);

    if(res.low < seq.low)
    {
        // Being the type of the parameter an 'uint32_t', the increment of 'high' will be as much as 1.
        ++res.high;
    }

    return res;
}

/**
 * Subtract one SequenceNumber_t to another
 * @param minuend Minuend. Has to be greater than or equal to subtrahend.
 * @param subtrahend Subtrahend.
 * @return Result of the subtraction
 */
inline SequenceNumber_t operator-(const SequenceNumber_t& minuend, const SequenceNumber_t& subtrahend)
{
    assert(minuend >= subtrahend);
    SequenceNumber_t res(minuend.high - subtrahend.high, minuend.low - subtrahend.low);

    if(minuend.low < subtrahend.low)
        --res.high;

    return res;
}

#endif

const SequenceNumber_t c_SequenceNumber_Unknown(-1,0);

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * Sorts two instances of SequenceNumber_t
 * @param s1 First SequenceNumber_t to compare
 * @param s2 First SequenceNumber_t to compare
 * @return True if s1 is less than s2
 */
inline bool sort_seqNum(const SequenceNumber_t& s1, const SequenceNumber_t& s2)
{
    return(s1 < s2);
}

/**
 * 
 * @param output
 * @param seqNum
 * @return 
 */
inline std::ostream& operator<<(std::ostream& output, const SequenceNumber_t& seqNum)
{
#ifdef LLONG_MAX
    return output << seqNum.to64long();
#else
    return output << "{high: " << seqNum.high << ", low: " << seqNum.low << "}";
#endif
}

inline std::ostream& operator<<(std::ostream& output, std::vector<SequenceNumber_t>& seqNumSet)
{
    for(std::vector<SequenceNumber_t>::iterator sit = seqNumSet.begin(); sit != seqNumSet.end(); ++sit)
    {
        output << *sit << " ";
    }
    return output;
}

/*!
 * @brief Defines the STL hash function for type SequenceNumber_t.
 */
struct SequenceNumberHash
{
    std::size_t operator()(const SequenceNumber_t& sequence_number) const
    {
#ifdef LLONG_MAX
        return static_cast<std::size_t>(sequence_number.to64long());
#else
        return static_cast<std::size_t>(sequence_number.low);
#endif
    };
};

#endif

//!Structure SequenceNumberSet_t, contains a group of sequencenumbers.
//!@ingroup COMMON_MODULE
class SequenceNumberSet_t
{
    public:
        //!Base sequence number
        SequenceNumber_t base;

        /**
         * Assignment operator
         * @param set2 SequenceNumberSet_t to copy the data from
         */
        SequenceNumberSet_t& operator=(const SequenceNumberSet_t& set2)
        {
            base = set2.base;
            set = set2.set;
            return *this;
        }

        /**
         * Add a sequence number to the set
         * @param in SequenceNumberSet_t to add
         * @return True on success
         */
        bool add(const SequenceNumber_t& in)
        {
            if(in >= base && in < base + 255)
                set.push_back(in);
            else
                return false;

            return true;
        }

        /**
         * Get the maximum sequence number in the set
         * @return maximum sequence number in the set
         */
        SequenceNumber_t get_maxSeqNum() const
        {
            return *std::max_element(set.begin(),set.end(),sort_seqNum);
        }

        /**
         * Check if the set is empty
         * @return True if the set is empty
         */
        bool isSetEmpty() const
        {
            return set.empty();
        }

        /**
         * Get the begin of the set
         * @return Vector iterator pointing to the begin of the set
         */
        std::vector<SequenceNumber_t>::const_iterator get_begin() const
        {
            return set.begin();
        }

        /**
         * Get the end of the set
         * @return Vector iterator pointing to the end of the set
         */
        std::vector<SequenceNumber_t>::const_iterator get_end() const
        {
            return set.end();
        }

        /**
         * Get the number of SequenceNumbers in the set
         * @return Size of the set
         */
        size_t get_size()
        {
            return set.size();
        }

        /**
         * Get the set of SequenceNumbers 
         * @return Set of SequenceNumbers
         */
        std::vector<SequenceNumber_t> get_set()
        {
            return set;
        }

        /**
         * Get a string representation of the set
         * @return string representation of the set
         */
        std::string print()
        {
            std::stringstream ss;

#ifdef LLONG_MAX
            ss << base.to64long() << ":";
#else
            ss << "{high: " << base.high << ", low: " << base.low << "} :";
#endif
            for(std::vector<SequenceNumber_t>::iterator it = set.begin(); it != set.end(); ++it)
            {
#ifdef LLONG_MAX
                ss << it->to64long() << "-";
#else
                ss << "{high: " << it->high << ", low: " << it->low << "} -";
#endif
            }
            return ss.str();
        }

    private:
        std::vector<SequenceNumber_t> set;
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/**
 * Prints a sequence Number set
 * @param output Output Stream
 * @param sns SequenceNumber set
 * @return OStream.
 */
inline std::ostream& operator<<(std::ostream& output, SequenceNumberSet_t& sns)
{
    return output << sns.print();
}

#endif

}
}
}

#endif /* RPTS_ELEM_SEQNUM_H_ */
