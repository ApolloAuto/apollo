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
 * @file FragmentNumber.h
 */

#ifndef RPTS_ELEM_FRAGNUM_H_
#define RPTS_ELEM_FRAGNUM_H_
#include "../../fastrtps_dll.h"
#include "Types.h"

#include <set>
#include <cmath>
#include <algorithm>
#include <sstream>
namespace eprosima{
namespace fastrtps{
namespace rtps{

typedef uint32_t FragmentNumber_t;

//!Structure FragmentNumberSet_t, contains a group of fragmentnumbers.
//!@ingroup COMMON_MODULE
class FragmentNumberSet_t
{
    public:

        //!Base fragment number
        FragmentNumber_t base;

        /**
         * Assignment operator
         * @param set2 FragmentNumberSet_t to copy the data from
         */
        FragmentNumberSet_t& operator=(const FragmentNumberSet_t& set2)
        {
            base = set2.base;
            set = set2.set;
            return *this;
        }

        FragmentNumberSet_t(): base(0), set() {}

        FragmentNumberSet_t(const std::set<FragmentNumber_t>& set2) : base(0)
        {
            set = set2;
            auto min = set.begin();
            if (min != set.end())
                base = *min;
        }

        /**
         * Compares object with other FragmentNumberSet_t.
         * @param other FragmentNumberSet_t to compare
         * @return True if equal
         */
        bool operator==(const FragmentNumberSet_t& other) {

            if (base != other.base)
                return false;
            return other.set == set;	
        }


        /**
         * Add a fragment number to the set
         * @param in FragmentNumberSet_t to add
         * @return True on success
         */
        bool add(FragmentNumber_t in)
        {
            if (in >= base && in <= base + 255)
                set.insert(in);
            else
                return false;
            return true;
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
        std::set<FragmentNumber_t>::const_iterator get_begin() const
        {
            return set.begin();
        }

        /**
         * Get the end of the set
         * @return Vector iterator pointing to the end of the set
         */
        std::set<FragmentNumber_t>::const_iterator get_end() const
        {
            return set.end();
        }

        /**
         * Get the number of FragmentNumbers in the set
         * @return Size of the set
         */
        size_t get_size()
        {
            return set.size();
        }

        /**
         * Get a string representation of the set
         * @return string representation of the set
         */
        std::string print()
        {
            std::stringstream ss;
            ss << base << ":";
            for (auto it = set.begin(); it != set.end(); ++it)
                ss << *it << "-";
            return ss.str();
        }

        FragmentNumberSet_t& operator-=(const FragmentNumberSet_t& rhs)
        {
            for ( auto element : rhs.set)
                set.erase(element);
            return *this;
        }

        FragmentNumberSet_t& operator-=(const FragmentNumber_t& fragment_number)
        {
            set.erase(fragment_number);
            return *this;
        }

        FragmentNumberSet_t& operator+=(const FragmentNumberSet_t& rhs)
        {
            for ( auto element : rhs.set )
                add(element);
            return *this;
        }

        std::set<FragmentNumber_t> set;
};

/**
 * Prints a Fragment Number set
 * @param output Output Stream
 * @param sns SequenceNumber set
 * @return OStream.
 */
inline std::ostream& operator<<(std::ostream& output, FragmentNumberSet_t& sns){
    return output << sns.print();
}

inline FragmentNumberSet_t operator-(FragmentNumberSet_t lhs, const FragmentNumberSet_t& rhs)
{
    for ( auto element : rhs.set)
        lhs.set.erase(element);
    return lhs;
}

inline FragmentNumberSet_t operator+(FragmentNumberSet_t lhs, const FragmentNumberSet_t& rhs)
{
    for ( auto element : rhs.set)
        lhs.add(element);
    return lhs;
}

}
}
}

#endif /* RPTS_ELEM_FRAGNUM_H_ */
