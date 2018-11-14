/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 2018 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials, and
 * your use of them is governed by the express license under which they were
 * provided to you (License). Unless the License provides otherwise, you may not
 * use, modify, copy, publish, distribute, disclose or transmit this software or
 * the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/**
 * @file
 *
 */

#pragma once
#include <limits>

#include <cstdint>
#include <string>
#include <unordered_map>
namespace rss {
namespace situation {
/*!
 * \brief Enum LongitudinalRelativePosition
 *
 * Enumeration describing the relative longitudinal position between two objects, a and b, within their situation
 * coordinate system.
 */
enum class LongitudinalRelativePosition : int32_t
{
  InFront = 0, /*!< The object a is completely in front of object b. This means there is an actual longitudinal space
                  between them. */
  OverlapFront = 1, /*!< The objects overlap. The front border of object a is in front of the front border of object b
                       AND the back border of object a is in front of the back border of object b. */
  Overlap = 2, /*!< The objects overlap, but neither the conditions for OverlapFront nor OverlapBack are applicable. */
  OverlapBack
  = 3, /*!< The objects overlap. The front border of object a is at back of the front border of object b AND the back
          border of object a is at back of the back border of object b. */
  AtBack = 4 /*!< The object a is completely at back of object b. This means there is an actual longitudinal space
                between them. */
};

} // namespace situation
} // namespace rss
/*!
 * @brief Conversion of ::rss::situation::LongitudinalRelativePosition to std::string helper.
 *
 */
std::string toString(::rss::situation::LongitudinalRelativePosition const e);

/*!
 * @brief Conversion from std::string to enum type T helper.
 *
 * @param [in] str - a fully qualified string name of enum class type
 *
 * @return T enum value
 *
 * @throws std::out_of_range exception if the given string does not match any enum type
 *
 * Example usage:
 * @code
 *   auto value = fromString<SomeEnumType>("SomeEnumType::eValue");
 *   assert(value == SomeEnumType::eValue);
 *   // Or:
 *   auto value = fromString<SomeEnumType>("eValue");
 *   assert(value == SomeEnumType::eValue);
 * @endcode
 */
template <typename EnumType> EnumType fromString(std::string const &str);

template <>::rss::situation::LongitudinalRelativePosition fromString(std::string const &str);
