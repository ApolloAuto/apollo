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
 * \brief Enum LateralRelativePosition
 *
 * Enumeration describing the relative lateral position between two objects, a and b, within their situation coordinate
 * system.
 */
enum class LateralRelativePosition : int32_t
{
  AtLeft
  = 0, /*!< The object a is completely left of object b. This means there is an actual lateral space between them. */
  OverlapLeft = 1, /*!< The objects overlap. The left border of object a is left of the left border of object b AND the
                      right border of object a is left of the right border of object b. */
  Overlap = 2, /*!< The objects overlap, but neither the conditions for OverlapLeft nor OverlapRight are applicable. */
  OverlapRight = 3, /*!< The objects overlap. The left border of object a is right of the left border of object b AND
                       the right border of object a is right of the right border of object b. */
  AtRight
  = 4 /*!< The object a is completely right of object b. This means there is an actual lateral space between them. */
};

} // namespace situation
} // namespace rss
/*!
 * @brief Conversion of ::rss::situation::LateralRelativePosition to std::string helper.
 *
 */
std::string toString(::rss::situation::LateralRelativePosition const e);

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

template <>::rss::situation::LateralRelativePosition fromString(std::string const &str);
