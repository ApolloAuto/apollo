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
 * \brief Enum CoordinateSystemAxis
 *
 * Enumeration defining the axis of the situation coordinate system.
 */
enum class CoordinateSystemAxis : int32_t
{
  Longitudinal = 0, /*!< longitudinal axis: ego vehicle is driving in positive direction */
  Lateral = 1       /*!< lateral axis: ego vehicle left to right defines the positive direction */
};

} // namespace situation
} // namespace rss
/*!
 * @brief Conversion of ::rss::situation::CoordinateSystemAxis to std::string helper.
 *
 */
std::string toString(::rss::situation::CoordinateSystemAxis const e);

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

template <>::rss::situation::CoordinateSystemAxis fromString(std::string const &str);
