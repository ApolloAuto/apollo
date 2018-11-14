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
 * \brief Enum SituationType
 *
 * Enumeration describing the type of situation.
 */
enum class SituationType : int32_t
{
  NotRelevant
  = 0, /*!< The other vehicle cannot conflict with the ego vehicle. This kind of situations are always considered to be
          safe. Use this situation state to make the object visible in the result vector to be a known object, but not
          relevant for RSS (e.g. object in opposite direction, but already passed by). */
  SameDirection = 1,     /*!< Both drive on the same road in the same direction. */
  OppositeDirection = 2, /*!< Both drive on the same road in the opposite direction. */
  IntersectionEgoHasPriority
  = 3, /*!< Both drive on individual roads which intersect at the end. Ego vehicle has priority over object. */
  IntersectionObjectHasPriority
  = 4, /*!< Both drive on individual roads which intersect at the end. Object has priority over ego vehicle. */
  IntersectionSamePriority
  = 5 /*!< Both drive on individual roads which intersect at the end. Object and ego vehicle have same priority. */
};

} // namespace situation
} // namespace rss
/*!
 * @brief Conversion of ::rss::situation::SituationType to std::string helper.
 *
 */
std::string toString(::rss::situation::SituationType const e);

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

template <>::rss::situation::SituationType fromString(std::string const &str);
