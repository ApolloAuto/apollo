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

#include <string>
#include "rss/time/TimeIndex.hpp"
#include "rss/world/AccelerationRange.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace world
 */
namespace world {

struct AccelerationRestriction
{
  ::rss::time::TimeIndex timeIndex{0u};
  ::rss::world::AccelerationRange lateralLeftRange;
  ::rss::world::AccelerationRange longitudinalRange;
  ::rss::world::AccelerationRange lateralRightRange;
};

/*
 * \brief Event to support type within statecharts
 */
struct evRssAccelerationRestriction
{
  evRssAccelerationRestriction(AccelerationRestriction const &accelerationRestriction)
    : accelerationRestriction(accelerationRestriction)
  {
  }

  AccelerationRestriction const &data() const
  {
    return accelerationRestriction;
  }

  AccelerationRestriction const &accelerationRestriction;
};

} // namespace world
} // namespace rss

/*!
 * @brief Conversion of event evRssAccelerationRestriction to std::string (for logging purposes)
 */
std::string toString(::rss::world::evRssAccelerationRestriction const &);
