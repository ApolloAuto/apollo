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

#include "rss/situation/Distance.hpp"
#include "rss/situation/LateralRelativePosition.hpp"
#include "rss/situation/LongitudinalRelativePosition.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace situation
 */
namespace situation {

struct RelativePosition
{
  ::rss::situation::LongitudinalRelativePosition longitudinalPosition{
    ::rss::situation::LongitudinalRelativePosition::Overlap};
  ::rss::situation::Distance longitudinalDistance{0.0};
  ::rss::situation::LateralRelativePosition lateralPosition{::rss::situation::LateralRelativePosition::Overlap};
  ::rss::situation::Distance lateralDistance{0.0};

  ::rss::situation::LongitudinalRelativePosition getLongitudinalPosition() const
  {
    return longitudinalPosition;
  }

  void setLongitudinalPosition(::rss::situation::LongitudinalRelativePosition const newVal)
  {
    longitudinalPosition = newVal;
  }

  ::rss::situation::LateralRelativePosition getLateralPosition() const
  {
    return lateralPosition;
  }

  void setLateralPosition(::rss::situation::LateralRelativePosition const newVal)
  {
    lateralPosition = newVal;
  }
};

} // namespace situation
} // namespace rss
