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
#include "rss/situation/Dynamics.hpp"
#include "rss/situation/Velocity.hpp"
#include "rss/time/Duration.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace situation
 */
namespace situation {

struct VehicleState
{
  ::rss::situation::Velocity velocity;
  ::rss::situation::Dynamics dynamics;
  ::rss::time::Duration responseTime{0.0};
  bool hasPriority{false};
  bool isInCorrectLane{false};
  ::rss::situation::Distance distanceToEnterIntersection{std::numeric_limits<Distance>::max()};
  ::rss::situation::Distance distanceToLeaveIntersection{std::numeric_limits<Distance>::max()};
};

} // namespace situation
} // namespace rss
