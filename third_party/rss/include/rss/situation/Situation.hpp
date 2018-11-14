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

#include "rss/situation/RelativePosition.hpp"
#include "rss/situation/SituationId.hpp"
#include "rss/situation/SituationType.hpp"
#include "rss/situation/VehicleState.hpp"
#include "rss/time/TimeIndex.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace situation
 */
namespace situation {

struct Situation
{
  ::rss::time::TimeIndex timeIndex;
  ::rss::situation::SituationId situationId;
  ::rss::situation::SituationType situationType;
  ::rss::situation::VehicleState egoVehicleState;
  ::rss::situation::VehicleState otherVehicleState;
  ::rss::situation::RelativePosition relativePosition;

  ::rss::situation::SituationType getSituationType() const
  {
    return situationType;
  }

  void setSituationType(::rss::situation::SituationType const newVal)
  {
    situationType = newVal;
  }
};

} // namespace situation
} // namespace rss
