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

#include "rss/situation/SituationType.hpp"
#include "rss/world/Object.hpp"
#include "rss/world/RoadArea.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace world
 */
namespace world {

struct Scene // LCOV_EXCL_LINE
{
  ::rss::situation::SituationType situationType{::rss::situation::SituationType::SameDirection};
  ::rss::world::RoadArea egoVehicleRoad;
  ::rss::world::RoadArea intersectingRoad;
  ::rss::world::Object object;

  ::rss::situation::SituationType getSituationType() const
  {
    return situationType;
  }

  void setSituationType(::rss::situation::SituationType const newVal)
  {
    situationType = newVal;
  }
};

} // namespace world
} // namespace rss
