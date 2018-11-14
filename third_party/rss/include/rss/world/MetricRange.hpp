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

#include "rss/world/Distance.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace world
 */
namespace world {

struct MetricRange
{
  ::rss::world::Distance minimum{std::numeric_limits<Distance>::lowest()};
  ::rss::world::Distance maximum{std::numeric_limits<Distance>::max()};
};

} // namespace world
} // namespace rss
