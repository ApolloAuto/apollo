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

#include "rss/state/LongitudinalResponse.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace state
 */
namespace state {
struct LongitudinalRssState
{
  //c++11
  bool isSafe;
  ::rss::state::LongitudinalResponse response;

  //c++14
  //bool isSafe = false;
  //::rss::state::LongitudinalResponse response= ::rss::state::LongitudinalResponse::BrakeMin;

  ::rss::state::LongitudinalResponse getResponse() const
  {
    return response;
  }

  void setResponse(::rss::state::LongitudinalResponse const newVal)
  {
    response = newVal;
  }
};

} // namespace state
} // namespace rss
