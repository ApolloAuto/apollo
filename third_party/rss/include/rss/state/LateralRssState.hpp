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

#include "rss/state/LateralResponse.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace state
 */
namespace state {

struct LateralRssState
{
  //c++14 only
  //bool isSafe{false};
  
  bool isSafe;
  ::rss::state::LateralResponse response;

  ::rss::state::LateralResponse getResponse() const
  {
    return response;
  }

  void setResponse(::rss::state::LateralResponse const newVal)
  {
    response = newVal;
  }
};

} // namespace state
} // namespace rss
