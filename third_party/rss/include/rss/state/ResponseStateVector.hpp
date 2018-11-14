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
#include <vector>
#include "rss/state/ResponseState.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace state
 */
namespace state {

using ResponseStateVector = std::vector<::rss::state::ResponseState>;

/*
 * \brief Event to support type within statecharts
 */
struct evRssResponseStateVector
{
  evRssResponseStateVector(ResponseStateVector const &responseStateVector)
    : responseStateVector(responseStateVector)
  {
  }

  ResponseStateVector const &data() const
  {
    return responseStateVector;
  }

  ResponseStateVector const &responseStateVector;
};

} // namespace state
} // namespace rss

/*!
 * @brief Conversion of event evRssResponseStateVector to std::string (for logging purposes)
 */
std::string toString(::rss::state::evRssResponseStateVector const &);
