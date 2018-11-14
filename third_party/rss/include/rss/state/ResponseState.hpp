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
#include "rss/situation/SituationId.hpp"
#include "rss/state/LateralRssState.hpp"
#include "rss/state/LongitudinalRssState.hpp"
#include "rss/time/TimeIndex.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace state
 */
namespace state {

struct ResponseState
{
  ::rss::time::TimeIndex timeIndex;
  ::rss::situation::SituationId situationId{0u};
  ::rss::state::LongitudinalRssState longitudinalState;
  ::rss::state::LateralRssState lateralStateRight;
  ::rss::state::LateralRssState lateralStateLeft;
};

/*
 * \brief Event to support type within statecharts
 */
struct evRssResponseState
{
  evRssResponseState(ResponseState const &responseState)
    : responseState(responseState)
  {
  }

  ResponseState const &data() const
  {
    return responseState;
  }

  ResponseState const &responseState;
};

} // namespace state
} // namespace rss

/*!
 * @brief Conversion of event evRssResponseState to std::string (for logging purposes)
 */
std::string toString(::rss::state::evRssResponseState const &);
