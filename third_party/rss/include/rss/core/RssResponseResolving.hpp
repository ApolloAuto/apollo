// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// INTEL CONFIDENTIAL
//
// Copyright (c) 2018 Intel Corporation
//
// This software and the related documents are Intel copyrighted materials, and
// your use of them is governed by the express license under which they were
// provided to you (License). Unless the License provides otherwise, you may not
// use, modify, copy, publish, distribute, disclose or transmit this software or
// the related documents without Intel's prior written permission.
//
// This software and the related documents are provided as is, with no express or
// implied warranties, other than those that are expressly stated in the License.
//
// ----------------- END LICENSE BLOCK -----------------------------------
/**
 * @file
 */

#pragma once

#include <map>
#include "rss/state/ResponseStateVector.hpp"

/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief namespace core
 */
namespace core {

/**
 * @brief RssResponseResolving
 *
 * Class to resolve the responseStateVector of the different situation specific responses
 * into a single responseState. This class tracks the RSS response state of every
 * situation id over time and especially stores the respective response state before
 * the blame time. This requires that the id of a RSS situation remains constant over
 * time in case it refers to the same object; otherwise tracking over time will fail.
 */
class RssResponseResolving
{
public:
  /**
   * @brief constructor
   */
  explicit RssResponseResolving();

  /**
   * @brief Calculate the proper response out of the current responses
   *
   * @param[in]  currentResponseState all the response states gathered for the current situations
   * @param[out] responseState the proper overall response state
   *
   * @return true if response could be calculated, false otherwise
   * If false is returned the internal state has not been updated
   */
  bool provideProperResponse(state::ResponseStateVector const &currentResponseStates,
                             state::ResponseState &responseState) noexcept;

private:
  struct RssState
  {
    bool longitudinalSafe{false};
    bool lateralSafe{false};
  };

  /**
   * @brief typedef for the mapping of object id to the corresponding RssState before the blame time
   */
  typedef std::map<situation::SituationId, RssState> RssStateBeforeBlameTimeMap;

  /**
   * @brief the state of all responses before the blame time of each response
   *
   * Needs to be stored to check which is the response that changed and required to solve an unclear situation
   */
  RssStateBeforeBlameTimeMap mStatesBeforeBlameTime;
};

} // namespace core
} // namespace rss
