// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (c) 2018 Intel Corporation
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//	  and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//    IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//    INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//    OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//    POSSIBILITY OF SUCH DAMAGE.
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
