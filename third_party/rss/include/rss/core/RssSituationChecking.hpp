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

#include <memory>
#include "rss/situation/SituationVector.hpp"
#include "rss/state/ResponseStateVector.hpp"

/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief Forward declaration
 */
namespace situation {
class RssIntersectionChecker;
}

/*!
 * @brief namespace core
 */
namespace core {

/*!
 * @brief class RssSituationChecking
 *
 * class providing functions required for the RSS checks of the situation.
 */
class RssSituationChecking
{
public:
  /*!
   * @brief constructor
   */
  explicit RssSituationChecking();

  /*!
   * @brief destructor
   */
  ~RssSituationChecking();

  /*!
   * @brief Check if the current situation is safe.
   *
   * @param[in] situation      the Situation that should be analyzed
   * @param[out] responseState the response state for the current situation
   *
   * @return true if situation could be analyzed, false if there was an error during evaluation
   */
  bool checkSituation(situation::Situation const &situation, state::ResponseState &responseState) noexcept;

  /*!
   * @brief Checks if the current situations are safe.
   *
   * @param [in] situationVector the vector of situations that should be analyzed
   * @param[out] responseVector the vector of response states for the current situations
   *
   * @return true if the situations could be analyzed, false if an error occurred during evaluation.
   */
  bool checkSituations(situation::SituationVector const &situationVector,
                       state::ResponseStateVector &responseStateVector) noexcept;

private:
  std::unique_ptr<rss::situation::RssIntersectionChecker> mIntersectionChecker;
};
} // namespace core
} // namespace rss
