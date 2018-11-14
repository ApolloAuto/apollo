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
#include "rss/world/AccelerationRestriction.hpp"
#include "rss/world/WorldModel.hpp"
#include "rss/utils/utils.hpp"


/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief namespace core
 */
namespace core {

class RssResponseResolving;
class RssSituationChecking;

/**
 * @brief RssCheck
 *
 * Class providing the functionality of the RSS check sequence at once with the RSS world model as input and
 * restrictions of the acceleration for the actuator control as output. This class internally makes use of the
 * RssSituationExtraction, RssSituationChecking, RssResponseResolving and RssResponseTransformation functionality.
 */
class RssCheck
{
public:
  /**
   * @brief constructor
   */
  explicit RssCheck();

  ~RssCheck();

  /**
   * @brief calculateAccelerationRestriction
   *
   * @param [in] worldModel - the current world model information
   * \param [out] accelerationRestriction - The restrictions on the vehicle acceleration to become RSS safe.
   *
   * @return return true if the acceleration restrictions could be calculated, false otherwise.
   */
  bool calculateAccelerationRestriction(world::WorldModel const &worldModel,
                                        world::AccelerationRestriction &accelerationRestriction) noexcept;

private:
  std::unique_ptr<RssResponseResolving> mResponseResolving;
  std::unique_ptr<RssSituationChecking> mSituationChecking;
};

} // namespace core
} // namespace rss
