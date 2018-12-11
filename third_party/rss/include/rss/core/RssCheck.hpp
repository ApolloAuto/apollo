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

#include <memory>
#include "rss/world/AccelerationRestriction.hpp"
#include "rss/world/WorldModel.hpp"
//#include "rss/utils/utils.hpp"


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
