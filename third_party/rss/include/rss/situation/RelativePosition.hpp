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
 *
 */

#pragma once
#include <limits>

#include "rss/situation/Distance.hpp"
#include "rss/situation/LateralRelativePosition.hpp"
#include "rss/situation/LongitudinalRelativePosition.hpp"
/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace situation
 */
namespace situation {

struct RelativePosition
{
  ::rss::situation::LongitudinalRelativePosition longitudinalPosition{
    ::rss::situation::LongitudinalRelativePosition::Overlap};
  ::rss::situation::Distance longitudinalDistance{0.0};
  ::rss::situation::LateralRelativePosition lateralPosition{::rss::situation::LateralRelativePosition::Overlap};
  ::rss::situation::Distance lateralDistance{0.0};

  ::rss::situation::LongitudinalRelativePosition getLongitudinalPosition() const
  {
    return longitudinalPosition;
  }

  void setLongitudinalPosition(::rss::situation::LongitudinalRelativePosition const newVal)
  {
    longitudinalPosition = newVal;
  }

  ::rss::situation::LateralRelativePosition getLateralPosition() const
  {
    return lateralPosition;
  }

  void setLateralPosition(::rss::situation::LateralRelativePosition const newVal)
  {
    lateralPosition = newVal;
  }
};

} // namespace situation
} // namespace rss
