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

#include "rss/situation/Acceleration.hpp"
#include "rss/time/Duration.hpp"

/*!
 * @brief namespace rss
 */
namespace rss {
/*!
 * @brief namespace for RSS situation coordinate system datatypes and operations
 */
namespace situation {

// @TODO: remove this file when constants are replaced by parameters

const time::Duration cResponseTimeEgoVehicle = 1;    /*!< Response time of the ego vehicle in seconds. */
const time::Duration cResponseTimeOtherVehicles = 2; /*!< Response time of non-ego vehicles in seconds. */

const Acceleration cMaximumLongitudinalAcceleration = 3.5;
const Acceleration cMinimumLongitudinalBrakingDeceleleration = 4;
const Acceleration cMaximumLongitudinalBrakingDeceleleration = 8;
const Acceleration cMinimumLongitudinalBrakingDecelelerationCorrect = 3;

const Acceleration cMaximumLateralAcceleration = 0.2;
const Acceleration cMinimumLateralBrakingDeceleleration = 0.8;

} // namespace situation
} // namespace rss
