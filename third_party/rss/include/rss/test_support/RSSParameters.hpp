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
