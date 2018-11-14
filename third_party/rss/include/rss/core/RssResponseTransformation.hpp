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

#include "rss/state/ResponseState.hpp"
#include "rss/world/AccelerationRestriction.hpp"
#include "rss/world/WorldModel.hpp"

/*!
 * @brief namespace rss
 */
namespace rss {

/*!
 * @brief namespace core
 */
namespace core {

/*!
 * @brief namespace RssResponseTransformation
 *
 * Namespace providing functions required to transform the proper response into restrictions
 * of the acceleration for the actuator control.
 */
namespace RssResponseTransformation {

/*!
 * @brief transformProperResponse
 *
 * Transform the proper response into restrictions of the acceleration for the actuator control.
 *
 * @param [in] worldModel - The current world model information.
 * @param [in] response - The proper overall response to be transformed.
 * @param [out] accelerationRestriction - The restrictions on the vehicle acceleration to become RSS safe.
 *
 * @return return true if the acceleration restrictions could be calculated, false otherwise.
 */
bool transformProperResponse(world::WorldModel const &worldModel,
                             state::ResponseState const &response,
                             world::AccelerationRestriction &accelerationRestriction) noexcept;

} // namespace RssResponseTransformation
} // namespace core
} // namespace rss
