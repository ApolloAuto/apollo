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

#include "rss/situation/SituationVector.hpp"
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
 * @brief namespace RssSituationExtraction
 *
 * Namespace providing functions required for the extraction of the RSS situations from the RSS world model.
 */
namespace RssSituationExtraction {

/**
 * @brief Extract the RSS situation of the ego vehicle and the object to be checked.
 *
 * @param [in] egoVehicle - the information on the ego vehicle object
 * @param [in] currentScene - the information on the object to be checked and the according lane information
 * @param [out] situation - the situation to be analyzed with RSS
 *
 * @return true if the situation could be created, false if there was an error during the operation.
 */
bool extractSituation(world::Object const &egoVehicle,
                      world::Scene const &currentScene,
                      situation::Situation &situation) noexcept;

/*
 * @brief Extract all RSS situations to be checked from the world model.
 *
 * @param [in] worldModel - the current world model information
 * @param [out] situationVector - the vector of situations to be analyzed with RSS
 *
 * @return true if the situations could be created, false if there was an error during the operation.
 */
bool extractSituations(world::WorldModel const &worldModel, situation::SituationVector &situationVector) noexcept;

} // namespace RssSituationExtraction
} // namespace core
} // namespace rss
