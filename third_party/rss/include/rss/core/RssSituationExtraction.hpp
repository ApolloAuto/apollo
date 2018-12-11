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
