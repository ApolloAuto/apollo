/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include <gtest/gtest.h>
#include "rss/core/RssResponseTransformation.hpp"

namespace rss {
namespace core {

using state::LateralResponse;
using state::LongitudinalResponse;

TEST(RssResponseTransformationTests, invalidTimeStamp) {
  ::rss::world::WorldModel worldModel;
  ::rss::state::ResponseState responseState;
  ::rss::world::AccelerationRestriction accelerationRestriction;

  worldModel.timeIndex = 1u;
  responseState.timeIndex = 0u;

  ASSERT_FALSE(::rss::core::RssResponseTransformation::transformProperResponse(
    worldModel, responseState, accelerationRestriction));
}

}   //  namespace core
}   //  namespace rss
