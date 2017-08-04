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

#include "modules/common/util/down_sampler.h"

#include "modules/common/math/vec2d.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(DownSamplerTest, DownSampleByAngle) {
    using ::apollo::common::math::Vec2d;

    std::vector<Vec2d> points;
    points.push_back(Vec2d(-405.778, -134.969));
    points.push_back(Vec2d(-403.919, -127.696));
    points.push_back(Vec2d(-400.635, -115.407));
    points.push_back(Vec2d(-397.997, -105.291));
    points.push_back(Vec2d(-395.801, -96.8637));
    points.push_back(Vec2d(-392.889, -86.1015));
    points.push_back(Vec2d(-388.054, -67.9935));
    points.push_back(Vec2d(-385.994, -60.1831));
    points.push_back(Vec2d(-378.213, -30.2776));
    points.push_back(Vec2d(-376.702, -24.5804));
    points.push_back(Vec2d(-373.825, -13.3855));
    points.push_back(Vec2d(-367.583, 10.4028));
    points.push_back(Vec2d(-363.025, 27.4212));

    std::vector<int> sampled_indices;

    EXPECT_TRUE(DownSampleByAngle(points, 0.1, &sampled_indices));
    EXPECT_EQ(2, sampled_indices.size());
    EXPECT_EQ(0, sampled_indices[0]);
    EXPECT_EQ(12, sampled_indices[1]);
}

}  // namespace util
}  // namespace common
}  // namespace apollo

