/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 */

#pragma once

#include <map>

/**
 * @namespace apollo::third_party_perception::conversion_base
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {
namespace conversion_base {

std::map<std::int32_t, apollo::hdmap::LaneBoundaryType_Type>
    lane_conversion_map = {{0, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {1, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {2, apollo::hdmap::LaneBoundaryType::UNKNOWN},
                           {3, apollo::hdmap::LaneBoundaryType::CURB},
                           {4, apollo::hdmap::LaneBoundaryType::SOLID_YELLOW},
                           {5, apollo::hdmap::LaneBoundaryType::DOTTED_YELLOW},
                           {6, apollo::hdmap::LaneBoundaryType::UNKNOWN}};

}  // namespace conversion_base
}  // namespace third_party_perception
}  // namespace apollo
