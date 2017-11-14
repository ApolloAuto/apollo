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
#include "modules/perception/obstacle/base/object_supplement.h"

namespace apollo {
namespace perception {

/**RadarSupplement implementation*/
RadarSupplement::RadarSupplement() {
}
RadarSupplement::~RadarSupplement() {
}
RadarSupplement::RadarSupplement(const RadarSupplement& rhs) {
    range = rhs.range;
    angle = rhs.angle;
    relative_radial_velocity = rhs.relative_radial_velocity;
    relative_tangential_velocity = rhs.relative_tangential_velocity;
    radial_velocity = rhs.radial_velocity;
}
RadarSupplement& RadarSupplement::operator = (const RadarSupplement& rhs) {
    range = rhs.range;
    angle = rhs.angle;
    relative_radial_velocity = rhs.relative_radial_velocity;
    relative_tangential_velocity = rhs.relative_tangential_velocity;
    radial_velocity = rhs.radial_velocity;
    return (*this);
}
void RadarSupplement::clone(const RadarSupplement& rhs) {
    range = rhs.range;
    angle = rhs.angle;
    relative_radial_velocity = rhs.relative_radial_velocity;
    relative_tangential_velocity = rhs.relative_tangential_velocity;
    radial_velocity = rhs.radial_velocity;
}

}  // namespace perception
}  // namespace apollo
