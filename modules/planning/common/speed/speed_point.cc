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
 */

#include "modules/planning/common/speed/speed_point.h"

#include <sstream>

namespace apollo {
namespace planning {

SpeedPoint::SpeedPoint(const STPoint& st_point, const double v, const double a, const double j) :
    STPoint(st_point), v_(v), a_(a), j_(j) {
}

SpeedPoint::SpeedPoint(
    const double s,
    const double t,
    const double v,
    const double a,
    const double j) : STPoint(s, t), v_(v), a_(a), j_(j) {
}

void SpeedPoint::set_v(const double v) {
    v_ = v;
}

void SpeedPoint::set_a(const double a) {
    a_ = a;
}

void SpeedPoint::set_j(const double j) {
    j_ = j;
}

double SpeedPoint::v() const {
    return v_;
}

double SpeedPoint::a() const {
    return a_;
}

double SpeedPoint::j() const {
    return j_;
}

SpeedPoint SpeedPoint::interpolate(const SpeedPoint& left, const SpeedPoint& right,
        const double weight) {
    double s = (1 - weight) * left.s() + weight * right.s();
    double t = (1 - weight) * left.t() + weight * right.t();
    double v = (1 - weight) * left.v() + weight * right.v();
    double a = (1 - weight) * left.a() + weight * right.a();
    double j = (1 - weight) * left.j() + weight * right.j();
    return SpeedPoint(s, t, v, a, j);
}

std::string SpeedPoint::DebugString() const {
    return "";
}

} // namespace planning
} // namespace apollo

