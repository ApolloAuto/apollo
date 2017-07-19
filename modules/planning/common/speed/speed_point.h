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

#ifndef MODULES_PLANNING_COMMON_SPEED_SPEED_POINT_H
#define MODULES_PLANNING_COMMON_SPEED_SPEED_POINT_H

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

class SpeedPoint : public STPoint {
public:
    SpeedPoint() = default;
    SpeedPoint(const double s,
        const double t,
        const double v,
        const double a,
        const double j);
    SpeedPoint(const STPoint& st_point, const double v, const double a, const double j);
    void set_v(const double v);
    void set_a(const double a);
    void set_j(const double j);

    double v() const;
    double a() const;
    double j() const;

    std::string DebugString() const;
    static SpeedPoint interpolate(const SpeedPoint& left, const SpeedPoint& right,
        const double weight);
private:
    double v_ = 0.0;
    double a_ = 0.0;
    double j_ = 0.0;
};

} // namespace planning
} // namespace apollo

#endif // MODULES_PLANNING_COMMON_SPEED_SPEED_POINT_H

