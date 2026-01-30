/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <google/protobuf/repeated_field.h>
#include <google/protobuf/text_format.h>

#include "Eigen/Core"
#include "unsupported/Eigen/Splines"

#include "modules/control/control_component/controller_task_base/common/interpolation_1d.h"

namespace apollo {
namespace control {

class InterpolationPlus1D : public Interpolation1D {
public:
    static double interpolation_1d(
            const double& value,
            const google::protobuf::RepeatedField<double>& input_vector,
            const google::protobuf::RepeatedField<double>& output_vector);
};

}  // namespace control
}  // namespace apollo
