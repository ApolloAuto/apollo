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
#include "modules/control/control_task_base_extend/common/interpolation_plus_1d.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace apollo {
namespace control {

double InterpolationPlus1D::interpolation_1d(
        const double& value,
        const google::protobuf::RepeatedField<double>& input_vector,
        const google::protobuf::RepeatedField<double>& output_vector) {
    int input_size = input_vector.size();
    int output_size = output_vector.size();

    ADEBUG << "input_size is " << input_size << ", output_size is " << output_size;
    if (input_size < 1 || output_size < 1) {
        AERROR << "1d interpolation: vector size is 0";
        return 1.0;
    } else if (input_size != output_size) {
        AERROR << "1d interpolation: vector size is wrong"
               << ", input_size: " << input_size << ", output_size: " << output_size;
        return 1.0;
    }

    if (value <= input_vector.Get(0)) {
        ADEBUG << "return the output_vector.Get(0)";
        return output_vector.Get(0);
    } else if (value >= input_vector.Get(input_size - 1)) {
        ADEBUG << "return the output_vector.Get(input_size - 1)";
        return output_vector.Get(input_size - 1);
    }

    for (int i = 1; i < input_size; ++i) {
        if (value >= input_vector.Get(i - 1) && value < input_vector.Get(i)) {
            ADEBUG << "return the interpolation_1d";
            return output_vector.Get(i - 1)
                    + (output_vector.Get(i) - output_vector.Get(i - 1)) * (value - input_vector.Get(i - 1))
                    / (input_vector.Get(i) - input_vector.Get(i - 1));
        }
    }

    AERROR << "1d interpolation: something is wrong";
    return 1.0;
}

}  // namespace control
}  // namespace apollo
