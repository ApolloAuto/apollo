/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "gflags/gflags.h"

#include "cyber/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/quaternion.h"

DEFINE_double(qx, 0, "quaternion x");
DEFINE_double(qy, 0, "quaternion y");
DEFINE_double(qz, 0, "quaternion z");
DEFINE_double(qw, 0, "quaternion w");

int main(int32_t argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::common::math::EulerAnglesZXY<double> euler(FLAGS_qw, FLAGS_qx,
                                                     FLAGS_qy, FLAGS_qz);
  AINFO << "roll: " << euler.roll() << " pitch:" << euler.pitch()
        << " yaw:" << euler.yaw();
  AINFO << "heading: "
        << apollo::common::math::QuaternionToHeading(FLAGS_qw, FLAGS_qx,
                                                     FLAGS_qy, FLAGS_qz)
        << " heading degree: "
        << 180.0 / M_PI *
               apollo::common::math::QuaternionToHeading(FLAGS_qw, FLAGS_qx,
                                                         FLAGS_qy, FLAGS_qz);
}
