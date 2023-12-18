/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/common/sim_control_manager/core/sim_control_base.h"

namespace apollo {
namespace dreamview {
using apollo::common::Point3D;
using apollo::common::Quaternion;
using apollo::common::math::HeadingToQuaternion;
using apollo::common::math::InverseQuaternionRotate;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::QuaternionToHeading;

void SimControlBase::TransformToVRF(const Point3D& point_mrf,
                                    const Quaternion& orientation,
                                    Point3D* point_vrf) {
  Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
  auto v_vrf = InverseQuaternionRotate(orientation, v_mrf);
  point_vrf->set_x(v_vrf.x());
  point_vrf->set_y(v_vrf.y());
  point_vrf->set_z(v_vrf.z());
}

}  // namespace dreamview
}  // namespace apollo
