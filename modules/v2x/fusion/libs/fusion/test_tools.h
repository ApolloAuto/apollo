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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "modules/v2x/fusion/libs/common/v2x_object.h"

namespace apollo {
namespace v2x {
namespace ft {

bool LoadData(const std::string& file, std::vector<base::Object>* objects,
              const std::string& frame_id) {
  std::fstream fin;
  fin.open(file.c_str(), std::ios::in);
  if (!fin) {
    return false;
  }
  std::string line;
  while (getline(fin, line)) {
    std::istringstream iss(line);
    float sub_type_probs, x, y, z, xx, xy, xz, yx, yy, yz, zx, zy, zz;
    Eigen::Vector3d pos;
    Eigen::Matrix3d var;
    int type, sub_type;
    iss >> type >> sub_type >> sub_type_probs >> x >> y >> z >> xx >> xy >>
        xz >> yx >> yy >> yz >> zx >> zy >> zz;
    pos << x, y, z;
    var << xx, xy, xz, yx, yy, yz, zx, zy, zz;
    base::Object obj;
    obj.type = base::ObjectType::VEHICLE;
    Eigen::Vector3d car_size, bus_size, van_size;
    Eigen::Matrix3d id_var;
    car_size << 4.2, 2.0, 1.8;
    bus_size << 12, 2.2, 3;
    van_size << 4.5, 2.1, 2;
    id_var << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    switch (sub_type) {
      case 3:
        obj.sub_type = base::ObjectSubType::CAR;
        obj.size.Set(car_size, id_var);
        break;
      case 4:
        obj.sub_type = base::ObjectSubType::VAN;
        obj.size.Set(van_size, id_var);
        break;
      case 5:
        obj.sub_type = base::ObjectSubType::BUS;
        obj.size.Set(bus_size, id_var);
        break;
      default:
        break;
    }
    obj.sensor_type = base::SensorType::MONOCULAR_CAMERA;
    obj.frame_id = frame_id;
    obj.position.Set(pos, var);
    obj.theta.Set(0, 1);
    obj.type_probs.push_back(0.9f);
    obj.sub_type_probs.push_back(sub_type_probs);
    objects->push_back(obj);
  }
  return true;
}

}  // namespace ft
}  // namespace v2x
}  // namespace apollo
