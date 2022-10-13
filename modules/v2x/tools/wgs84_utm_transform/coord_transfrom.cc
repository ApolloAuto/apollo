/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "absl/strings/str_cat.h"

#include "modules/map/hdmap/adapter/coordinate_convert_tool.h"

using ::apollo::hdmap::adapter::CoordinateConvertTool;

int main(int argc, char *argv[]) {
  double x(0), y(0), z(0);
  bool flag(0);
  int zone_id(0);
  std::cout << "0|1 zone_id x y.   0 is WGS84 to UTM | 1 is UTM to WGS84"
            << std::endl;
  while (1) {
    std::cin >> flag >> zone_id >> x >> y;
    std::string wgs_84_str = "+proj=longlat +datum=WGS84 +no_defs";
    std::string utm_str =
        absl::StrCat("+proj=utm +zone=", std::to_string(zone_id),
                     " +ellps=WGS84 +datum=WGS84 +units=m +no_defs");
    std::string from_coordinate_, to_coordinate_;
    if (flag == 0) {
      from_coordinate_ = wgs_84_str;
      to_coordinate_ = utm_str;
    } else {
      from_coordinate_ = utm_str;
      to_coordinate_ = wgs_84_str;
    }

    double res_x(0), res_y(0), res_z(0);
    CoordinateConvertTool::GetInstance()->SetConvertParam(from_coordinate_,
                                                          to_coordinate_);
    CoordinateConvertTool::GetInstance()->CoordiateConvert(x, y, z, &res_x,
                                                           &res_y, &res_z);
    std::cout << "result: " << std::fixed << res_x << " " << std::fixed << res_y
              << std::endl;
  }
  return 0;
}
