/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/test/camera_lib_calibrator_laneline_lane_io.h"

#include "absl/strings/str_split.h"

namespace apollo {
namespace perception {
namespace camera {

bool ParseOneLaneLine(const std::string &s, LaneLine *lane_line) {
  assert(lane_line != nullptr);
  int len = static_cast<int>(s.size());
  if (len == 0) {
    return false;
  }
  lane_line->lane_point.clear();
  std::stringstream ss_temp;
  for (int i = 0; i < len; ++i) {
    if (s[i] == '(') {
      std::string x;
      std::string y;
      int j = i + 1;
      while (j < len && s[j] != ' ') {
        x += s[j++];
      }
      ++j;
      while (j < len && s[j] != ')') {
        y += s[j++];
      }
      Eigen::Vector2f pt;
      ss_temp << x;
      ss_temp >> pt(0);
      assert(!ss_temp.fail());
      ss_temp.clear();
      ss_temp << y;
      ss_temp >> pt(1);
      assert(!ss_temp.fail());
      ss_temp.clear();
      lane_line->lane_point.push_back(pt);
      i = j + 1;
    }
  }
  std::sort(lane_line->lane_point.begin(), lane_line->lane_point.end(),
            CmpLanePointY());
  return true;
}

bool LoadLaneDet(const std::string &filename, EgoLane *ego_lane) {
  std::ifstream fin;
  fin.open(filename.c_str());
  if (!fin) {
    std::cerr << "Fail to open the lane det file: " << filename << std::endl;
    return false;
  }
  int line_count = 0;
  std::string line;
  while (getline(fin, line)) {
    switch (line_count) {
      case (0):
      case (2):
        break;
      case (1):
        if (!ParseOneLaneLine(line, &ego_lane->left_line)) {
          fin.close();
          return false;
        }
        break;
      case (3):
        if (!ParseOneLaneLine(line, &ego_lane->right_line)) {
          fin.close();
          return false;
        }
        break;
      default:
        fin.close();
        return false;
    }
    ++line_count;
  }
  fin.close();
  return true;
}

// bool LoadLaneDets(const std::string path, const std::string suffix,
//                   std::vector<std::string> *fnames,  // named with time-stamp
//                   std::vector<EgoLane> *ego_lanes) {
//   fnames->clear();
//   ego_lanes->clear();
//   adu::perception::obstacle::load_filename(path, suffix, fnames);
//   int nr_frames = fnames->size();
//   for (int i = 0; i < nr_frames; ++i) {
//     EgoLane ego_lane;
//     if (!LoadLaneDet(fnames->data()[i], &ego_lane)) {
//       return false;
//     }
//     ego_lanes->push_back(ego_lane);
//   }
//   return true;
// }

bool LoadCamera2WorldTfs(const std::string &filename,
                         std::vector<std::string> *frame_list,
                         std::vector<double> *time_stamps,
                         std::vector<Eigen::Matrix4d> *camera2world) {
  frame_list->clear();
  camera2world->clear();
  std::ifstream fin;
  fin.open(filename.c_str());
  if (!fin) {
    std::cerr << "Fail to open the camera2world file: " << filename
              << std::endl;
    return false;
  }
  std::stringstream ss_temp;
  std::string line;
  const int kLength = 18;  // 2 info items + 4 * 4 transform
  const int kShift = 2;
  while (getline(fin, line)) {
    const std::vector<std::string> tf_info = absl::StrSplit(line, '\t');
    assert(tf_info.size() == kLength);

    frame_list->push_back(tf_info[0]);

    double time_stamp = 0.0;
    ss_temp << tf_info[1];
    ss_temp >> time_stamp;
    ss_temp.clear();
    // std::cout << time_stamp << std::endl;
    time_stamps->push_back(time_stamp);

    Eigen::Matrix4d tf;
    for (int i = kShift; i < kLength; ++i) {
      int j = i - kShift;
      int r = j / 4;
      int c = j - r * 4;
      float val = 0.0f;
      ss_temp << tf_info[i];
      ss_temp >> val;
      assert(!ss_temp.fail());
      ss_temp.clear();
      tf(r, c) = val;
      // std::cout << val << std::endl;
    }
    camera2world->push_back(tf);
  }
  fin.close();
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
