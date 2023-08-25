/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tools/common/visualizer.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>

#include "opencv2/opencv.hpp"

#include "cyber/common/log.h"
#include "modules/perception/tools/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

static const cv::Scalar kFaceColors[] = {
    cv::Scalar(255, 255, 255),  // black
    cv::Scalar(255, 0, 0),      // blue
    cv::Scalar(0, 255, 0),      // green
    cv::Scalar(0, 0, 255),      // red
};

bool CameraVisualization(onboard::CameraFrame* frame,
                         const std::string& file_name) {
  cv::Mat cv_img(frame->data_provider->src_height(),
                 frame->data_provider->src_width(), CV_8UC3,
                 cv::Scalar(0, 0, 0));

  RecoveryImage(frame, &cv_img);

  int obj_id = 0;
  for (auto obj : frame->detected_objects) {
    auto& supp = obj->camera_supplement;
    auto& box = supp.box;
    auto area_id = supp.area_id;

    cv::rectangle(
        cv_img,
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
        cv::Scalar(0, 0, 0), 8);
    float xmid = (box.xmin + box.xmax) / 2;
    ACHECK(area_id > 0 && area_id < 9);
    if (area_id & 1) {
      cv::rectangle(
          cv_img,
          cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
          cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
          kFaceColors[area_id / 2], 2);
    } else {
      auto& tl = supp.cut_off_ratios[2];
      auto& tr = supp.cut_off_ratios[3];
      auto&& left_ratio = supp.visible_ratios[(area_id / 2) % 4];
      auto w = box.xmax - box.xmin;
      auto x = box.xmin;
      auto tm = std::max(tl, tr);
      if (tm > 1e-2) {
        if (tl > tr) {
          xmid = (x - w * tl) + (w + w * tl) * left_ratio;
        } else if (tl < tr) {
          xmid = x + (w + w * tr) * left_ratio;
        }
      } else {
        xmid = x + w * left_ratio;
      }
      cv::rectangle(
          cv_img,
          cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
          cv::Point(static_cast<int>(xmid), static_cast<int>(box.ymax)),
          kFaceColors[(area_id / 2) % 4], 3);
      cv::rectangle(
          cv_img, cv::Point(static_cast<int>(xmid), static_cast<int>(box.ymin)),
          cv::Point(static_cast<int>(box.xmax), static_cast<int>(box.ymax)),
          kFaceColors[area_id / 2 - 1], 2);
    }
    fprintf(stderr,
            "obj-%02d: %.3f %.3f %.3f %.3f -- %.3f %.3f %.3f %.3f "
            "-- %.0f %.0f %.0f %d\n",
            obj_id, supp.visible_ratios[0], supp.visible_ratios[1],
            supp.visible_ratios[2], supp.visible_ratios[3],
            supp.cut_off_ratios[0], supp.cut_off_ratios[1],
            supp.cut_off_ratios[2], supp.cut_off_ratios[3], box.xmin, xmid,
            box.xmax, area_id);
    std::stringstream text;
    auto& name = base::kSubType2NameMap.at(obj->sub_type);
    text << name[0] << name[1] << name[2] << " - " << obj_id++;
    cv::putText(
        cv_img, text.str(),
        cv::Point(static_cast<int>(box.xmin), static_cast<int>(box.ymin)),
        cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);
  }

  cv::imwrite(file_name.c_str(), cv_img);
  return true;
}

bool TfVisualization(onboard::CameraFrame* frame,
                     const std::string& file_name) {
  return true;
}

bool LaneVisualization(onboard::CameraFrame* frame,
                       const std::string& file_name) {
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
