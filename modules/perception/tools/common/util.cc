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

#include "modules/perception/tools/common/util.h"

#include <fstream>

#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

bool FillImage(onboard::CameraFrame* frame, const std::string& file_name) {
  // Read image from file_name
  cv::Mat image = cv::imread(file_name);
  if (image.empty()) {
    AERROR << "Read image failed! " << file_name;
    return false;
  }

  // Fill image to frame->data_provider
  bool res = frame->data_provider->FillImageData(image.rows, image.cols,
                                                 image.data, "bgr8");
  if (!res) {
    AERROR << "Fill image failed! " << file_name;
    return false;
  }
  return true;
}

bool RecoveryImage(onboard::CameraFrame* frame, cv::Mat* cv_img) {
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  image_options.do_crop = false;
  base::Image8U image(frame->data_provider->src_height(),
                      frame->data_provider->src_width(), base::Color::RGB);
  frame->data_provider->GetImage(image_options, &image);

  memcpy(cv_img->data, image.cpu_data(), image.total() * sizeof(uint8_t));
  return true;
}

bool GetFileListFromPath(const std::string& file_path,
                         std::vector<std::string>* file_list) {
  // todo(zero): need complete
  return true;
}

bool GetFileListFromFile(const std::string& file_name,
                         std::vector<std::string>* file_list) {
  std::ifstream in(file_name.c_str());

  if (!in) {
    AERROR << "Failed to open file: " << file_name;
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) {
      file_list->push_back(line);
    }
  }

  in.close();
  return true;
}

bool SaveCameraDetectionResult(onboard::CameraFrame* frame,
                               const std::string& file_name) {
  FILE* fp = fopen(file_name.c_str(), "w");
  if (fp == nullptr) {
    AERROR << "Failed to open result file: " << file_name;
    return false;
  }

  for (auto obj : frame->detected_objects) {
    auto& supp = obj->camera_supplement;
    fprintf(fp,
            "%s 0 0 %6.3f %8.2f %8.2f %8.2f %8.2f %6.3f %6.3f %6.3f "
            "%6.3f %6.3f %6.3f %6.3f %6.3f "
            "%4d %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
            base::kSubType2NameMap.at(obj->sub_type).c_str(), supp.alpha,
            supp.box.xmin, supp.box.ymin, supp.box.xmax, supp.box.ymax,
            obj->size[2], obj->size[1], obj->size[0], obj->center[0],
            obj->center[1] + obj->size[2] * .5, obj->center[2],
            supp.alpha + atan2(obj->center[0], obj->center[2]),
            obj->type_probs[static_cast<int>(obj->type)], supp.area_id,
            supp.visible_ratios[0], supp.visible_ratios[1],
            supp.visible_ratios[2], supp.visible_ratios[3],
            supp.cut_off_ratios[0], supp.cut_off_ratios[1],
            supp.cut_off_ratios[2], supp.cut_off_ratios[3]);
  }

  fclose(fp);
  return true;
}

bool SaveTfDetectionResult(onboard::CameraFrame* frame,
                           const std::string& file_name) {
  return true;
}

bool SaveLaneDetectionResult(onboard::CameraFrame* frame,
                             const std::string& file_name) {
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
