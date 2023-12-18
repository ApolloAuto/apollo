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
#include "modules/perception/common/camera/common/util.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

bool Equal(double x, double target, double eps) {
  return std::abs(x - target) < eps;
}
bool Equal(float x, float target, float eps) {
  return std::abs(x - target) < eps;
}

bool LoadAnchors(const std::string &path, std::vector<float> *anchors) {
  int num_anchors = 0;
  std::ifstream ifs(path, std::ifstream::in);
  ifs >> num_anchors;
  if (!ifs.good()) {
    AERROR << "Failed to get number of anchors!";
    return false;
  }
  (*anchors).resize(num_anchors * 2);
  for (int i = 0; i < num_anchors; ++i) {
    ifs >> (*anchors)[i * 2] >> (*anchors)[i * 2 + 1];
    if (!ifs.good()) {
      AERROR << "Failed to load the " << i << "-th anchor!";
      return false;
    }
  }
  ifs.close();
  return true;
}

bool LoadTypes(const std::string &path,
               std::vector<base::ObjectSubType> *types) {
  std::ifstream ifs(path, std::ifstream::in);
  if (!ifs.good()) {
    AERROR << "Type_list not found: " << path;
    return false;
  }
  std::string type;
  AINFO << "Supported types: ";
  while (ifs >> type) {
    if (base::kName2SubTypeMap.find(type) == base::kName2SubTypeMap.end()) {
      AERROR << "Invalid type: " << type;
      return false;
    }
    (*types).push_back(base::kName2SubTypeMap.at(type));
    AINFO << "\t\t" << type;
  }
  AINFO << "\t\t" << (*types).size() << " in total.";
  ifs.close();
  return true;
}
bool LoadExpand(const std::string &path, std::vector<float> *expands) {
  std::ifstream ifs(path, std::ifstream::in);
  if (!ifs.good()) {
    AERROR << "expand_list not found: " << path;
    return false;
  }
  float expand;
  AINFO << "Expand nums: ";
  while (ifs >> expand) {
    expands->push_back(expand);
    AINFO << "\t\t" << expand;
  }
  ifs.close();
  return true;
}
bool ResizeCPU(const base::Blob<uint8_t> &src_blob,
               std::shared_ptr<base::Blob<float>> dst_blob, int stepwidth,
               int start_axis) {
  int width = dst_blob->shape(2);
  int height = dst_blob->shape(1);
  int channel = dst_blob->shape(3);
  int origin_channel = src_blob.shape(3);
  int origin_height = src_blob.shape(1);
  int origin_width = src_blob.shape(2);
  if (origin_channel != dst_blob->shape(3)) {
    AERROR << "channel should be the same after resize.";
    return false;
  }
  float fx = static_cast<float>(origin_width) / static_cast<float>(width);
  float fy = static_cast<float>(origin_height) / static_cast<float>(height);
  auto src = src_blob.cpu_data();
  auto dst = dst_blob->mutable_cpu_data();
  for (int dst_y = 0; dst_y < height; dst_y++) {
    for (int dst_x = 0; dst_x < width; dst_x++) {
      float src_x = (static_cast<float>(dst_x) + 0.5f) * fx - 0.5f;
      float src_y = (static_cast<float>(dst_y) + 0.5f) * fy - 0.5f;
      const int x1 = static_cast<int>(src_x + 0.5);
      const int y1 = static_cast<int>(src_y + 0.5);
      const int x1_read = std::max(x1, 0);
      const int y1_read = std::max(y1, 0);
      const int x2 = x1 + 1;
      const int y2 = y1 + 1;
      const int x2_read = std::min(x2, width - 1);
      const int y2_read = std::min(y2, height - 1);
      int src_reg = 0;
      for (int c = 0; c < channel; c++) {
        float out = 0.0f;

        int idx11 = (y1_read * stepwidth + x1_read) * channel;
        src_reg = src[idx11 + c];
        out = out + (static_cast<float>(x2) - src_x) *
                        (static_cast<float>(y2) - src_y) *
                        static_cast<float>(src_reg);
        int idx12 = (y1_read * stepwidth + x2_read) * channel;
        src_reg = src[idx12 + c];
        out = out + static_cast<float>(src_reg) *
                        (src_x - static_cast<float>(x1)) *
                        (static_cast<float>(y2) - src_y);

        int idx21 = (y2_read * stepwidth + x1_read) * channel;
        src_reg = src[idx21 + c];
        out = out + static_cast<float>(src_reg) *
                        (static_cast<float>(x2) - src_x) *
                        (src_y - static_cast<float>(y1));

        int idx22 = (y2_read * stepwidth + x2_read) * channel;
        src_reg = src[idx22 + c];
        out = out + static_cast<float>(src_reg) *
                        (src_x - static_cast<float>(x1)) *
                        (src_y - static_cast<float>(y1));
        if (out < 0) {
          out = 0;
        }
        if (out > 255) {
          out = 255;
        }
        int dst_idx = (dst_y * width + dst_x) * channel + c;
        //   printf("%f %d %d %d %d\n",out,x1,y1,x2,y2);
        dst[dst_idx] = out;
      }
    }
  }
  return true;
}

void FillObjectPolygonFromBBox3D(base::Object *object_ptr) {
  if (!object_ptr) {
    return;
  }
  const double length = object_ptr->size(0);
  const double width = object_ptr->size(1);
  //  const double height = object_ptr->size(2);
  double x1 = length / 2;
  double x2 = -x1;
  double y1 = width / 2;
  double y2 = -y1;
  double len = sqrt(object_ptr->direction[0] * object_ptr->direction[0] +
                    object_ptr->direction[1] * object_ptr->direction[1]);
  double cos_theta = object_ptr->direction[0] / len;
  double sin_theta = -object_ptr->direction[1] / len;

  object_ptr->polygon.resize(4);
  object_ptr->polygon[0].x =
      x1 * cos_theta + y1 * sin_theta + object_ptr->center[0];
  object_ptr->polygon[0].y =
      y1 * cos_theta - x1 * sin_theta + object_ptr->center[1];
  object_ptr->polygon[0].z = 0.0;

  object_ptr->polygon[1].x =
      x1 * cos_theta + y2 * sin_theta + object_ptr->center[0];
  object_ptr->polygon[1].y =
      y2 * cos_theta - x1 * sin_theta + object_ptr->center[1];
  object_ptr->polygon[1].z = 0.0;

  object_ptr->polygon[2].x =
      x2 * cos_theta + y2 * sin_theta + object_ptr->center[0];
  object_ptr->polygon[2].y =
      y2 * cos_theta - x2 * sin_theta + object_ptr->center[1];
  object_ptr->polygon[2].z = 0.0;

  object_ptr->polygon[3].x =
      x2 * cos_theta + y1 * sin_theta + object_ptr->center[0];
  object_ptr->polygon[3].y =
      y1 * cos_theta - x2 * sin_theta + object_ptr->center[1];
  object_ptr->polygon[3].z = 0.0;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
