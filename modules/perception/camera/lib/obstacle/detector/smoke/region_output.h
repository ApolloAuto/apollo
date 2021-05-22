/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/perception/base/blob.h"
#include "modules/perception/base/box.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/math_functions.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/proto/smoke.pb.h"

namespace apollo {
namespace perception {
namespace camera {

struct SmokeNormalizedBBox {
  float xmin = -1;
  float ymin = -1;
  float xmax = -1;
  float ymax = -1;
  int label = -1;
  float score = -1;
  float size = -1;
  bool mask = false;

  bool operator()(SmokeNormalizedBBox i, SmokeNormalizedBBox j) {
    return i.score < j.score;
  }
};

struct SmokeBBox3D {
  float h = -1;
  float w = -1;
  float l = -1;
  float alpha = -1;
};

struct SmokeAnchorBox {
  float w;
  float h;
};

struct SmokeNMSParam {
  float threshold;
  float inter_cls_nms_thresh;
  float inter_cls_conf_thresh;
  float sigma;
  std::string type = "BoxVote";
};

struct SmokeBlobs {
  std::shared_ptr<base::Blob<float>> det1_loc_blob;
};
struct SmokeMinDims {
  float min_2d_height = 0.0f;
  float min_3d_height = 0.0f;
  float min_3d_length = 0.0f;
  float min_3d_width = 0.0f;
};

int get_smoke_objects_gpu();

void get_smoke_objects_cpu(const SmokeBlobs &smoke_blobs,
                     const std::vector<base::ObjectSubType> &types,
                     const smoke::ModelParam &model_param,
                     float light_vis_conf_threshold,
                     float light_swt_conf_threshold,
                     base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                     std::vector<base::ObjectPtr> *objects,
                     int width, int height);

void recover_smoke_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects);

void filter_bbox(const SmokeMinDims &min_dims,
                 std::vector<base::ObjectPtr> *objects);

void fill_smoke_base(base::ObjectPtr obj, const float *bbox,
                     int width, int height);
void fill_smoke_bbox3d(bool with_bbox3d, base::ObjectPtr obj,
                       const float *bbox);
base::ObjectSubType get_smoke_object_subtype(int cls);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
