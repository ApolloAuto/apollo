/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <vector>

#include "modules/perception/common/base/blob.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/base/object.h"

#include "modules/perception/camera_detection_single_stage/detector/smoke/proto/model_param.pb.h"

namespace apollo {
namespace perception {
namespace camera {

void GetObjectsCpu(const std::shared_ptr<base::Blob<float>> &output_blob,
                   const std::vector<base::ObjectSubType> &types,
                   const smoke::ModelParam &model_param,
                   std::vector<base::ObjectPtr> *objects,
                   int width, int height);

void RecoverBBox(int roi_w, int roi_h, int offset_y,
                 std::vector<base::ObjectPtr> *objects);

void FilterByMinDims(const smoke::MinDims &min_dims,
                     std::vector<base::ObjectPtr> *objects);

void FillBBox(base::ObjectPtr obj, const float *bbox, int width, int height);
void FillBBox3d(base::ObjectPtr obj, const float *bbox);

base::ObjectSubType GetSubtype(
    int cls, const std::vector<base::ObjectSubType> &types);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
