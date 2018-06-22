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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_NETWORK_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_NETWORK_H_

namespace apollo {
namespace perception {

void GetObjectsGPU(int n, const float *loc_data, const float *obj_data,
                   const float *cls_data, const float *ori_data,
                   const float *dim_data, const float *lof_data,
                   const float *lor_data, const float *anchor_data, int width,
                   int height, int num_anchors, int num_classes,
                   float confidence_threshold, bool with_ori, bool with_dim,
                   bool with_lof, bool with_lor, float *res_box_data,
                   float *res_cls_data, int s_box_block_size);

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_COMMON_NETWORK_H_
