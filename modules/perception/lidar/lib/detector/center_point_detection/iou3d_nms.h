// Copyright (c) 2022 PaddlePaddle Authors. All Rights Reserved.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IOU3D_NMS_H
#define IOU3D_NMS_H

#include <vector>
//#include <cuda.h>
#include "/usr/local/cuda/include/cuda.h"
//#include <cuda_runtime_api.h>
#include "/usr/local/cuda/include/cuda_runtime_api.h"
#include "paddle/include/experimental/ext_all.h"

std::vector<paddle::Tensor> boxes_overlap_bev_gpu(const paddle::Tensor &boxes_a, const paddle::Tensor &boxes_b);
std::vector<paddle::Tensor> boxes_iou_bev_gpu(const paddle::Tensor & boxes_a_tensor, const paddle::Tensor & boxes_b_tensor);
std::vector<paddle::Tensor> nms_gpu(const paddle::Tensor &boxes, float nms_overlap_thresh);
std::vector<paddle::Tensor> nms_normal_gpu(const paddle::Tensor &boxes, float nms_overlap_thresh);

#endif
