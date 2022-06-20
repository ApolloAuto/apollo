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


#include <vector>
//#include <cuda.h>
#include "/usr/local/cuda/include/cuda.h"
//#include <cuda_runtime_api.h>
#include "/usr/local/cuda/include/cuda_runtime_api.h"

#include "paddle/include/experimental/ext_all.h"
#include "iou3d_cpu.h"
#include "iou3d_nms.h"

std::vector<paddle::DataType> BoxesIouBevCpuInferDtype(paddle::DataType boxes_a_dtype,
													                             paddle::DataType boxes_b_dtype) {
  return {boxes_a_dtype};
}

std::vector<std::vector<int64_t>>
BoxesIouBevCpuInferShape(std::vector<int64_t> boxes_a_shape,
                         std::vector<int64_t> boxes_b_shape) {
  return {{boxes_a_shape[0], boxes_b_shape[0]}};
}

std::vector<paddle::DataType> NmsInferDtype(paddle::DataType boxes_dtype) {
  return {paddle::DataType::INT64, paddle::DataType::INT64};
}

std::vector<std::vector<int64_t>> NmsInferShape(std::vector<int64_t> boxes_shape) {
  return {{boxes_shape[0]}, {1}};
}

std::vector<paddle::DataType>
NmsNormalInferDtype(paddle::DataType boxes_dtype) {
  return {paddle::DataType::INT64, paddle::DataType::INT64};
}

std::vector<std::vector<int64_t>>
NmsNormalInferShape(std::vector<int64_t> boxes_shape) {
  return {{boxes_shape[0]}, {1}};
}

std::vector<paddle::DataType> BoxesIouBevGpuInferDtype(paddle::DataType boxes_a_dtype,
													                             paddle::DataType boxes_b_dtype) {
  return {boxes_a_dtype};
}

std::vector<std::vector<int64_t>>
BoxesIouBevGpuInferShape(std::vector<int64_t> boxes_a_shape,
                         std::vector<int64_t> boxes_b_shape) {
  return {{boxes_a_shape[0], boxes_b_shape[0]}};
}

std::vector<paddle::DataType> BoxesOverlapBevGpuInferDtype(paddle::DataType boxes_a_dtype,
													                                 paddle::DataType boxes_b_dtype) {
  return {boxes_a_dtype};
}

std::vector<std::vector<int64_t>>
BoxesOverlapBevGpuInferShape(std::vector<int64_t> boxes_a_shape,
                             std::vector<int64_t> boxes_b_shape) {
  return {{boxes_a_shape[0], boxes_b_shape[0]}};
}

PD_BUILD_OP(boxes_iou_bev_cpu)
    .Inputs({"boxes_a_tensor", " boxes_b_tensor"})
    .Outputs({"ans_iou_tensor"})
    .SetKernelFn(PD_KERNEL(boxes_iou_bev_cpu))
    .SetInferDtypeFn(PD_INFER_DTYPE(BoxesIouBevCpuInferDtype))
	  .SetInferShapeFn(PD_INFER_SHAPE(BoxesIouBevCpuInferShape));

PD_BUILD_OP(boxes_iou_bev_gpu)
    .Inputs({"boxes_a_tensor", " boxes_b_tensor"})
    .Outputs({"ans_iou_tensor"})
    .SetKernelFn(PD_KERNEL(boxes_iou_bev_gpu))
    .SetInferDtypeFn(PD_INFER_DTYPE(BoxesIouBevGpuInferDtype))
	  .SetInferShapeFn(PD_INFER_SHAPE(BoxesIouBevGpuInferShape));

PD_BUILD_OP(boxes_overlap_bev_gpu)
    .Inputs({"boxes_a", " boxes_b"})
    .Outputs({"ans_overlap"})
    .SetKernelFn(PD_KERNEL(boxes_overlap_bev_gpu))
    .SetInferDtypeFn(PD_INFER_DTYPE(BoxesOverlapBevGpuInferDtype))
	  .SetInferShapeFn(PD_INFER_SHAPE(BoxesOverlapBevGpuInferShape));

PD_BUILD_OP(nms_gpu)
    .Inputs({"boxes"})
    .Outputs({"keep", "num_to_keep"})
    .Attrs({"nms_overlap_thresh: float"})
    .SetKernelFn(PD_KERNEL(nms_gpu))
    .SetInferDtypeFn(PD_INFER_DTYPE(NmsInferDtype))
	  .SetInferShapeFn(PD_INFER_SHAPE(NmsInferShape));

PD_BUILD_OP(nms_normal_gpu)
    .Inputs({"boxes"})
    .Outputs({"keep", "num_to_keep"})
    .Attrs({"nms_overlap_thresh: float"})
    .SetInferShapeFn(PD_INFER_SHAPE(NmsNormalInferShape))
    .SetKernelFn(PD_KERNEL(nms_normal_gpu))
    .SetInferDtypeFn(PD_INFER_DTYPE(NmsNormalInferDtype));
