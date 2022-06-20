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

/*
3D IoU Calculation and Rotated NMS(modified from 2D NMS written by others)
Written by Shaoshuai Shi
All Rights Reserved 2019-2020.
*/

#include <vector>
//#include <cuda.h>
#include "/usr/local/cuda/include/cuda.h"
//#include <cuda_runtime_api.h>
#include "/usr/local/cuda/include/cuda_runtime_api.h"
#include "iou3d_nms.h"


#define DIVUP(m,n) ((m) / (n) + ((m) % (n) > 0))

const int THREADS_PER_BLOCK_NMS = sizeof(int64_t) * 8;


void BoxesOverlapLauncher(const cudaStream_t &stream, const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_overlap);
void BoxesIouBevLauncher(const cudaStream_t &stream, const int num_a, const float *boxes_a, const int num_b, const float *boxes_b, float *ans_iou);
void NmsLauncher(const cudaStream_t &stream, const float *boxes, int64_t * mask, int boxes_num, float nms_overlap_thresh);
void NmsNormalLauncher(const cudaStream_t &stream, const float *boxes, int64_t * mask, int boxes_num, float nms_overlap_thresh);

std::vector<paddle::Tensor>  boxes_overlap_bev_gpu(const paddle::Tensor &boxes_a, const paddle::Tensor &boxes_b){
    // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
    // params ans_overlap: (N, M)
    int num_a = boxes_a.shape()[0];
    int num_b = boxes_b.shape()[0];

    const float * boxes_a_data = boxes_a.data<float>();
    const float * boxes_b_data = boxes_b.data<float>();
    auto ans_overlap = paddle::Tensor(paddle::PlaceType::kGPU, {num_a, num_b});
    float *ans_overlap_data = ans_overlap.mutable_data<float>();

    BoxesOverlapLauncher(boxes_a.stream(), num_a, boxes_a_data, num_b, boxes_b_data, ans_overlap_data);

    return {ans_overlap};
}

std::vector<paddle::Tensor>  boxes_iou_bev_gpu(const paddle::Tensor & boxes_a_tensor, const paddle::Tensor & boxes_b_tensor) {
    // params boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params boxes_b: (M, 7) [x, y, z, dx, dy, dz, heading]
    // params ans_overlap: (N, M)

    int num_a = boxes_a_tensor.shape()[0];
    int num_b = boxes_b_tensor.shape()[0];

    const float * boxes_a_data = boxes_a_tensor.data<float>();
    const float * boxes_b_data = boxes_b_tensor.data<float>();
    auto ans_iou_tensor = paddle::Tensor(paddle::PlaceType::kGPU, {num_a, num_b});
    float *ans_iou_data = ans_iou_tensor.mutable_data<float>();

    BoxesIouBevLauncher(boxes_a_tensor.stream(), num_a, boxes_a_data, num_b, boxes_b_data, ans_iou_data);

    return {ans_iou_tensor};
}

std::vector<paddle::Tensor>  nms_gpu(const paddle::Tensor &boxes,
                                     float nms_overlap_thresh){
  // params boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
  auto keep = paddle::Tensor(paddle::PlaceType::kCPU, {boxes.shape()[0]});
  auto num_to_keep_tensor = paddle::Tensor(paddle::PlaceType::kCPU, {1});
  int *num_to_keep_data = num_to_keep_tensor.mutable_data<int>();

  int boxes_num = boxes.shape()[0];
  const float *boxes_data = boxes.data<float>();
  long *keep_data = keep.mutable_data<long>();

  const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);

  // int64_t *mask_data = NULL;
  // CHECK_ERROR(cudaMalloc((void**)&mask_data, boxes_num * col_blocks *
  // sizeof(int64_t)));
  auto mask = paddle::Tensor(paddle::PlaceType::kGPU, {boxes_num * col_blocks});
  int64_t *mask_data = mask.mutable_data<int64_t>();
  NmsLauncher(boxes.stream(), boxes_data, mask_data, boxes_num,
               nms_overlap_thresh);

  // std::vector<int64_t> mask_cpu(boxes_num * col_blocks);

  // CHECK_ERROR(cudaMemcpy(&mask_cpu[0], mask_data, boxes_num * col_blocks *
  // sizeof(int64_t),
  //                       cudaMemcpyDeviceToHost));
  //const paddle::Tensor mask_cpu_tensor = mask.copy_to(phi::CPUPlace(), true);
  const paddle::Tensor mask_cpu_tensor = mask.copy_to(paddle::PlaceType::kCPU, true);
  const int64_t *mask_cpu = mask_cpu_tensor.data<int64_t>();
  // cudaFree(mask_data);

  int64_t remv_cpu[col_blocks];
  memset(remv_cpu, 0, col_blocks * sizeof(int64_t));

  int num_to_keep = 0;

  for (int i = 0; i < boxes_num; i++) {
    int nblock = i / THREADS_PER_BLOCK_NMS;
    int inblock = i % THREADS_PER_BLOCK_NMS;

    if (!(remv_cpu[nblock] & (1ULL << inblock))) {
      keep_data[num_to_keep++] = i;
      const int64_t *p = &mask_cpu[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; j++) {
        remv_cpu[j] |= p[j];
      }
    }
  }

  num_to_keep_data[0] = num_to_keep;
  if (cudaSuccess != cudaGetLastError())
    printf("Error!\n");

  return {keep, num_to_keep_tensor};
}


std::vector<paddle::Tensor> nms_normal_gpu(const paddle::Tensor &boxes,
                                           float nms_overlap_thresh) {
  // params boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
  // params keep: (N)

  auto keep = paddle::Tensor(paddle::PlaceType::kCPU, {boxes.shape()[0]});
  auto num_to_keep_tensor = paddle::Tensor(paddle::PlaceType::kCPU, {1});
  int *num_to_keep_data = num_to_keep_tensor.mutable_data<int>();
  int boxes_num = boxes.shape()[0];
  const float *boxes_data = boxes.data<float>();
  long *keep_data = keep.mutable_data<long>();

  const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);

  // int64_t *mask_data = NULL;
  // CHECK_ERROR(cudaMalloc((void**)&mask_data, boxes_num * col_blocks *
  // sizeof(int64_t)));
  auto mask = paddle::Tensor(paddle::PlaceType::kGPU, {boxes_num * col_blocks});
  int64_t *mask_data = mask.mutable_data<int64_t>();
  NmsNormalLauncher(boxes.stream(), boxes_data, mask_data, boxes_num,
                      nms_overlap_thresh);

  // int64_t mask_cpu[boxes_num * col_blocks];
  // int64_t *mask_cpu = new int64_t [boxes_num * col_blocks];
  // std::vector<int64_t> mask_cpu(boxes_num * col_blocks);

  // CHECK_ERROR(cudaMemcpy(&mask_cpu[0], mask_data, boxes_num * col_blocks *
  // sizeof(int64_t),
  //                       cudaMemcpyDeviceToHost));

  // cudaFree(mask_data);

  //const paddle::Tensor mask_cpu_tensor = mask.copy_to(phi::CPUPlace(), true);
  const paddle::Tensor mask_cpu_tensor = mask.copy_to(paddle::PlaceType::kCPU, true);
  const int64_t *mask_cpu = mask_cpu_tensor.data<int64_t>();

  int64_t remv_cpu[col_blocks];
  memset(remv_cpu, 0, col_blocks * sizeof(int64_t));

  int num_to_keep = 0;

  for (int i = 0; i < boxes_num; i++) {
    int nblock = i / THREADS_PER_BLOCK_NMS;
    int inblock = i % THREADS_PER_BLOCK_NMS;

    if (!(remv_cpu[nblock] & (1ULL << inblock))) {
      keep_data[num_to_keep++] = i;
      const int64_t *p = &mask_cpu[0] + i * col_blocks;
      for (int j = nblock; j < col_blocks; j++) {
        remv_cpu[j] |= p[j];
      }
    }
  }

  num_to_keep_data[0] = num_to_keep;
  if (cudaSuccess != cudaGetLastError()) {
    printf("Error!\n");
  }
  return {keep, num_to_keep_tensor};
}
