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

#include "network.h"

namespace apollo {
namespace perception {

__host__ __device__
float sigmoid_gpu(float x) {
    return 1.0 / (1.0 + exp(-x));
}
__global__ void get_object_kernel(int n,
                                  const float *loc_data,
                                  const float *obj_data,
                                  const float *cls_data,
                                  const float *ori_data,
                                  const float *dim_data,
                                  const float *lof_data,
                                  const float *lor_data,
                                  const float *anchor_data,
                                  int width,
                                  int height,
                                  int num_anchors,
                                  int num_classes,
                                  float confidence_threshold,
                                  bool with_ori,
                                  bool with_dim,
                                  bool with_lof,
                                  bool with_lor,
                                  float *res_box_data,
                                  float *res_cls_data,
                                  int s_box_block_size) {

    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); i += blockDim.x * gridDim.x) {
        int box_block = s_box_block_size;

        int idx = i;
        int c = idx % num_anchors;
        idx = idx / num_anchors;
        int w = idx % width;
        idx = idx / width;
        int h = idx;
        int offset_loc = ((h * width + w) * num_anchors + c) * 4;
        int offset_obj = (h * width + w) * num_anchors + c;
        int offset_cls = ((h * width + w) * num_anchors + c) * num_classes;
        float scale = obj_data[offset_obj];
       //   printf("%d %d %d %d %d (%d %d %d)| ",i,c,w,h,offset_loc,num_anchors,width,height);
        float cx = (w + sigmoid_gpu(loc_data[offset_loc + 0])) / width;
        float cy = (h + sigmoid_gpu(loc_data[offset_loc + 1])) / height;
        float hw = exp(loc_data[offset_loc + 2])* anchor_data[2 * c] / width * 0.5;
        float hh = exp(loc_data[offset_loc + 3]) * anchor_data[2 * c + 1] / height * 0.5;

        for (int k = 0; k < num_classes; ++k) {
              float prob = (cls_data[offset_cls + k] * scale > confidence_threshold ?
                            cls_data[offset_cls + k] * scale : 0);
        //    printf("%f %f | ",prob,cls_data[offset_cls + k] * scale);
              res_cls_data[k*width*height*num_anchors+i]=prob;
        }
        res_box_data[i * box_block + 0] = cx - hw;
        res_box_data[i * box_block + 1] = cy - hh;
        res_box_data[i * box_block + 2] = cx + hw;
        res_box_data[i * box_block + 3] = cy + hh;

        if (with_ori) {
            int offset_ori = ((h * width + w) * num_anchors + c) * 2;
            res_box_data[i*box_block+4]=atan2(ori_data[offset_ori+1],ori_data[offset_ori]);
        }

        if (with_dim) {
            int offset_dim = ((h * width + w) * num_anchors + c) * 3;
            res_box_data[i*box_block+5]=dim_data[offset_dim + 0];
            res_box_data[i*box_block+6]=dim_data[offset_dim + 1];
            res_box_data[i*box_block+7]=dim_data[offset_dim + 2];
        }

        if (with_lof) {
            int offset_lof = ((h * width + w) * num_anchors + c) * 4;
            auto &&dst_ptr = res_box_data + i * box_block + 8;
            auto &&src_ptr = lof_data + offset_lof;
            auto sb_x  = src_ptr[0] * hw * 2 + cx;
            auto sb_y  = src_ptr[1] * hh * 2 + cy;
            auto sb_hw = exp(src_ptr[2]) * hw;
            auto sb_hh = exp(src_ptr[3]) * hh;
            dst_ptr[0] = sb_x - sb_hw;
            dst_ptr[1] = sb_y - sb_hh;
            dst_ptr[2] = sb_x + sb_hw;
            dst_ptr[3] = sb_y + sb_hh;
        }

        if (with_lor) {
            int offset_lor = ((h * width + w) * num_anchors + c) * 4;
            auto &&dst_ptr = res_box_data + i * box_block + 12;
            auto &&src_ptr = lor_data + offset_lor;
            auto sb_x  = src_ptr[0] * hw * 2 + cx;
            auto sb_y  = src_ptr[1] * hh * 2 + cy;
            auto sb_hw = exp(src_ptr[2]) * hw;
            auto sb_hh = exp(src_ptr[3]) * hh;
            dst_ptr[0] = sb_x - sb_hw;
            dst_ptr[1] = sb_y - sb_hh;
            dst_ptr[2] = sb_x + sb_hw;
            dst_ptr[3] = sb_y + sb_hh;
        }
    }
}

void GetObjectsGPU(int n,
                                  const float *loc_data,
                                  const float *obj_data,
                                  const float *cls_data,
                                  const float *ori_data,
                                  const float *dim_data,
                                  const float *lof_data,
                                  const float *lor_data,
                                  const float *anchor_data,
                                  int width,
                                  int height,
                                  int num_anchors,
                                  int num_classes,
                                  float confidence_threshold,
                                  bool with_ori,
                                  bool with_dim,
                                  bool with_lof,
                                  bool with_lor,
                                  float *res_box_data,
                                  float *res_cls_data, 
                                  int s_box_block_size) {
	const int thread_size = 512;
	int block_size = (n + thread_size -1) / thread_size;
        {
		get_object_kernel << < block_size, thread_size >> >
                                           (n, loc_data, obj_data, cls_data, ori_data, dim_data, lof_data, lor_data, anchor_data, width,
                                           height, num_anchors, num_classes, confidence_threshold, with_ori, with_dim,  with_lof, with_lor, 
                                           res_box_data, res_cls_data, s_box_block_size);
	}    
	cudaDeviceSynchronize();
}

} // namespace apollo
} // namespace perception
