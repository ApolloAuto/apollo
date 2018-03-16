/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "util.h"

namespace apollo {
namespace perception {
__global__ void
resize_linear_kernel(const unsigned char *src, float *dst, int channel, int height, int width,
                     int dst_height, int dst_width, float fx, float fy) {
    const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
    const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
    if (dst_x < dst_width && dst_y < dst_height) {
        float src_x = (dst_x + 0.5) * fx - 0.5;
        float src_y = (dst_y + 0.5) * fy - 0.5;
        const int x1 = __float2int_rd(src_x);
        const int y1 = __float2int_rd(src_y);
        const int x2 = x1 + 1;
        const int y2 = y1 + 1;
        const int x2_read = min(x2, width - 1);
        const int y2_read = min(y2, height - 1);
        //(h*width+w)*channel+c
        float src_reg = 0;
        for (int c = 0; c < 3; c++) {
            float out = 0;

            int idx11 = (y1 * width + x1) * channel;
            src_reg = (int) src[idx11 + c];
            out = out + (x2 - src_x) * (y2 - src_y) * src_reg;
            int idx12 = (y1 * width + x2_read) * channel;
            src_reg = (int) src[idx12 + c];
            out = out + src_reg * (src_x - x1) * (y2 - src_y);

            int idx21 = (y2_read * width + x1) * channel;
            src_reg = (int) src[idx21 + c];
            out = out + src_reg * (x2 - src_x) * (src_y - y1);

            int idx22 = (y2_read * width + x2_read) * channel;
            src_reg = (int) src[idx22 + c];
            out = out + src_reg * (src_x - x1) * (src_y - y1);
            if (out < 0) {
                out = 0;
            }
            if (out > 255) {
                out = 255;
            }
            int dst_idx = (dst_y * dst_width + dst_x) * channel + c;
            //   printf("%f %d %d %d %d\n",out,x1,y1,x2,y2);
            dst[dst_idx] = out;

        }
    }
}
__global__ void
resize_linear_with_mean_scale_kernel(const unsigned char *src, float *dst, int channel, 
                                    int height, int width, int dst_height, int dst_width, 
                                    float fx, float fy, const float mean_b, const float mean_g, 
                                    const float mean_r, const float scale) {
    const int dst_x = blockDim.x * blockIdx.x + threadIdx.x;
    const int dst_y = blockDim.y * blockIdx.y + threadIdx.y;
    if (dst_x < dst_width && dst_y < dst_height) {
        float src_x = (dst_x + 0.5) * fx - 0.5;
        float src_y = (dst_y + 0.5) * fy - 0.5;
        const int x1 = __float2int_rd(src_x);
        const int y1 = __float2int_rd(src_y);
        const int x2 = x1 + 1;
        const int y2 = y1 + 1;
        const int x2_read = min(x2, width - 1);
        const int y2_read = min(y2, height - 1);
        //(h*width+w)*channel+c
        float src_reg = 0;
        int idx11 = (y1 * width + x1) * channel;
        int idx12 = (y1 * width + x2_read) * channel;
        int idx21 = (y2_read * width + x1) * channel;
        int idx22 = (y2_read * width + x2_read) * channel;
        int dst_idx = (dst_y * dst_width + dst_x) * channel;
        for (int c = 0; c < channel; c++) {
            float mean = 0.f;
            if(c == 0){
              mean = mean_b;    
            }else if (c == 1){
              mean = mean_g;    
            }else{
              mean = mean_r;    
            }
            float out = 0;

            src_reg =  src[idx11 + c];
            out = out + (x2 - src_x) * (y2 - src_y) * src_reg;

            src_reg = src[idx12 + c];
            out = out + src_reg * (src_x - x1) * (y2 - src_y);

            src_reg =  src[idx21 + c];
            out = out + src_reg * (x2 - src_x) * (src_y - y1);

            src_reg =  src[idx22 + c];
            out = out + src_reg * (src_x - x1) * (src_y - y1);
            dst[dst_idx + c] = (out - mean) * scale;
        }
    }
}
int divup(int a, int b) {
    if (a % b) {
        return a / b + 1;
    } else {
        return a / b;
    }
}
void gpu_memcpy(const size_t N, const void *X, void *Y) {
    if (X != Y) {
        CUDA_CHECK(cudaMemcpy(Y, X, N, cudaMemcpyDefault));  // NOLINT(caffe/alt_fn)
    }
}

void resize(cv::Mat frame, caffe::Blob<float> *dst, std::shared_ptr <SyncedMemory> src_gpu,
            int start_axis) {
    int origin_width = frame.cols;
    int origin_height = frame.rows;
    
    const dim3 block(32, 8);
    int width = dst->height();
    int height = dst->channels();
    int channel = dst->width();
    LOG(INFO) << width << "|" << height << "|" << channel;
    float fx = (float) origin_width / (float) width;
    float fy = (float) origin_height / (float) height;
    const dim3 grid(divup(width, block.x), divup(height, block.y));
    if (src_gpu == nullptr) {
        src_gpu.reset(
                new SyncedMemory(origin_width * origin_height * channel * sizeof(unsigned char)));
    }
    src_gpu->set_cpu_data(frame.data);
    resize_linear_kernel << < grid, block >> > ((const unsigned char *) src_gpu->gpu_data(), dst
            ->mutable_gpu_data(), channel, origin_height, origin_width, height, width, fx, fy);

}

void resize(cv::Mat frame, caffe::Blob<float> *dst, std::shared_ptr <SyncedMemory> src_gpu,
            int start_axis, const float mean_b, const float mean_g, const float mean_r, 
            const float scale) {
    int origin_width = frame.cols;
    int origin_height = frame.rows;
    
    const dim3 block(32, 8);
    int width = dst->height();
    int height = dst->channels();
    int channel = dst->width();
    float fx = (float) origin_width / (float) width;
    float fy = (float) origin_height / (float) height;
    const dim3 grid(divup(width, block.x), divup(height, block.y));
    if (src_gpu == nullptr) {
        src_gpu.reset(
                new SyncedMemory(origin_width * origin_height * channel * sizeof(unsigned char)));
    }
    src_gpu->set_cpu_data(frame.data);
    resize_linear_with_mean_scale_kernel << < grid, block >> > ((const unsigned char *) src_gpu
            ->gpu_data(), dst->mutable_gpu_data(), channel, origin_height, origin_width, height, 
            width, fx, fy, mean_b, mean_g, mean_r, scale);

}

}
}
