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

#include "modules/perception/cuda_util/util.h"
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/device_vector.h>

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

void resize(cv::Mat frame, caffe::Blob<float> *dst, std::shared_ptr <caffe::SyncedMemory> src_gpu,
            int start_axis) {
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
                new caffe::SyncedMemory(origin_width * origin_height * channel * sizeof(unsigned char)));
    }
    src_gpu->set_cpu_data(frame.data);
    resize_linear_kernel << < grid, block >> > ((const unsigned char *) src_gpu->gpu_data(), dst
            ->mutable_gpu_data(), channel, origin_height, origin_width, height, width, fx, fy);

}

void resize(cv::Mat frame, caffe::Blob<float> *dst, std::shared_ptr <caffe::SyncedMemory> src_gpu,
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
                new caffe::SyncedMemory(origin_width * origin_height * channel * sizeof(unsigned char)));
    }
    src_gpu->set_cpu_data(frame.data);
    resize_linear_with_mean_scale_kernel << < grid, block >> > ((const unsigned char *) src_gpu
            ->gpu_data(), dst->mutable_gpu_data(), channel, origin_height, origin_width, height, 
            width, fx, fy, mean_b, mean_g, mean_r, scale);

}
/******************/
struct index_functor : public thrust::unary_function<int, int> {
    int div_;
    int mul_;
    int offset_;
    index_functor(int div, int mul, int offset)
        : div_(div), mul_(mul), offset_(offset) {}

    __host__ __device__
    int operator()(const int &index) {
        return (index / div_) * mul_ + offset_;
    }
};

struct yuv2bgr_functor {
    template <typename Tuple>
    __host__ __device__
    void operator()(Tuple t) {
        const uint8_t &y = thrust::get<0>(t);
        const uint8_t &u = thrust::get<1>(t);
        const uint8_t &v = thrust::get<2>(t);
        uint8_t &b = thrust::get<3>(t);
        uint8_t &g = thrust::get<4>(t);
        uint8_t &r = thrust::get<5>(t);

        const int y2 = (int)y;
        const int u2 = (int)u - 128;
        const int v2 = (int)v - 128;

        float r2 = y2 + (1.4065 * v2);
        float g2 = y2 - (0.3455 * u2) - (0.7169 * v2);
        float b2 = y2 + (2.041 * u2);

        // Cap the values.
        r = clip_value(r2);
        g = clip_value(g2);
        b = clip_value(b2);
    }

    __host__ __device__
    inline uint8_t clip_value(float v) {
        v = v < 0 ? 0 : v;
        return v > 255 ? 255 : v;
    }
};
typedef thrust::device_vector<int>::iterator IntegerIterator;
void yuyv2bgr(const uint8_t *yuv_data, uint8_t *bgr_data, const int pixel_num) {
    thrust::device_vector<uint8_t> bgr(pixel_num * 3);
    thrust::device_vector<uint8_t> yuv(yuv_data, yuv_data + pixel_num * 2);
    thrust::counting_iterator<int> first(0);
    thrust::counting_iterator<int> last(yuv.size() / 2);
    thrust::for_each(
        thrust::make_zip_iterator(thrust::make_tuple(
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(first, index_functor(1, 2, 0))),
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(first, index_functor(2, 4, 1))),
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(first, index_functor(2, 4, 3))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(first, index_functor(1, 3, 0))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(first, index_functor(1, 3, 1))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(first, index_functor(1, 3, 2))))),
        thrust::make_zip_iterator(thrust::make_tuple(
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(last, index_functor(1, 2, 0))),
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(last, index_functor(2, 4, 1))),
                thrust::make_permutation_iterator(yuv.begin(),
                    thrust::make_transform_iterator(last, index_functor(2, 4, 3))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(last, index_functor(1, 3, 0))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(last, index_functor(1, 3, 1))),
                thrust::make_permutation_iterator(bgr.begin(),
                    thrust::make_transform_iterator(last, index_functor(1, 3, 2))))),
        yuv2bgr_functor());
    cudaMemcpy(bgr_data, thrust::raw_pointer_cast(bgr.data()), pixel_num * 3, cudaMemcpyDeviceToHost);
}


}
}
