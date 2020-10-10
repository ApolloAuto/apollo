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

#include "gtest/gtest.h"
#include "modules/perception/inference/utils/cuda_util.h"
#include "modules/perception/inference/utils/gemm.h"
#include "modules/perception/inference/utils/resize.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace inference {

TEST(loadTest, test) {
  ACHECK(!apollo::perception::inference::load_binary_data("unknow.txt"));
}

TEST(UtilTest, GemmTest) {
  base::Blob<float> a;
  base::Blob<float> b;
  base::Blob<float> c;
  a.Reshape({2, 2});
  b.Reshape({2, 2});
  c.Reshape({3, 3});
  float *p = a.mutable_cpu_data();
  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  p = b.mutable_cpu_data();
  p[0] = 2;
  p[1] = 3;
  p[2] = 4;
  p[3] = 5;
  inference::CudaUtil::set_device_id(0);
  inference::GPUGemmFloat(CblasNoTrans, CblasTrans, 2, 2, 2, 1, a.gpu_data(),
                          b.gpu_data(), 0, c.mutable_gpu_data());
  const float *out = c.cpu_data();
  EXPECT_EQ(8, out[0]);
  EXPECT_EQ(14, out[1]);
  EXPECT_EQ(18, out[2]);
  EXPECT_EQ(32, out[3]);

  inference::GPUGemmFloat(CblasNoTrans, CblasNoTrans, 2, 2, 2, 1, a.gpu_data(),
                          b.gpu_data(), 0, c.mutable_gpu_data());
  out = c.cpu_data();
  EXPECT_EQ(out[0], 10);
  EXPECT_EQ(out[1], 13);
  EXPECT_EQ(out[2], 22);
  EXPECT_EQ(out[3], 29);

  inference::GPUGemmFloat(CblasTrans, CblasNoTrans, 2, 2, 2, 1, a.gpu_data(),
                          b.gpu_data(), 0, c.mutable_gpu_data());
  out = c.cpu_data();
  EXPECT_EQ(out[0], 14);
  EXPECT_EQ(out[1], 18);
  EXPECT_EQ(out[2], 20);
  EXPECT_EQ(out[3], 26);
}

TEST(UtilTest, NormTest) {
  inference::CudaUtil::set_device_id(0);
  {
    inference::GPUL2Norm norm;
    base::Blob<float> data;
    const int num = 20;
    const int dim = 512;
    data.Reshape({num, dim});
    float *p = data.mutable_cpu_data();
    for (int i = 0; i < num; ++i) {
      for (int j = 0; j < dim; ++j) {
        // TODO(gaohan) add a function
        *p = rand() % 10000 / 10000.0f;  // NOLINT
        ++p;
      }
    }
    norm.L2Norm(&data);
    const float *result = data.cpu_data();
    for (int i = 0; i < num; ++i) {
      float sum = 0.0f;
      for (int j = 0; j < dim; ++j) {
        sum += (*result) * (*result);
        ++result;
      }
      ASSERT_TRUE(fabs(sum - 1) < 1e-3);
    }
  }

  {
    inference::GPUL2Norm norm;
    base::Blob<float> data;
    const int num = 0;
    const int dim = 512;
    data.Reshape({num, dim});
    norm.L2Norm(&data);
    EXPECT_EQ(data.num(), 0);
  }
}

const int thread_count = 3;
std::vector<cublasHandle_t> handlers(thread_count, nullptr);
void CudaUtilFun(int gpu_count, int thread_id) {
  EXPECT_TRUE(inference::CudaUtil::set_device_id(0));
  for (int i = 1; i < gpu_count; ++i) {
    EXPECT_TRUE(inference::CudaUtil::set_device_id(i));
  }
  handlers[thread_id] = inference::CudaUtil::get_handler();
}

TEST(UtilTest, CudaUtilTest) {
  int gpu_count = 0;
  cudaGetDeviceCount(&gpu_count);
  EXPECT_GT(gpu_count, 0);
  std::thread threads[5];
  for (int i = 0; i < thread_count; ++i) {
    threads[i] = std::thread(CudaUtilFun, gpu_count, i);
  }
  for (int i = 0; i < thread_count; ++i) {
    threads[i].join();
  }
}

TEST(UtilTest, test_resize_gpu) {
  cv::Mat img = cv::imread("inference_test_data/images/test.jpg");
  base::Image8U src_image(img.rows, img.cols, base::Color::BGR);

  std::shared_ptr<apollo::perception::base::Blob<uint8_t>> src_blob(
      new apollo::perception::base::Blob<uint8_t>);
  NppiSize roi;
  int height = src_image.rows();
  int width = src_image.cols();
  roi.height = height;
  roi.width = width;
  src_blob->Reshape({1, roi.height, roi.width, src_image.channels()});
  nppiCopy_8u_C3R(src_image.gpu_data(), src_image.width_step(),
                  src_blob->mutable_gpu_data(),
                  src_blob->count(2) * sizeof(uint8_t), roi);

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5,
                              img.channels()};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    EXPECT_TRUE(inference::ResizeGPU(src_image, dst_blob, width, 0));
    EXPECT_EQ(dst_blob->shape(0), 1);
    EXPECT_EQ(dst_blob->shape(1), src_image.rows() * 0.5);
    EXPECT_EQ(dst_blob->shape(2), src_image.cols() * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_image.channels());
  }

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5, 0};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    EXPECT_FALSE(inference::ResizeGPU(src_image, dst_blob, width, 0));
  }

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5,
                              img.channels()};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_TRUE(inference::ResizeGPU(src_image, dst_blob, width, 0, mean_bgr[0],
                                     mean_bgr[1], mean_bgr[2], true, 1.0));
    EXPECT_EQ(dst_blob->shape(0), 1);
    EXPECT_EQ(dst_blob->shape(1), src_image.rows() * 0.5);
    EXPECT_EQ(dst_blob->shape(2), src_image.cols() * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_image.channels());
  }

  {
    std::vector<int> shape = {1, img.channels(), img.rows * 0.5,
                              img.cols * 0.5};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_TRUE(inference::ResizeGPU(src_image, dst_blob, width, 0, mean_bgr[0],
                                     mean_bgr[1], mean_bgr[2], false, 1.0));
    EXPECT_EQ(dst_blob->shape(0), 1);
    EXPECT_EQ(dst_blob->shape(1), src_image.channels());
    EXPECT_EQ(dst_blob->shape(2), src_image.rows() * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_image.cols() * 0.5);
  }

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5, 0};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_FALSE(inference::ResizeGPU(src_image, dst_blob, width, 0,
                                      mean_bgr[0], mean_bgr[1], mean_bgr[2],
                                      true, 1.0));
  }

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5,
                              img.channels()};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_TRUE(inference::ResizeGPU(*src_blob, dst_blob, width, 0, mean_bgr[0],
                                     mean_bgr[1], mean_bgr[2], true, 1.0));
    EXPECT_EQ(dst_blob->shape(0), src_blob->shape(0));
    EXPECT_EQ(dst_blob->shape(1), src_blob->shape(1) * 0.5);
    EXPECT_EQ(dst_blob->shape(2), src_blob->shape(2) * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_blob->shape(3));
  }

  {
    std::vector<int> shape = {1, img.channels(), img.rows * 0.5,
                              img.cols * 0.5};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_TRUE(inference::ResizeGPU(*src_blob, dst_blob, width, 0, mean_bgr[0],
                                     mean_bgr[1], mean_bgr[2], false, 1.0));
    EXPECT_EQ(dst_blob->shape(0), src_blob->shape(0));
    EXPECT_EQ(dst_blob->shape(1), src_blob->shape(3));
    EXPECT_EQ(dst_blob->shape(2), src_blob->shape(1) * 0.5);
    EXPECT_EQ(dst_blob->shape(3), src_blob->shape(2) * 0.5);
  }

  {
    std::vector<int> shape = {1, img.rows * 0.5, img.cols * 0.5, 0};
    std::shared_ptr<apollo::perception::base::Blob<float>> dst_blob(
        new apollo::perception::base::Blob<float>(shape));
    int mean_bgr[3] = {128, 128, 128};
    EXPECT_FALSE(inference::ResizeGPU(*src_blob, dst_blob, width, 0,
                                      mean_bgr[0], mean_bgr[1], mean_bgr[2],
                                      true, 1.0));
  }
}
}  // namespace inference
}  // namespace perception
}  // namespace apollo
