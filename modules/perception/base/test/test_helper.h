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
#pragma once

#define CUDA_TEST_DEVICE -1
#define EXAMPLES_SOURCE_DIR "examples/"
#define ABS_TEST_DATA_DIR "src/caffe/test/test_data"

namespace apollo {
namespace perception {
namespace base {

template <typename TypeParam>
class MultiDeviceTest : public ::testing::Test {
 public:
  typedef typename TypeParam::Dtype Dtype;

 protected:
  MultiDeviceTest() {
    // Caffe::set_mode(TypeParam::device);
  }
  virtual ~MultiDeviceTest() {}
};

typedef ::testing::Types<float, double> TestDtypes;

template <typename TypeParam>
struct CPUDevice {
  typedef TypeParam Dtype;
  // static const Caffe::Brew device = Caffe::CPU;
};

template <typename Dtype>
class CPUDeviceTest : public MultiDeviceTest<CPUDevice<Dtype>> {};

#ifdef PERCEPTION_CPU_ONLY

typedef ::testing::Types<CPUDevice<float>, CPUDevice<double>>
    TestDtypesAndDevices;

#else

template <typename TypeParam>
struct GPUDevice {
  typedef TypeParam Dtype;
  // static const Caffe::Brew device = Caffe::GPU;
};

template <typename Dtype>
class GPUDeviceTest : public MultiDeviceTest<GPUDevice<Dtype>> {};

typedef ::testing::Types<CPUDevice<float>, CPUDevice<double>, GPUDevice<float>,
                         GPUDevice<double>>
    TestDtypesAndDevices;

#endif

}  // namespace base
}  // namespace perception
}  // namespace apollo
