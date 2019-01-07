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

#include "modules/perception/inference/caffe/caffe_net.h"

#include "caffe/util/math_functions.hpp"
#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace inference {

class TestableCaffeNet : public CaffeNet {
 public:
 public:
  TestableCaffeNet(const std::string &net_file, const std::string &modelfile,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs)
      : CaffeNet(net_file, modelfile, outputs, inputs) {}
  using CaffeNet::net_;
  using CaffeNet::reshape;
  using CaffeNet::shape;
};

TEST(CaffeNetTest, init_test) {
  // std::vector<std::string> file_list =
  // {"ARZ034_12_1499218335_1499218635_500",
  //                                       "ARZ034_12_1499218335_1499218635_520",
  //                                       "ARZ034_12_1499218335_1499218635_540",
  //                                       "ARZ034_12_1499218335_1499218635_560",
  //                                       "ARZ034_12_1499218335_1499218635_580",
  //                                       "ARZ034_12_1499218335_1499218635_606",
  //                                       "ARZ034_12_1499218335_1499218635_834",
  //                                       "ARZ034_12_1499218335_1499218635_854",
  //                                       "ARZ034_12_1499218335_1499218635_874",
  //                                       "ARZ034_12_1499218335_1499218635_894"};

  // std::string image_root = "./inference_test_data/images/";
  // std::string image_ext = ".jpg";
  // std::string output_blob_path = "data/yolo/blob_names.txt";
  // std::string proto_file = "./inference_test_data/yolo/caffe.pt";
  // std::string weight_file = "./inference_test_data/yolo/caffe.caffemodel";
  // std::string result_file = "data/yolo/result.dat";
  // std::string input_blob_name = "data";

  // std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
  //                                  "ori_pred", "dim_pred", "lof_pred",
  //                                  "lor_pred", "bad"};

  // std::vector<std::string> inputs{"data", "bad"};
  // auto rt_net = apollo::perception::inference::CreateInferenceByName(
  //     "CaffeNet", proto_file, weight_file, outputs, inputs);
  // CHECK(rt_net);
  // int height = 576;
  // int width = 1440;
  // int offset_y = 312;
  // int batchsize = 1;
  // std::vector<int> shape = {batchsize, height, width, 3};
  // std::map<std::string, std::vector<int>> shape_map{{"bad", shape}};
  // CHECK(rt_net->Init(shape_map));
  // rt_net->Infer();
  // CHECK(!rt_net->get_blob("bad"));
}

TEST(CaffeNetTest, cpu_test) {
  // std::vector<std::string> file_list =
  // {"ARZ034_12_1499218335_1499218635_500",
  //                                       "ARZ034_12_1499218335_1499218635_520",
  //                                       "ARZ034_12_1499218335_1499218635_540",
  //                                       "ARZ034_12_1499218335_1499218635_560",
  //                                       "ARZ034_12_1499218335_1499218635_580",
  //                                       "ARZ034_12_1499218335_1499218635_606",
  //                                       "ARZ034_12_1499218335_1499218635_834",
  //                                       "ARZ034_12_1499218335_1499218635_854",
  //                                       "ARZ034_12_1499218335_1499218635_874",
  //                                       "ARZ034_12_1499218335_1499218635_894"};

  // std::string image_root = "./inference_test_data/images/";
  // std::string image_ext = ".jpg";
  // std::string output_blob_path = "data/yolo/blob_names.txt";
  // std::string proto_file = "./inference_test_data/yolo/caffe.pt";
  // std::string weight_file = "./inference_test_data/yolo/caffe.caffemodel";
  // std::string result_file = "data/yolo/result.dat";
  // std::string input_blob_name = "data";

  // std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
  //                                  "ori_pred", "dim_pred", "lof_pred",
  //                                  "lor_pred"};

  // apollo::perception::inference::Inference *rt_net = nullptr;
  // std::vector<std::string> inputs{"data", "bad"};

  // rt_net = apollo::perception::inference::CreateInferenceByName(
  //     "CaffeNet", proto_file, weight_file, outputs, inputs);
  // CHECK(rt_net);
  // int height = 576;
  // int width = 1440;
  // int offset_y = 312;
  // int batchsize = 1;
  // std::vector<int> shape = {batchsize, height, width, 3};
  // rt_net->set_gpu_id(-1);
  // std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
  // shape}}; rt_net->set_max_batch_size(batchsize);
  // CHECK(rt_net->Init(shape_map));
  // rt_net->Infer();
  // if (rt_net != nullptr) {
  //   delete rt_net;
  // }
}
TEST(CaffeNetTest, reshape_test) {
  // std::vector<std::string> file_list =
  // {"ARZ034_12_1499218335_1499218635_500",
  //                                       "ARZ034_12_1499218335_1499218635_520",
  //                                       "ARZ034_12_1499218335_1499218635_540",
  //                                       "ARZ034_12_1499218335_1499218635_560",
  //                                       "ARZ034_12_1499218335_1499218635_580",
  //                                       "ARZ034_12_1499218335_1499218635_606",
  //                                       "ARZ034_12_1499218335_1499218635_834",
  //                                       "ARZ034_12_1499218335_1499218635_854",
  //                                       "ARZ034_12_1499218335_1499218635_874",
  //                                       "ARZ034_12_1499218335_1499218635_894"};

  // std::string image_root = "./inference_test_data/images/";
  // std::string image_ext = ".jpg";
  // std::string output_blob_path = "data/yolo/blob_names.txt";
  // std::string proto_file = "./inference_test_data/yolo/caffe.pt";
  // std::string weight_file = "./inference_test_data/yolo/caffe.caffemodel";
  // std::string result_file = "data/yolo/result.dat";
  // std::string input_blob_name = "data";

  // std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
  //                                  "ori_pred", "dim_pred", "lof_pred",
  //                                  "lor_pred"};

  // std::vector<std::string> inputs{"data", "bad"};
  // TestableCaffeNet caffenet(proto_file, weight_file, outputs, inputs);
  // int height = 111;
  // int width = 222;
  // int offset_y = 23;
  // int batchsize = 2;
  // caffenet.Init(std::map<std::string, std::vector<int>>{});
  // std::vector<int> shape = {batchsize, height, width, 3};
  // CHECK(!caffenet.shape("bad", &shape));
  // auto datablob = caffenet.get_blob("data");
  // {
  //   datablob->Reshape(1, height, width, 3);
  //   for (int i = 0; i < datablob->count(); i++) {
  //     datablob->mutable_cpu_data()[i] = i;
  //   }
  //   caffenet.reshape();
  //   auto caffedatablob = caffenet.net_->blob_by_name("data");
  //   for (int i = 0; i < datablob->count(); i++) {
  //     CHECK_EQ(caffedatablob->cpu_data()[i], datablob->cpu_data()[i]);
  //   }
  // }
  // {
  //   datablob->Reshape(2, height, width, 3);
  //   caffe::caffe_gpu_set(datablob->count(), 0.3f,
  //   datablob->mutable_gpu_data()); caffenet.reshape(); auto caffedatablob =
  //   caffenet.net_->blob_by_name("data"); for (int i = 0; i <
  //   datablob->count(); i++) {
  //     CHECK_EQ(caffedatablob->cpu_data()[i], datablob->cpu_data()[i]);
  //   }
  // }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
