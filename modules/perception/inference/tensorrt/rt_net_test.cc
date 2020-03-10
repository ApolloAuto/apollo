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

#include "modules/perception/inference/tensorrt/rt_net.h"

#include "gtest/gtest.h"
#include "gtest/gtest_prod.h"

#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/util.h"

namespace apollo {
namespace perception {
namespace inference {

class TestableRTNet : public RTNet {
 public:
 public:
  TestableRTNet(const std::string &net_file, const std::string &model_file,
                const std::vector<std::string> &outputs,
                const std::vector<std::string> &inputs)
      : RTNet(net_file, model_file, outputs, inputs) {}
  using RTNet::checkInt8;
  using RTNet::loadLayerWeights;
  using RTNet::mergeBN;
  using RTNet::shape;
};

// TEST(RTNetTest, init_test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};

//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "bad1",
//                                    "lof_pred", "lor_pred", "bad"};

//   std::vector<std::string> inputs{"data", "bad"};
//   TestableRTNet rt_net(proto_file, weight_file, outputs, inputs);

//   int height = 576;
//   int width = 1440;
//   int offset_y = 312;
//   int batchsize = 1;
//   std::vector<int> shape = {batchsize, height, width, 3};
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net.set_max_batch_size(batchsize);
//   ACHECK(rt_net.Init(shape_map));
//   ACHECK(!rt_net.shape("bad", &shape));

//   ACHECK(!rt_net.checkInt8("test", nullptr));
//   std::shared_ptr<nvinfer1::Int8EntropyCalibrator> calibrator;
//   apollo::perception::inference::BatchStream stream;
//   calibrator.reset(new nvinfer1::Int8EntropyCalibrator(stream, 0, true, ""));
//   ACHECK(!rt_net.checkInt8("test", calibrator.get()));
//   ACHECK(rt_net.checkInt8("Tesla P4", calibrator.get()));
// }
// TEST(RTNetLayerOwnCalibratorTest, test) {
//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "bad",      "lor_pred", "bad"};

//   std::vector<std::string> inputs{"data", "bad"};
//   std::shared_ptr<nvinfer1::Int8EntropyCalibrator> calibrator;
//   apollo::perception::inference::BatchStream stream;
//   calibrator.reset(new nvinfer1::Int8EntropyCalibrator(stream, 0, true, ""));

//   apollo::perception::inference::RTNet *rt_net =
//       new apollo::perception::inference::RTNet(proto_file, weight_file,
//       outputs,
//                                                inputs, calibrator.get());
//   int height = 576;
//   int width = 1440;
//   int batchsize = 1;
//   std::vector<int> shape = {batchsize, height, width, 3};
//   rt_net->set_gpu_id(-1);
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net->Init(shape_map); delete rt_net;
// }
// TEST(RTNetLayerWeightsTest, test) {
//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "bad",      "lor_pred", "bad"};

//   std::vector<std::string> inputs{"data", "bad"};
//   TestableRTNet rt_net(proto_file, weight_file, outputs, inputs);
//   rt_net.set_gpu_id(-1);
//   auto weights = rt_net.loadLayerWeights(0.1, 0);
//   EXPECT_EQ(weights.count, 0);
//   weights = rt_net.loadLayerWeights(nullptr, 0);
//   EXPECT_EQ(weights.count, 0);
// }
// TEST(RTNetScaleLayerTest, test) {
//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "bad",      "lor_pred", "bad"};

//   std::vector<std::string> inputs{"data", "bad"};
//   TestableRTNet rt_net(proto_file, weight_file, outputs, inputs);
//   LayerParameter layer_param;
//   auto bn_param = layer_param.mutable_batch_norm_param();
//   bn_param->set_eps(1);
//   for (int i = 0; i < 3; i++) {
//     auto blob = layer_param.add_blobs();
//     for (int j = 0; j < 10; j++) {
//       blob->add_data(4);
//     }
//   }
//   rt_net.set_gpu_id(-1);
//   rt_net.mergeBN(0, &layer_param);
//   for (int i = 0; i < 2; i++) {
//     auto blob = layer_param.blobs(i);
//     for (int j = 0; j < blob.data_size(); j++) {
//       EXPECT_NEAR(blob.data(j), 0.707107, 1e5);
//     }
//   }
//   rt_net.mergeBN(1, &layer_param);
//   for (int i = 0; i < 1; i++) {
//     auto blob = layer_param.blobs(i);
//     for (int j = 0; j < blob.data_size(); j++) {
//       EXPECT_NEAR(blob.data(j), 0.707107, 1e5);
//     }
//   }
// }
// TEST(RTNetCPUInitTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};

//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred"};

//   apollo::perception::inference::Inference *rt_net = nullptr;
//   std::vector<std::string> inputs{"data", "bad"};

//   rt_net = apollo::perception::inference::CreateInferenceByName(
//       "RTNet", proto_file, weight_file, outputs, inputs);
//   ACHECK(rt_net);
//   int height = 576;
//   int width = 1440;
//   int offset_y = 312;
//   int batchsize = 1;
//   std::vector<int> shape = {batchsize, height, width, 3};
//   rt_net->set_gpu_id(-1);
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net->set_max_batch_size(batchsize);
//   ACHECK(!rt_net->Init(shape_map));
//   if (rt_net != nullptr) {
//     delete rt_net;
//   }
// }
// TEST(InferenceTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};

//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred"};

//   apollo::perception::inference::Inference *rt_net = nullptr;
//   std::vector<std::string> inputs{"data", "bad"};

//   rt_net = apollo::perception::inference::CreateInferenceByName(
//       "UNKNWONNet", proto_file, weight_file, outputs, inputs);
//   ACHECK(!rt_net);
//   if (rt_net != nullptr) {
//     delete rt_net;
//   }
// }
// TEST(RTNetReshapeTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};

//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred", "lor_pred"};

//   apollo::perception::inference::Inference *rt_net = nullptr;
//   std::vector<std::string> inputs{"data", "bad"};

//   rt_net = apollo::perception::inference::CreateInferenceByName(
//       "RTNet", proto_file, weight_file, outputs, inputs);
//   ACHECK(rt_net);
//   int height = 111;
//   int width = 222;
//   int offset_y = 23;
//   int batchsize = 2;
//   std::vector<int> shape = {batchsize, height, width, 4};
//   std::map<std::string, std::vector<int>> shape_map{{"bad", shape}};
//   ACHECK(rt_net->Init(shape_map));
//   auto blob = rt_net->get_blob(input_blob_name);
//   for (int i = 0; i < shape.size(); i++) {
//     CHECK_NE(blob->shape(i), shape[i]);
//   }
//   if (rt_net != nullptr) {
//     delete rt_net;
//   }
// }

// TEST(RTNetTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};
//   std::string image_root =
//       "modules/perception/inference//inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_name = "input";
//   std::string input_blob_name = "data";
//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred"};
//   apollo::perception::inference::load_data<std::string>(output_blob_path,
//                                                         &outputs);
//   apollo::perception::inference::Inference *rt_net = nullptr;
//   std::vector<std::string> inputs{"data"};

//   rt_net = apollo::perception::inference::CreateInferenceByName(
//       "RTNet", proto_file, weight_file, outputs, inputs);
//   ACHECK(rt_net);
//   int height = 576;
//   int width = 1440;
//   int offset_y = 312;
//   std::vector<int> shape = {1, height, width, 3};
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net->Init(shape_map); auto input_blob =
//   rt_net->get_blob(input_blob_name); auto gt =
//   apollo::perception::inference::load_binary_data(result_file); int idx = 0;

//   int count = 3 * width * height;
//   for (auto image_file : file_list) {
//     cv::Mat img = cv::imread(image_root + image_file + image_ext);
//     cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
//     cv::Mat img_roi = img(roi);
//     img_roi.copyTo(img);
//     cv::resize(img, img, cv::Size(width, height));
//     auto input = input_blob->mutable_cpu_data();

//     for (int i = 0; i < count; i++) {
//       input[i] = img.data[i];
//     }
//     cudaDeviceSynchronize();
//     rt_net->Infer();
//     cudaDeviceSynchronize();
//     for (auto output_name : outputs) {
//       auto blob = rt_net->get_blob(output_name);
//       for (int tmp = 0; tmp < blob->count(); tmp++) {
//         EXPECT_NEAR(blob->cpu_data()[tmp], (gt.get())[idx], 1e-4)
//             << " idx: " << idx;
//         idx++;
//       }
//     }
//   }
//   if (rt_net != nullptr) {
//     delete rt_net;
//   }
// }

// TEST(CaffeNetTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};
//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_name = "input";
//   std::string input_blob_name = "data";

//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred"};
//   std::vector<std::string> inputs{"data"};

//   apollo::perception::inference::load_data<std::string>(output_blob_path,
//                                                         &outputs);
//   std::shared_ptr<apollo::perception::inference::Inference> rt_net;

//   rt_net.reset(apollo::perception::inference::CreateInferenceByName(
//       "CaffeNet", proto_file, weight_file, outputs, inputs));
//   ACHECK(rt_net);

//   int height = 576;
//   int width = 1440;
//   int offset_y = 312;
//   std::vector<int> shape = {1, height, width, 3};
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net->Init(shape_map); auto input_blob =
//   rt_net->get_blob(input_blob_name); auto gt =
//   apollo::perception::inference::load_binary_data(result_file); int idx = 0;

//   int count = 3 * width * height;
//   for (auto image_file : file_list) {
//     cv::Mat img = cv::imread(image_root + image_file + image_ext);
//     cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
//     cv::Mat img_roi = img(roi);
//     img_roi.copyTo(img);
//     cv::resize(img, img, cv::Size(width, height));
//     auto input = input_blob->mutable_cpu_data();

//     for (int i = 0; i < count; i++) {
//       input[i] = img.data[i];
//     }
//     cudaDeviceSynchronize();
//     rt_net->Infer();
//     cudaDeviceSynchronize();
//     for (auto output_name : outputs) {
//       auto blob = rt_net->get_blob(output_name);
//       for (int tmp = 0; tmp < blob->count(); tmp++) {
//         CHECK_NEAR(blob->cpu_data()[tmp], (gt.get())[idx], 1e-4);
//         idx++;
//       }
//     }
//   }
// }

// TEST(RTNetBatchsizeTest, test) {
//   std::vector<std::string> file_list =
//   {"ARZ034_12_1499218335_1499218635_500",
//                                         "ARZ034_12_1499218335_1499218635_520",
//                                         "ARZ034_12_1499218335_1499218635_540",
//                                         "ARZ034_12_1499218335_1499218635_560",
//                                         "ARZ034_12_1499218335_1499218635_580",
//                                         "ARZ034_12_1499218335_1499218635_606",
//                                         "ARZ034_12_1499218335_1499218635_834",
//                                         "ARZ034_12_1499218335_1499218635_854",
//                                         "ARZ034_12_1499218335_1499218635_874",
//                                         "ARZ034_12_1499218335_1499218635_894"};
//   std::string image_root =
//       "modules/perception/inference/inference_test_data/images/";
//   std::string image_ext = ".jpg";
//   std::string output_blob_path =
//       "modules/perception/inference/inference_test_data/yolo/blob_names.txt";
//   std::string proto_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.pt";
//   std::string weight_file =
//       "modules/perception/inference/inference_test_data/yolo/caffe.caffemodel";
//   std::string result_file =
//       "modules/perception/inference/inference_test_data/yolo/result.dat";
//   std::string input_name = "input";
//   std::string input_blob_name = "data";
//   std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
//                                    "ori_pred", "dim_pred", "lof_pred",
//                                    "lor_pred"};
//   apollo::perception::inference::load_data<std::string>(output_blob_path,
//                                                         &outputs);
//   apollo::perception::inference::Inference *rt_net = nullptr;
//   std::vector<std::string> inputs{"data"};

//   rt_net = apollo::perception::inference::CreateInferenceByName(
//       "RTNet", proto_file, weight_file, outputs, inputs);
//   CHECK_NOTNULL(rt_net);

//   int height = 576;
//   int width = 1440;
//   int offset_y = 312;
//   int batchsize = 4;
//   std::vector<int> shape = {batchsize, height, width, 3};
//   std::map<std::string, std::vector<int>> shape_map{{input_blob_name,
//   shape}}; rt_net->set_max_batch_size(batchsize); rt_net->Init(shape_map);
//   auto input_blob = rt_net->get_blob(input_blob_name);
//   auto gt = apollo::perception::inference::load_binary_data(result_file);
//   int idx = 0;
//   for (int img_idx = 0;
//        img_idx < (file_list.size() + batchsize - 1) / batchsize; img_idx++) {
//     auto input = input_blob->mutable_cpu_data();
//     int start = img_idx * batchsize;
//     int tmp_batchsize = file_list.size() > (start + batchsize)
//                             ? batchsize
//                             : (file_list.size() - start);
//     for (int i = start; i < start + tmp_batchsize; i++) {
//       std::string image_file = file_list[i];
//       int offset = input_blob->offset({i % batchsize});
//       cv::Mat img = cv::imread(image_root + image_file + image_ext);
//       cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
//       cv::Mat img_roi = img(roi);
//       img_roi.copyTo(img);
//       cv::resize(img, img, cv::Size(width, height));
//       for (int j = 0; j < input_blob->count(1); j++) {
//         input[offset + j] = img.data[j];
//       }
//     }
//     cudaDeviceSynchronize();
//     rt_net->Infer();
//     cudaDeviceSynchronize();
//     for (int b = 0; b < tmp_batchsize; b++) {
//       for (auto output_name : outputs) {
//         auto blob = rt_net->get_blob(output_name);
//         for (int tmp = 0; tmp < blob->count(1); tmp++) {
//           int offset = blob->offset({b});
//           EXPECT_NEAR(blob->cpu_data()[offset + tmp], (gt.get())[idx], 1e-4)
//               << " idx: " << idx;
//           idx++;
//         }
//       }
//     }
//   }
//   delete rt_net;
// }

}  // namespace inference
}  // namespace perception
}  // namespace apollo
