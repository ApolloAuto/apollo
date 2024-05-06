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

#include "gflags/gflags.h"
#include "opencv2/opencv.hpp"

#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/tensorrt/batch_stream.h"
#include "modules/perception/common/inference/tensorrt/entropy_calibrator.h"
#include "modules/perception/common/inference/tensorrt/rt_net.h"
#include "modules/perception/common/inference/utils/util.h"

DEFINE_bool(int8, false, "use int8");
DEFINE_string(output_blob, "loc_pred", "name of output blob");
DEFINE_string(names_file, "./yolo/blob_names.txt", "path of output blob");
DEFINE_string(test_list, "image_list.txt", "path of image list");
DEFINE_string(image_root, "./images/", "path of image dir");
DEFINE_string(image_ext, ".jpg", "path of image ext");
DEFINE_string(res_dir, "./result.dat", "path of result");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  const std::string proto_file = "./yolo/caffe.pt";
  const std::string weight_file = "./yolo/caffe.caffemodel";
  const std::string model_root = "./yolo";

  apollo::perception::inference::Inference *rt_net;
  std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
                                   "ori_pred", "dim_pred", "lof_pred",
                                   "lor_pred", "conv4_3"};
  std::vector<std::string> inputs{"data"};
  apollo::perception::inference::load_data<std::string>(FLAGS_names_file,
                                                        &outputs);

  std::shared_ptr<nvinfer1::Int8EntropyCalibrator> calibrator = nullptr;
  if (FLAGS_int8) {
    apollo::perception::inference::BatchStream stream(2, 100, "./batches/");
    calibrator.reset(
        new nvinfer1::Int8EntropyCalibrator(stream, 0, true, "./"));
    rt_net = new apollo::perception::inference::RTNet(
        proto_file, weight_file, outputs, inputs, calibrator.get());
  } else {
    rt_net = new apollo::perception::inference::RTNet(proto_file, weight_file,
                                                      outputs, inputs);
  }
  const int height = 576;
  const int width = 1440;
  const int offset_y = 312;
  std::vector<int> shape = {1, height, width, 3};
  std::map<std::string, std::vector<int>> shape_map{{"data", shape}};
  rt_net->Init(shape_map);

  //  auto input_gpu = rt_net->get_blob("data")->mutable_gpu_data();
  std::vector<std::string> image_lists;
  apollo::perception::inference::load_data<std::string>(FLAGS_test_list,
                                                        &image_lists);

  const int count = 3 * width * height;
  std::vector<float> output_data_vec;
  for (auto &image_file : image_lists) {
    cv::Mat img = cv::imread(FLAGS_image_root + image_file + FLAGS_image_ext);
    cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
    cv::Mat img_roi = img(roi);
    img_roi.copyTo(img);
    cv::resize(img, img, cv::Size(width, height));
    auto input = rt_net->get_blob("data")->mutable_cpu_data();
    for (int i = 0; i < count; ++i) {
      input[i] = img.data[i];
    }
    cudaDeviceSynchronize();
    rt_net->Infer();
    cudaDeviceSynchronize();
    for (auto output_name : outputs) {
      std::vector<int> shape;
      auto blob = rt_net->get_blob(output_name);
      const int size = blob->count();
      std::vector<float> tmp_vec(blob->cpu_data(), blob->cpu_data() + size);
      output_data_vec.insert(output_data_vec.end(), tmp_vec.begin(),
                             tmp_vec.end());
    }
  }
  apollo::perception::inference::write_result(FLAGS_res_dir, output_data_vec);
  delete rt_net;
  return 0;
}
