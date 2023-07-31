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

#include "cyber/common/log.h"
#include "modules/perception/common/inference/inference.h"
#include "modules/perception/common/inference/tensorrt/batch_stream.h"
#include "modules/perception/common/inference/tensorrt/entropy_calibrator.h"
#include "modules/perception/common/inference/tensorrt/rt_net.h"
#include "modules/perception/common/inference/utils/util.h"

DEFINE_bool(int8, false, "use int8");
DEFINE_string(names_file, "./lane_parser/blob_names.txt",
              "path of output blob");
DEFINE_string(test_list, "image_list.txt", "path of image list");
DEFINE_string(image_root, "./images/", "path of image dir");
DEFINE_string(image_ext, ".jpg", "path of image ext");
DEFINE_string(res_dir, "./result.dat", "path of result");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::vector<cv::Scalar> color_table;
  color_table.push_back(cv::Scalar(0, 97, 255));    // for other >0 mask values
  color_table.push_back(cv::Scalar(0, 0, 255));     // for mask value = 1
  color_table.push_back(cv::Scalar(0, 255, 0));     // for mask value = 2
  color_table.push_back(cv::Scalar(255, 0, 0));     // for mask value = 3
  color_table.push_back(cv::Scalar(240, 32, 160));  // for mask value = 4

  const std::string proto_file = "./lane_parser/caffe.pt";
  const std::string weight_file = "./lane_parser/caffe.caffemodel";
  const std::string model_root = "./lane_parser/";

  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, 0);
  AINFO << prop.name;

  apollo::perception::inference::Inference *rt_net;
  const std::string input_blob_name = "data";
  std::vector<std::string> inputs{"data"};

  std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
                                   "ori_pred", "dim_pred", "lof_pred",
                                   "lor_pred", "conv4_3"};

  apollo::perception::inference::load_data<std::string>(FLAGS_names_file,
                                                        &outputs);
  for (auto &name : outputs) {
    ADEBUG << name;
  }

  if (FLAGS_int8) {
    apollo::perception::inference::BatchStream stream(2, 50, "./batches/");
    nvinfer1::Int8EntropyCalibrator calibrator(stream, 0, true, "./");
    rt_net = new apollo::perception::inference::RTNet(
        proto_file, weight_file, outputs, inputs, model_root);
  } else {
    rt_net = new apollo::perception::inference::RTNet(proto_file, weight_file,
                                                      outputs, inputs);
  }
  const int height = 608;
  const int width = 1024;
  const int offset_y = 0;
  std::vector<int> shape = {1, 3, height, width};
  std::map<std::string, std::vector<int>> shape_map{{input_blob_name, shape}};

  rt_net->Init(shape_map);

  auto input_blob = rt_net->get_blob("data");
  std::vector<std::string> image_lists;
  apollo::perception::inference::load_data<std::string>(FLAGS_test_list,
                                                        &image_lists);
  std::vector<float> output_data_vec;

  const int count = width * height;
  for (auto &image_file : image_lists) {
    cv::Mat img =
        cv::imread(FLAGS_image_root + image_file + FLAGS_image_ext, CV_8UC1);
    ADEBUG << img.channels();
    cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
    cv::Mat img_roi = img(roi);
    img_roi.copyTo(img);
    cv::resize(img, img, cv::Size(width, height));

    std::vector<float> input(count);
    for (int i = 0; i < count; ++i) {
      input[i] = static_cast<float>(img.data[i] - 128) * 0.0078125f;
    }
    cudaMemcpy(input_blob->mutable_gpu_data(), &input[0], count * sizeof(float),
               cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    rt_net->Infer();
    cudaDeviceSynchronize();

    for (auto &output_name : outputs) {
      auto blob = rt_net->get_blob(output_name);
      std::vector<float> tmp_vec(blob->cpu_data(),
                                 blob->cpu_data() + blob->count());
      if (output_name == "output") {
        const float *output_data = blob->cpu_data();
        for (int h = 0; h < img.rows; h++) {
          for (int w = 0; w < img.cols; w++) {
            int offset = blob->offset(0, 0, h, w);
            if (output_data[offset] > 0) {
              img.at<cv::Vec3b>(h, w)[0] = static_cast<unsigned char>(
                  img.at<cv::Vec3b>(w, h)[0] * 0.7 + color_table[0][0] * 0.3);
            }
          }
        }
        cv::imwrite("res.jpg", img);
      }
      output_data_vec.insert(output_data_vec.end(), tmp_vec.begin(),
                             tmp_vec.end());
    }
  }

  apollo::perception::inference::write_result(FLAGS_res_dir, output_data_vec);
  delete rt_net;

  return 0;
}
