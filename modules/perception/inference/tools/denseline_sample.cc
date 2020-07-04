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
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/tensorrt/batch_stream.h"
#include "modules/perception/inference/tensorrt/entropy_calibrator.h"
#include "modules/perception/inference/tensorrt/rt_net.h"
#include "modules/perception/inference/utils/util.h"

DEFINE_bool(int8, false, "use int8");
DEFINE_string(names_file, "./denseline_parser/blob_names.txt",
              "path of output blob");
DEFINE_string(test_list, "image_list.txt", "path of image list");
DEFINE_string(image_root, "./images/", "path of image dir");
DEFINE_string(image_ext, ".jpg", "path of image ext");
DEFINE_string(res_dir, "./result.dat", "path of result");

DEFINE_int32(height, 512, "input blob height");
DEFINE_int32(width, 1536, "input blob width");
DEFINE_int32(offset_y, 440, "image offset of height");
DEFINE_int32(image_channel, 3, "image channel");
DEFINE_bool(has_mean_bgr, false, "has image b g r.");
DEFINE_int32(mean_b, 95, "image b");
DEFINE_int32(mean_g, 99, "image g");
DEFINE_int32(mean_r, 96, "image r");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  const int height = FLAGS_height;
  const int width = FLAGS_width;
  const int count = FLAGS_image_channel * width * height;

  const std::string proto_file = "./denseline_parser/deploy.prototxt";
  const std::string weight_file = "./denseline_parser/deploy.caffemodel";
  const std::string model_root = "./denseline_parser/";

  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, 0);
  AINFO << prop.name;

  apollo::perception::inference::Inference *rt_net;
  const std::string input_blob_name = "data";
  std::vector<std::string> inputs{input_blob_name};

  std::vector<std::string> outputs;
  apollo::perception::inference::load_data<std::string>(FLAGS_names_file,
                                                        &outputs);
  for (auto name : outputs) {
    AINFO << "outputs name: " << name;
  }

  apollo::perception::inference::BatchStream stream(2, 50, "./batches/");
  nvinfer1::Int8EntropyCalibrator *calibrator =
      new nvinfer1::Int8EntropyCalibrator(stream, 0, true, "./");
  if (FLAGS_int8) {
    AINFO << "int8";
    rt_net = new apollo::perception::inference::RTNet(
        proto_file, weight_file, outputs, inputs, calibrator);
  } else {
    AINFO << "fp32";
    rt_net = apollo::perception::inference::CreateInferenceByName(
        "RTNet", proto_file, weight_file, outputs, inputs);
  }
  std::vector<int> shape = {1, 3, height, width};
  std::map<std::string, std::vector<int>> shape_map{{input_blob_name, shape}};

  ACHECK(rt_net->Init(shape_map));

  auto input_blob = rt_net->get_blob(input_blob_name);
  std::vector<std::string> image_lists;
  apollo::perception::inference::load_data<std::string>(FLAGS_test_list,
                                                        &image_lists);
  std::vector<float> output_data_vec;

  for (auto image_file : image_lists) {
    cv::Mat img = cv::imread(FLAGS_image_root + image_file + FLAGS_image_ext);
    AINFO << img.channels();
    cv::Rect roi(0, FLAGS_offset_y, img.cols, img.rows - FLAGS_offset_y);
    cv::Mat img_roi = img(roi);
    img_roi.copyTo(img);
    cv::resize(img, img, cv::Size(width, height));

    // int count = 3 * width * height;
    std::vector<float> input(count);
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        input[(0 * height + i) * width + j] =
            img.at<cv::Vec3b>(i, j)[0] - static_cast<float>(FLAGS_mean_b);
        input[(1 * height + i) * width + j] =
            img.at<cv::Vec3b>(i, j)[1] - static_cast<float>(FLAGS_mean_g);
        input[(2 * height + i) * width + j] =
            img.at<cv::Vec3b>(i, j)[2] - static_cast<float>(FLAGS_mean_r);
      }
    }
    cudaMemcpy(input_blob->mutable_gpu_data(), input.data(),
               count * sizeof(float), cudaMemcpyHostToDevice);
    cudaDeviceSynchronize();
    rt_net->Infer();
    cudaDeviceSynchronize();

    for (auto output_name : outputs) {
      auto blob = rt_net->get_blob(output_name);
      std::vector<float> tmp_vec(blob->cpu_data(),
                                 blob->cpu_data() + blob->count());
      // if(output_name=="conv2" || output_name == "conv1") {
      AINFO << output_name << " " << blob->channels() << " " << blob->height()
            << " " << blob->width();
      double sum = 0;
      for (int i = 0; i < blob->count(); ++i) {
        sum += blob->cpu_data()[i];
        if (i < 100) {
          AINFO << blob->cpu_data()[i];
        }
      }
      AINFO << output_name << ", sum : " << std::endl;
      // end of if

      output_data_vec.insert(output_data_vec.end(), tmp_vec.begin(),
                             tmp_vec.end());
    }
  }
  apollo::perception::inference::write_result(FLAGS_res_dir, output_data_vec);
  delete rt_net;
  // rt_net has deletle calibrator?
  // delete calibrator;

  return 0;
}
