/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/common/inference/migraphx/mi_net.h"

#include <csignal>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace inference {

const std::string mi =
    "modules/perception/common/inference/inference_test_data/migraphx/";

struct blob {
  size_t size;
  std::shared_ptr<char> data;
};

struct network_param {
  std::string path;
  std::string model_filename;
  std::string weight_filename;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  std::vector<std::vector<int>> input_shapes;
};

class NetworkInferenceTests : public ::testing::TestWithParam<network_param> {
 protected:
  std::string path;
  std::string model_filename;
  std::string weight_filename;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  std::vector<std::vector<int>> input_shapes;

  std::map<std::string, std::vector<int>> input_shapes_map;
  std::vector<blob> input_data;
  std::vector<blob> output_data;
  int batchsize = 1;

  NetworkInferenceTests()
      : path{GetParam().path},
        model_filename{GetParam().model_filename},
        weight_filename{GetParam().weight_filename},
        input_names{GetParam().input_names},
        output_names{GetParam().output_names},
        input_shapes{GetParam().input_shapes} {}

  void SetUp() override {
    CHECK_EQ(input_names.size(), input_shapes.size());

    for (size_t i = 0; i < input_names.size(); ++i) {
      input_shapes_map[input_names[i]] = input_shapes[i];
      batchsize = std::max(batchsize, input_shapes[i][0]);
    }

    std::string bc_suffix = "";
    if (batchsize > 1) {
      bc_suffix = "_bc" + std::to_string(batchsize);
    }

    for (auto &n : input_names) {
      auto data = load_binary_data(path + n + bc_suffix);
      input_data.push_back(data);
    }

    for (auto &n : output_names) {
      auto data = load_binary_data(path + n + bc_suffix + "_gd");
      output_data.push_back(data);
    }
  }

 private:
  blob load_binary_data(const std::string &filename) {
    std::ifstream ifs(filename, std::ifstream::binary);

    EXPECT_TRUE(ifs);

    ifs.seekg(0, ifs.end);
    size_t size = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    blob blob;
    blob.size = size;
    blob.data.reset(new char[size]);

    ifs.read(reinterpret_cast<char *>(blob.data.get()), size);
    ifs.close();

    return blob;
  }
};

TEST_P(NetworkInferenceTests, InferenceFP32) {
  // clang-format off
  auto net = std::make_unique<MINet>(path + model_filename,
                                     path + weight_filename,
                                     output_names,
                                     input_names);
  // clang-format on

  net->Init(input_shapes_map);

  for (size_t i = 0; i < input_names.size(); ++i) {
    auto input_gd = input_data[i];
    auto input_blob = net->get_blob(input_names[i]);

    EXPECT_NE(input_blob.get(), nullptr);
    EXPECT_EQ(input_blob->count(), input_gd.size / sizeof(float));

    auto input_ptr = input_blob->mutable_cpu_data();

    std::memcpy(input_ptr, input_gd.data.get(), input_gd.size);

    std::stringstream ss;
    ss << "Input " << input_names[i] << ": ";
    for (size_t k = 0; k < 10; ++k) {
      ss << input_ptr[k] << ", ";
    }
    AINFO << ss.str();
  }

  cudaDeviceSynchronize();
  net->Infer();
  cudaDeviceSynchronize();

  for (size_t i = 0; i < output_names.size(); ++i) {
    auto output_gd = output_data[i];
    auto output_gd_ptr = reinterpret_cast<float *>(output_gd.data.get());

    const auto output_name = output_names[i];
    const auto output_blob = net->get_blob(output_name);
    EXPECT_NE(output_blob.get(), nullptr);
    EXPECT_EQ(output_blob->count(), output_gd.size / sizeof(float));

    const auto output_ptr = output_blob->mutable_cpu_data();

    size_t err_cnt = 0;
    std::stringstream ss;
    for (auto i = 0; i < output_blob->count(); ++i) {
      EXPECT_NEAR(output_ptr[i], output_gd_ptr[i], 1e-1)
          << output_name << " out: " << output_ptr[i]
          << " expected: " << output_gd_ptr[i] << " idx: " << i
          << " err: " << err_cnt++;
      if (i < 10)
        ss << "(" << output_ptr[i] << ", " << output_gd_ptr[i] << ") ";
      if (err_cnt > 10) break;
    }
    AINFO << output_name << ": " << ss.str();
  }
}

// clang-format off
const network_param cnnseg_velodyne16{
    mi + "cnnseg/velodyne16/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{1, 6, 672, 672}}};

const network_param cnnseg_velodyne16_bc4{
    mi + "cnnseg/velodyne16/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{4, 6, 672, 672}}};

const network_param cnnseg_velodyne64{
    mi + "cnnseg/velodyne64/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{1, 6, 672, 672}}};

const network_param cnnseg_velodyne64_bc4{
    mi + "cnnseg/velodyne64/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{4, 6, 672, 672}}};

const network_param cnnseg_velodyne128{
    mi + "cnnseg/velodyne128/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{1, 4, 864, 864}}};

const network_param cnnseg_velodyne128_bc4{
    mi + "cnnseg/velodyne128/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"category_score", "heading_pt", "class_score", "confidence_score",
     "instance_pt", "height_pt"},
    {{4, 4, 864, 864}}};

const network_param lane_detector_denseline{
    mi + "lane_detector/denseline/",
    "deploy.prototxt",
    "deploy.caffemodel",   {"data"},
    {"conv_out"},
    {{1, 3, 640, 1536}}};

const network_param lane_detector_denseline_bc4{
    mi + "lane_detector/denseline/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"conv_out"},
    {{4, 3, 640, 1536}}};

const network_param lane_detector_darkSCNN{
    mi + "lane_detector/darkSCNN/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"fc_out", "softmax"},
    {{1, 3, 480, 640}}};

const network_param lane_detector_darkSCNN_bc4{
    mi + "lane_detector/darkSCNN/",
    "deploy.prototxt",
    "deploy.caffemodel",
    {"data"},
    {"fc_out", "softmax"},
    {{4, 3, 480, 640}}};

const network_param traffic_light_detection{
    mi + "traffic_light_detection/",
    "deploy.prototxt",
    "baidu_iter_140000.caffemodel",
    {"img", "im_info"},
    {"bboxes"},
    {{1, 270, 270, 3}, {1, 6, 1, 1}}};

const network_param traffic_light_recognition_vertical{
    mi + "traffic_light_recognition/vertical/",
    "deploy.prototxt",
    "baidu_iter_250000.caffemodel",
    {"data_org"},
    {"prob"},
    {{1, 96, 32, 3}}};

const network_param traffic_light_recognition_vertical_bc4{
    mi + "traffic_light_recognition/vertical/",
    "deploy.prototxt",
    "baidu_iter_250000.caffemodel",
    {"data_org"},
    {"prob"},
    {{4, 96, 32, 3}}};

const network_param traffic_light_recognition_quadrate{
    mi + "traffic_light_recognition/quadrate/",
    "deploy.prototxt",
    "baidu_iter_200000.caffemodel",
    {"data_org"},
    {"prob"},
    {{1, 64, 64, 3}}};

const network_param traffic_light_recognition_quadrate_bc4{
    mi + "traffic_light_recognition/quadrate/",
    "deploy.prototxt",
    "baidu_iter_200000.caffemodel",
    {"data_org"},
    {"prob"},
    {{4, 64, 64, 3}}};

const network_param traffic_light_recognition_horizontal{
    mi + "traffic_light_recognition/horizontal/",
    "deploy.prototxt",
    "baidu_iter_200000.caffemodel",
    {"data_org"},
    {"prob"},
    {{1, 32, 96, 3}}};

const network_param traffic_light_recognition_horizontal_bc4{
    mi + "traffic_light_recognition/horizontal/",
    "deploy.prototxt",
    "baidu_iter_200000.caffemodel",
    {"data_org"},
    {"prob"},
    {{4, 32, 96, 3}}};

const network_param yolo_obstacle_detector_3d_r4_half{
    mi + "yolo_obstacle_detector/3d-r4-half/",
    "deploy.pt",
    "deploy.model",
    {"data"},
    {"loc_pred", "obj_pred", "cls_pred", "ori_pred",
     "dim_pred", "vis_pred", "cut_pred", "brvis_pred",
     "brswt_pred", "ltvis_pred", "ltswt_pred", "rtvis_pred",
     "rtswt_pred", "area_id_pred"},
    {{1, 800, 1440, 3}}};

const network_param yolo_obstacle_detector_3d_r4_half_bc4{
    mi + "yolo_obstacle_detector/3d-r4-half/",
    "deploy.pt",
    "deploy.model",
    {"data"},
    {"loc_pred", "obj_pred", "cls_pred", "ori_pred",
     "dim_pred", "vis_pred", "cut_pred", "brvis_pred",
     "brswt_pred", "ltvis_pred", "ltswt_pred", "rtvis_pred",
     "rtswt_pred", "area_id_pred"},
    {{4, 800, 1440, 3}}};

INSTANTIATE_TEST_SUITE_P(MINet_TEST, NetworkInferenceTests,
                      testing::Values(cnnseg_velodyne16,
                                      cnnseg_velodyne16_bc4,
                                      cnnseg_velodyne64,
                                      cnnseg_velodyne64_bc4,
                                      cnnseg_velodyne128,
                                      cnnseg_velodyne128_bc4,
                                      lane_detector_denseline,
                                      lane_detector_denseline_bc4,
                                      lane_detector_darkSCNN,
                                      lane_detector_darkSCNN_bc4,
                                      // only batchsize 1 supported
                                      traffic_light_detection,
                                      traffic_light_recognition_vertical,
                                      traffic_light_recognition_vertical_bc4,
                                      traffic_light_recognition_quadrate,
                                      traffic_light_recognition_quadrate_bc4,
                                      traffic_light_recognition_horizontal,
                                      traffic_light_recognition_horizontal_bc4,
                                      yolo_obstacle_detector_3d_r4_half,
                                      yolo_obstacle_detector_3d_r4_half_bc4));
// clang-format on

}  // namespace inference
}  // namespace perception
}  // namespace apollo
