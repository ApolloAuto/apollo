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
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include "gtest/gtest.h"

#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/utils/util.h"
TEST(RTNetBatchsizeTest, test) {
  std::vector<std::string> file_list = {"ARZ034_12_1499218335_1499218635_500",
                                        "ARZ034_12_1499218335_1499218635_520",
                                        "ARZ034_12_1499218335_1499218635_540",
                                        "ARZ034_12_1499218335_1499218635_560",
                                        "ARZ034_12_1499218335_1499218635_580",
                                        "ARZ034_12_1499218335_1499218635_606",
                                        "ARZ034_12_1499218335_1499218635_834",
                                        "ARZ034_12_1499218335_1499218635_854",
                                        "ARZ034_12_1499218335_1499218635_874",
                                        "ARZ034_12_1499218335_1499218635_894"};
  std::string image_root = "./inference_test_data/images/";
  std::string image_ext = ".jpg";
  std::string output_blob_path = "inference_test_data/yolo/blob_names.txt";
  std::string proto_file = "./inference_test_data/yolo/caffe.pt";
  std::string weight_file = "./inference_test_data/yolo/caffe.caffemodel";
  std::string result_file = "inference_test_data/yolo/result.dat";
  std::string input_name = "input";
  std::string input_blob_name = "data";
  std::vector<std::string> outputs{"loc_pred", "obj_pred", "cls_pred",
                                   "ori_pred", "dim_pred", "lof_pred",
                                   "lor_pred"};
  apollo::perception::inference::load_data<std::string>(output_blob_path,
                                                        &outputs);
  apollo::perception::inference::Inference *rt_net = nullptr;
  std::vector<std::string> inputs{"data"};

  rt_net = apollo::perception::inference::CreateInferenceByName(
      "RTNet", proto_file, weight_file, outputs, inputs);
  CHECK(rt_net != nullptr);

  int height = 576;
  int width = 1440;
  int offset_y = 312;
  int batchsize = 4;
  std::vector<int> shape = {batchsize, height, width, 3};
  std::map<std::string, std::vector<int>> shape_map{{input_blob_name, shape}};
  rt_net->set_max_batch_size(batchsize);
  rt_net->Init(shape_map);
  auto input_blob = rt_net->get_blob(input_blob_name);
  auto gt = apollo::perception::inference::load_binary_data(result_file);
  int idx = 0;
  for (int img_idx = 0;
       img_idx < (file_list.size() + batchsize - 1) / batchsize; img_idx++) {
    auto input = input_blob->mutable_cpu_data();
    int start = img_idx * batchsize;
    int tmp_batchsize = file_list.size() > (start + batchsize)
                            ? batchsize
                            : (file_list.size() - start);
    for (int i = start; i < start + tmp_batchsize; i++) {
      std::string image_file = file_list[i];
      int offset = input_blob->offset({i % batchsize});
      cv::Mat img = cv::imread(image_root + image_file + image_ext);
      cv::Rect roi(0, offset_y, img.cols, img.rows - offset_y);
      cv::Mat img_roi = img(roi);
      img_roi.copyTo(img);
      cv::resize(img, img, cv::Size(width, height));
      for (int j = 0; j < input_blob->count(1); j++) {
        input[offset + j] = img.data[j];
      }
    }
    cudaDeviceSynchronize();
    rt_net->Infer();
    cudaDeviceSynchronize();
    for (int b = 0; b < tmp_batchsize; b++) {
      for (auto output_name : outputs) {
        auto blob = rt_net->get_blob(output_name);
        for (int tmp = 0; tmp < blob->count(1); tmp++) {
          int offset = blob->offset({b});
          EXPECT_NEAR(blob->cpu_data()[offset + tmp], (gt.get())[idx], 1e-4)
              << " idx: " << idx;
          idx++;
        }
      }
    }
  }
  delete rt_net;
}
