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

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "opencv2/opencv.hpp"

#include "cyber/common/log.h"
#include "modules/perception/inference/inference.h"
#include "modules/perception/inference/tensorrt/batch_stream.h"
#include "modules/perception/inference/tensorrt/entropy_calibrator.h"
#include "modules/perception/inference/tensorrt/rt_net.h"
#include "modules/perception/inference/utils/util.h"

DEFINE_string(names_file, "blob_names.txt", "path of output blob");
DEFINE_string(image_root, "./images/", "image dir");
DEFINE_bool(gen_batch, false, "generate batch data");
DEFINE_bool(rgb, true, "model support rgb or bgr");
DEFINE_string(batch_root, "./batches/", "batch dir");
DEFINE_int32(batch_size, 2, "num of images per batch");
DEFINE_int32(max_batch, 20, "total num of batches");
DEFINE_string(cal_table_root, "./", "calibration table root");
DEFINE_int32(height, 576, "num of images per batch");
DEFINE_int32(width, 1440, "num of images per batch");
DEFINE_string(image_ext, ".jpg", "image ext");
DEFINE_string(test_list, "./valid.txt", "image list");
DEFINE_string(proto_file, "caffe.pt", "model file");
DEFINE_string(weight_file, "caffe.caffemodel", "weight file");
DEFINE_int32(offset_y, 312, "offset of height");
DEFINE_int32(image_channel_num, 3, "image channel num");
DEFINE_int32(mean_b, 0, "image b");
DEFINE_int32(mean_g, 0, "image g");
DEFINE_int32(mean_r, 0, "image r");
DEFINE_bool(hwc_input, true, "input blob is hwc order.");

int evaluate_image_list() {
  CHECK_EQ(FLAGS_image_channel_num, 3);
  const int height = FLAGS_height;
  const int width = FLAGS_width;
  const int count = FLAGS_image_channel_num * width * height;

  std::ifstream fin;
  fin.open(FLAGS_test_list, std::ifstream::in);
  if (!fin.is_open()) {
    AERROR << "Failed to open test list file: " << FLAGS_test_list;
    return -1;
  }
  std::string image_name;
  std::vector<std::string> img_list;
  int i = 0;
  while (fin >> image_name) {
    std::string image_path = FLAGS_image_root + '/' + image_name;
    img_list.push_back(image_path);
    ++i;
    if (i >= FLAGS_max_batch * FLAGS_batch_size) {
      break;
    }
  }
  fin.close();

  std::string out_file = FLAGS_batch_root + "/Batch0";
  std::ofstream out_car(out_file, std::ofstream::out | std::ofstream::binary);
  // std::ofstream out_car(out_file, std::ofstream::out);
  if (!out_car.is_open()) {
    AERROR << "Failed to open out car file: " << out_file;
    return -1;
  }
  std::vector<float> cpu_data(count);
  // Main loop
  for (size_t i = 0; i < img_list.size(); ++i) {
    std::string image_path = img_list[i] + FLAGS_image_ext;
    cv::Mat img = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    cv::Mat img_org;

    img.copyTo(img_org);

    cv::Rect roi(0, FLAGS_offset_y, img.cols, img.rows - FLAGS_offset_y);
    cv::Mat img_roi = img(roi);
    img_roi.copyTo(img);
    int image_c0 = FLAGS_mean_b;
    int image_c1 = FLAGS_mean_g;
    int image_c2 = FLAGS_mean_r;
    if (FLAGS_rgb) {
      cv::cvtColor(img, img, CV_BGR2RGB);
      image_c0 = FLAGS_mean_r;
      image_c1 = FLAGS_mean_g;
      image_c2 = FLAGS_mean_b;
    }
    if (img.data == 0) {
      AERROR << "Failed to read image: " << image_path;
      return -1;
    }
    cv::resize(img, img, cv::Size(width, height));

    // Prepare the input data
    if (i % FLAGS_batch_size == 0) {
      out_car.close();
      out_file = absl::StrCat(FLAGS_batch_root, "Batch", i / FLAGS_batch_size);

      out_car.open(out_file, std::ofstream::out | std::ofstream::binary);
      if (!out_car.is_open()) {
        AERROR << "Failed to open out car file: " << out_file;
        return -1;
      }

      int num =
          std::min(static_cast<int>(img_list.size() - i), FLAGS_batch_size);
      int channels = img.channels();
      int rows = img.rows;
      int cols = img.cols;
      out_car.write((const char *)&num, sizeof(int));
      out_car.write((const char *)(&channels), sizeof(int));
      out_car.write((const char *)(&rows), sizeof(int));
      out_car.write((const char *)(&cols), sizeof(int));
    }
    if (FLAGS_hwc_input) {
      for (int idxx = 0, idx = 0; idx < count / FLAGS_image_channel_num;
           idx++) {
        idxx = idx * FLAGS_image_channel_num;
        cpu_data[idxx] = static_cast<float>(img.data[idxx] - image_c0);
        cpu_data[idxx + 1] = static_cast<float>(img.data[idxx + 1] - image_c1);
        cpu_data[idxx + 2] = static_cast<float>(img.data[idxx + 2] - image_c2);
      }
    } else {
      for (int ri = 0; ri < height; ++ri) {
        int row_idx0 = (0 * height + ri) * width;
        int row_idx1 = (1 * height + ri) * width;
        int row_idx2 = (2 * height + ri) * width;
        auto img_ptr = img.ptr<cv::Vec3b>(ri);
        for (int cj = 0; cj < width; ++cj) {
          cpu_data[row_idx0 + cj] =
              static_cast<float>(img_ptr[cj][0] - image_c0);
          cpu_data[row_idx1 + cj] =
              static_cast<float>(img_ptr[cj][1] - image_c1);
          cpu_data[row_idx2 + cj] =
              static_cast<float>(img_ptr[cj][2] - image_c2);
        }
      }
    }
    out_car.write((const char *)(cpu_data.data()), count * sizeof(float));
  }

  fin.clear();
  out_car.close();
  return 0;
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_gen_batch) {
    evaluate_image_list();
  }
  std::vector<std::string> outputs;
  std::vector<std::string> inputs = {"data"};

  apollo::perception::inference::load_data<std::string>(FLAGS_names_file,
                                                        &outputs);
  apollo::perception::inference::BatchStream stream(2, FLAGS_max_batch,
                                                    FLAGS_batch_root);
  nvinfer1::Int8EntropyCalibrator *calibrator =
      (new nvinfer1::Int8EntropyCalibrator(stream, 0, true,
                                           FLAGS_cal_table_root));
  apollo::perception::inference::RTNet *rt_net =
      new apollo::perception::inference::RTNet(
          FLAGS_proto_file, FLAGS_weight_file, outputs, inputs, calibrator);
  rt_net->Init(std::map<std::string, std::vector<int>>());
  rt_net->Infer();
  delete rt_net;
  delete calibrator;

  return 0;
}
