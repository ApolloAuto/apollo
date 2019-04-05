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
#include "modules/perception/lidar/lib/segmentation/cnnseg/feature_generator.h"

#include "opencv2/opencv.hpp"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/segmentation/cnnseg/util.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}

namespace lidar {

class FeatureGeneratorTest : public ::testing::Test {
 public:
  virtual void SetUp() {}

  void MapPointToGrid(const base::PointFCloudPtr& pc_ptr,
                      std::vector<int>* point2grid, float range, size_t width,
                      size_t height, float min_height, float max_height) {
    float inv_res_x = 0.5 * static_cast<float>(width) / range;
    float inv_res_y = 0.5 * static_cast<float>(height) / range;
    point2grid->assign(pc_ptr->size(), -1);
    for (size_t i = 0; i < pc_ptr->size(); ++i) {
      const auto& pt = pc_ptr->at(i);
      if (pt.z <= min_height || pt.z >= max_height) {
        continue;
      }
      // the coordinates of x and y are exchanged here
      // (row <-> x, column <-> y)
      int pos_x = F2I(pt.y, range, inv_res_x);  // col
      int pos_y = F2I(pt.x, range, inv_res_y);  // row
      if (pos_y < 0 || pos_y >= static_cast<int>(height) || pos_x < 0 ||
          pos_x >= static_cast<int>(width)) {
        continue;
      }
      point2grid->at(i) = pos_y * width + pos_x;
    }
  }

  void InitCPUBlobs(bool use_intensity_feature) {
    base::Blob<float>* out_blob = generator_->out_blob_;
    float* out_blob_data = nullptr;
    out_blob_data = out_blob->mutable_cpu_data();

    // set raw feature data
    int channel_index = 0;
    generator_->max_height_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    generator_->mean_height_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    generator_->count_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    generator_->direction_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    if (use_intensity_feature) {
      generator_->top_intensity_data_ =
          out_blob_data + out_blob->offset(0, channel_index++);
      generator_->mean_intensity_data_ =
          out_blob_data + out_blob->offset(0, channel_index++);
    }
    generator_->distance_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    generator_->nonempty_data_ =
        out_blob_data + out_blob->offset(0, channel_index++);
    CHECK_EQ(out_blob->offset(0, channel_index), out_blob->count());

    // compute direction and distance features
    int map_size = generator_->height_ * generator_->width_;
    std::vector<float> direction_data(map_size);
    std::vector<float> distance_data(map_size);
    for (int row = 0; row < generator_->height_; ++row) {
      for (int col = 0; col < generator_->width_; ++col) {
        int idx = row * generator_->width_ + col;
        // * row <-> x, column <-> y
        float center_x = Pixel2Pc(row, generator_->height_, generator_->range_);
        float center_y = Pixel2Pc(col, generator_->width_, generator_->range_);
        direction_data[idx] =
            static_cast<float>(std::atan2(center_y, center_x) / (2.0 * kPI));
        distance_data[idx] =
            static_cast<float>(std::hypot(center_x, center_y) / 60.0 - 0.5);
      }
    }

    // memory copy direction and distance features
    memcpy(generator_->direction_data_, direction_data.data(),
           direction_data.size() * sizeof(float));
    memcpy(generator_->distance_data_, distance_data.data(),
           distance_data.size() * sizeof(float));
  }

 protected:
  std::unique_ptr<FeatureGenerator> generator_;
};

TEST_F(FeatureGeneratorTest, basic_test) {
  char cyber_path[100] = "CYBER_PATH=";
  putenv(cyber_path);
  char module_path[100] = "MODULE_PATH=";
  putenv(module_path);
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "lidar/lib/segmentation/cnnseg/";

  // load pcd data
  base::PointFCloudPtr pc_ptr;
  pc_ptr.reset(new base::PointFCloud);
  std::string filename =
      "/apollo/modules/perception/testdata/lidar/lib/segmentation/cnnseg/"
      "pcd_data/3_car_1_person.pcd";
  bool ret = LoadPCLPCD(filename, pc_ptr.get());
  CHECK(ret) << "Failed to load " << filename;

  std::vector<int> point2grid;
  float range = 60.f;
  size_t width = 640;
  size_t height = 640;
  float min_height = -5.f;
  float max_height = 5.f;
  MapPointToGrid(pc_ptr, &point2grid, range, width, height, min_height,
                 max_height);

  FeatureParam param;
  param.set_point_cloud_range(range);
  param.set_width(width);
  param.set_height(height);
  param.set_min_height(min_height);
  param.set_max_height(max_height);
  param.set_use_intensity_feature(true);

  // cpu generator test
  {
    generator_.reset(new FeatureGenerator);
    base::Blob<float> feature_blob;
    feature_blob.Reshape(1, 8, param.height(), param.width());
    EXPECT_TRUE(generator_->Init(param, &feature_blob));
    InitCPUBlobs(true);
    generator_->GenerateCPU(pc_ptr, point2grid);
    EXPECT_NE(generator_->mean_intensity_data_, nullptr);
    EXPECT_NE(generator_->top_intensity_data_, nullptr);
    // save feature map
    std::ofstream ofs("cpu_top_intensity_map.txt");
    CHECK(ofs.is_open());
    for (size_t i = 0; i < param.height() * param.width(); ++i) {
      ofs << generator_->top_intensity_data_[i] << std::endl;
    }
    ofs.close();
    cv::Mat image(param.height(), param.width(), CV_32FC1,
                  generator_->top_intensity_data_);
    image.convertTo(image, CV_32FC3, 255.0);
    cv::imwrite("cpu_top_intensity_map.jpg", image);
  }
  {
    generator_.reset(new FeatureGenerator);
    base::Blob<float> feature_blob;
    param.set_use_intensity_feature(false);
    feature_blob.Reshape(1, 6, param.height(), param.width());
    EXPECT_TRUE(generator_->Init(param, &feature_blob));
    InitCPUBlobs(false);
    generator_->GenerateCPU(pc_ptr, point2grid);
    EXPECT_EQ(generator_->mean_intensity_data_, nullptr);
    EXPECT_EQ(generator_->top_intensity_data_, nullptr);
  }
  // gpu generator test
  {
    generator_.reset(new FeatureGenerator);
    base::Blob<float> feature_blob;
    param.set_use_intensity_feature(true);
    feature_blob.Reshape(1, 8, param.height(), param.width());
    EXPECT_TRUE(generator_->Init(param, &feature_blob));
    generator_->GenerateGPU(pc_ptr, point2grid);
    EXPECT_NE(generator_->mean_intensity_data_, nullptr);
    EXPECT_NE(generator_->top_intensity_data_, nullptr);
    // save feature map
    size_t map_size = param.height() * param.width();
    std::vector<float> top_intensity_data(map_size);
    cudaMemcpy(top_intensity_data.data(), generator_->top_intensity_data_,
               map_size * sizeof(float), cudaMemcpyDeviceToHost);
    std::ofstream ofs("gpu_top_intensity_map.txt");
    CHECK(ofs.is_open());
    for (size_t i = 0; i < param.height() * param.width(); ++i) {
      ofs << top_intensity_data[i] << std::endl;
    }
    ofs.close();
    cv::Mat image(param.height(), param.width(), CV_32FC1,
                  top_intensity_data.data());
    image.convertTo(image, CV_32FC3, 255.0);
    cv::imwrite("gpu_top_intensity_map.jpg", image);
  }
  {
    generator_.reset(new FeatureGenerator);
    base::Blob<float> feature_blob;
    param.set_use_intensity_feature(false);
    feature_blob.Reshape(1, 6, param.height(), param.width());
    EXPECT_TRUE(generator_->Init(param, &feature_blob));
    generator_->GenerateGPU(pc_ptr, point2grid);
    EXPECT_EQ(generator_->mean_intensity_data_, nullptr);
    EXPECT_EQ(generator_->top_intensity_data_, nullptr);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
