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

#include "modules/perception/obstacle/camera/lane_post_process/cc_lane_post_processor/cc_lane_post_processor.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/util.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromFile;
using std::shared_ptr;
using std::string;
using std::vector;

DEFINE_string(test_dir,
              "/apollo/modules/perception/data/cc_lane_post_processor_test/",
              "test data directory");

static const cv::Scalar lane_mask_color(0, 255, 255);  // yellow

class CCLanePostProcessorTest : public ::testing::Test {
 protected:
  CCLanePostProcessorTest() {}
  ~CCLanePostProcessorTest() {}
  void SetUp() override {
    cc_lane_post_processor_.reset(new CCLanePostProcessor());
  }

 protected:
  shared_ptr<CCLanePostProcessor> cc_lane_post_processor_;
  lane_post_process_config::ModelConfigs config_;
};

TEST_F(CCLanePostProcessorTest, test_lane_post_process_vis) {
  FLAGS_work_root = "modules/perception";

  CHECK(GetProtoFromFile(FLAGS_cc_lane_post_processor_config_file, &config_));

  cc_lane_post_processor_->set_vis(true);
  EXPECT_TRUE(cc_lane_post_processor_->Init());

  const string input_test_image_file = FLAGS_test_dir + "test_image.jpg";
  AINFO << "test original image file: " << input_test_image_file;
  cv::Mat test_image_ori =
      cv::imread(input_test_image_file, CV_LOAD_IMAGE_COLOR);

  const string input_lane_map_file = FLAGS_test_dir + "lane_map.jpg";
  AINFO << "input lane map file: " << input_lane_map_file;
  cv::Mat lane_map_ori =
      cv::imread(input_lane_map_file, CV_LOAD_IMAGE_GRAYSCALE);

  string input_lane_map_file_test = FLAGS_test_dir + "test_lane_map.jpg";
  cv::imwrite(input_lane_map_file_test, lane_map_ori);

  cv::Mat lane_map;
  lane_map_ori.convertTo(lane_map, CV_32FC1, 1.0f / 255.0f, 0.0f);

  float lane_map_conf_thresh = 0.5f;
  CHECK(config_.has_lane_map_confidence_thresh());
  lane_map_conf_thresh = config_.lane_map_confidence_thresh();
  ADEBUG << "lane_map_conf_thresh = " << lane_map_conf_thresh;

  LaneObjectsPtr lane_objects = std::make_shared<LaneObjects>();
  CameraLanePostProcessOptions lane_post_process_options;
  cc_lane_post_processor_->Process(lane_map, lane_post_process_options,
                                   &lane_objects);

  AINFO << "#lane objects = " << lane_objects->size();

  cv::Mat vis_lane_objects_image;
  test_image_ori.copyTo(vis_lane_objects_image);
  for (int h = 0; h < lane_map.rows; ++h) {
    for (int w = 0; w < lane_map.cols; ++w) {
      if (lane_map.at<float>(h, w) >= lane_map_conf_thresh) {
        vis_lane_objects_image.at<cv::Vec3b>(h, w)[0] =
            static_cast<unsigned char>(lane_mask_color[0]);
        vis_lane_objects_image.at<cv::Vec3b>(h, w)[1] =
            static_cast<unsigned char>(lane_mask_color[1]);
        vis_lane_objects_image.at<cv::Vec3b>(h, w)[2] =
            static_cast<unsigned char>(lane_mask_color[2]);
      }
    }
  }

  // draw lane objects
  for (size_t k = 0; k < lane_objects->size(); ++k) {
    if (lane_objects->at(k).is_compensated) {
      continue;
    }

    cv::Scalar lane_object_color;
    switch (lane_objects->at(k).spatial) {
      case SpatialLabelType::L_0: {
        lane_object_color = cv::Scalar(0, 0, 255);  // red
        break;
      }
      case SpatialLabelType::L_1: {
        lane_object_color = cv::Scalar(255, 0, 255);  // magenta
        break;
      }
      case SpatialLabelType::L_2: {
        lane_object_color = cv::Scalar(255, 63, 153);  // purple
        break;
      }
      case SpatialLabelType::R_0: {
        lane_object_color = cv::Scalar(255, 0, 0);  // blue
        break;
      }
      case SpatialLabelType::R_1: {
        lane_object_color = cv::Scalar(255, 255, 0);  // cyan
        break;
      }
      case SpatialLabelType::R_2: {
        lane_object_color = cv::Scalar(63, 255, 192);  // greenyellow
        break;
      }
      default: {
        AERROR << "unknown lane spatial label: "
               << static_cast<int>(lane_objects->at(k).spatial);
      }
    }

    // Besides the fitted polynomial curves we draw lane markers as well
    for (auto p = lane_objects->at(k).image_pos.begin();
         p != lane_objects->at(k).image_pos.end(); ++p) {
      cv::circle(vis_lane_objects_image,
                 cv::Point(static_cast<int>(p->x()), static_cast<int>(p->y())),
                 4, lane_object_color, -1);
    }

    // draw polynomial curve
    float img_y_start =
        static_cast<float>(lane_objects->at(k).img_curve.x_start);
    float img_y_end = static_cast<float>(lane_objects->at(k).img_curve.x_end);
    float start = std::min(img_y_start, img_y_end);
    float end = std::max(img_y_start, img_y_end);
    float a = lane_objects->at(k).img_curve.a;
    float b = lane_objects->at(k).img_curve.b;
    float c = lane_objects->at(k).img_curve.c;
    float d = lane_objects->at(k).img_curve.d;

    for (float l = start; l <= end; l++) {
      cv::circle(vis_lane_objects_image,
                 cv::Point(static_cast<int>(GetPolyValue(a, b, c, d, l)),
                           static_cast<int>(l)),
                 2, lane_object_color, -1);
    }
  }

  // string lane_objects_image_file =
  //     FLAGS_test_dir + "test_lane_objects_image.jpg";
  // cv::imwrite(lane_objects_image_file, vis_lane_objects_image);
}

}  // namespace perception
}  // namespace apollo
