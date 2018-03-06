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

#include "modules/common/log.h
#include <gtest/gtest.h>
#include <caffe/caffe.hpp>
#include <opencv2/opencv.hpp>

#include "modules/obstacle/camera/common/flags.h"
#include "modules/obstacle/camera/common/util.h"
#include "modules/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/region_output.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/yolo.pb.h"
#include "modules/obstacle/camera/interface/base_camera_detector.h"

namespace apollo {
namespace perception {

TEST(YoloCameraDetectorTest, yolo_camera_detector_roipooling_test) {
  std::string yolo_config = "./data/models/yolo_camera_detector/config.pt";

  adu::perception::yolo::YoloParam yolo_param;
  adu::perception::yolo::YoloParam origin_yolo_param;

  {
    BaseCameraDetector *camera_detector =
        BaseCameraDetectorRegisterer::get_instance_by_name(
            "YoloCameraDetector");
    CHECK(camera_detector->init());
    CHECK_EQ(camera_detector->name(), "YoloCameraDetector");
    cv::Mat frame = cv::imread("test.jpg", CV_LOAD_IMAGE_COLOR);
    CHECK((frame.data != nullptr));

    CameraDetectorOptions options;
    std::vector<VisualObjectPtr> objects;
    Timer timer;
    /*CHECK_EQ(camera_detector->detect(frame, options, NULL), false);
    CHECK(camera_detector->detect(frame, options, &objects));
    AINFO << "detection time: " << timer.toc() << "ms";
    int obj_idx = 0;
    for (const auto &obj : objects) {

        AINFO << "Obj-" << obj_idx++ << ": " << get_object_name(obj->type)
                   << " (feat: " << obj->object_feature.size() << "-D)";
        if (obj->object_feature.size() > 0) {
            float sum_of_squares = 0.0;
            for (const auto &f : obj->object_feature) {
                sum_of_squares += f * f;
            }
            // EXPECT_LT(abs(sum_of_squares - 1.0), 1e-3);
        }
    }*/
    std::vector<VisualObjectPtr> mixed_objects;
    cv::Mat lane_map(frame.rows, frame.cols, CV_32FC1);

    CHECK(camera_detector->multitask(frame, options, &objects, &lane_map));
    cv::imwrite("result.jpg", lane_map);
  }
}
/*
TEST(YoloCameraDetectorTest, nms_test) {
    {
        std::vector<NormalizedBBox> test_objects;
        NormalizedBBox obj_ped1;
        obj_ped1.xmin = .1;
        obj_ped1.xmax = .3;
        obj_ped1.ymin = .20;
        obj_ped1.ymax = .60;
        obj_ped1.score = 0.9;
        obj_ped1.label = PEDESTRIAN;

        NormalizedBBox obj_ped2;
        obj_ped2.xmin = .10;
        obj_ped2.xmax = .25;
        obj_ped2.ymin = .30;
        obj_ped2.ymax = .60;
        obj_ped2.score = 0.8;
        obj_ped2.label = PEDESTRIAN;
        NormalizedBBox obj_ped3;
        obj_ped3.xmin = .7;
        obj_ped3.xmax = .8;
        obj_ped3.ymin = .7;
        obj_ped3.ymax = .8;
        obj_ped3.score = 0.01;
        obj_ped3.label = PEDESTRIAN;

        test_objects.push_back(obj_ped1);
        test_objects.push_back(obj_ped2);
        test_objects.push_back(obj_ped3);

        std::vector<float> scores;
        scores.push_back(obj_ped1.score);
        scores.push_back(obj_ped2.score);
        scores.push_back(obj_ped3.score);

        std::vector<int> indices;
        apply_softnms_fast(test_objects, scores, 0.1, 0.5, 20, &indices, true,
0.4); CHECK_LT(scores[1], 0.8); scores[1] = 0.8; scores[2] = 0.01;
        apply_softnms_fast(test_objects, scores, 0.1, 0.5, 20, &indices, false,
0.4); CHECK_LT(scores[1], 0.8);

        scores[1] = 0.8;
        scores[2] = 0.01;
        apply_boxvoting_fast(test_objects, scores, 0.1, 0.5, 0.4, &indices);
                CHECK_LT(test_objects[0].ymin, .30);
                CHECK_GT(test_objects[0].ymin, .20);
                CHECK_LT(test_objects[0].xmax, .30);
                CHECK_GT(test_objects[0].xmax, .20);
                CHECK_LT(scores[1], 0.8);
        std::vector<NormalizedBBox> test_empty_objects;
        std::vector<float> test_empty_scores;

        apply_boxvoting_fast(test_empty_objects, test_empty_scores, 0.1, 0.5,
0.4, &indices);

    }
    {
        std::vector<NormalizedBBox> test_objects;
        NormalizedBBox obj_ped1;
        obj_ped1.xmin = .1;
        obj_ped1.xmax = .3;
        obj_ped1.ymin = .20;
        obj_ped1.ymax = .60;
        obj_ped1.score = 0.9;
        obj_ped1.label = PEDESTRIAN;

        NormalizedBBox obj_ped2;
        obj_ped2.xmin = .10;
        obj_ped2.xmax = .25;
        obj_ped2.ymin = .30;
        obj_ped2.ymax = .60;
        obj_ped2.score = 0.8;
        obj_ped2.label = PEDESTRIAN;
        NormalizedBBox obj_ped3;
        obj_ped3.xmin = .7;
        obj_ped3.xmax = .8;
        obj_ped3.ymin = .7;
        obj_ped3.ymax = .8;
        obj_ped3.score = 0.01;
        obj_ped3.label = PEDESTRIAN;

        test_objects.push_back(obj_ped1);
        test_objects.push_back(obj_ped2);
        test_objects.push_back(obj_ped3);

        std::vector<float> scores;
        scores.push_back(obj_ped1.score);
        scores.push_back(obj_ped2.score);
        scores.push_back(obj_ped3.score);

        std::vector<int> indices;
        apply_nms_fast(test_objects, scores, 0.1, 0.5, 1, 20, &indices);
                CHECK_EQ(indices.size(), 1);
        apply_nms_fast(test_objects, scores, 0.1, 0.5, 0.7, 20, &indices);
                CHECK_EQ(indices.size(), 1);

    }
    {
        std::vector<NormalizedBBox> test_objects;
        NormalizedBBox obj_cyc;
        obj_cyc.xmin = .1;
        obj_cyc.xmax = .3;
        obj_cyc.ymin = .20;
        obj_cyc.ymax = .60;
        obj_cyc.score = 0.9;
        obj_cyc.label = BICYCLE;

        NormalizedBBox obj_ped;
        obj_ped.xmin = .10;
        obj_ped.xmax = .25;
        obj_ped.ymin = .30;
        obj_ped.ymax = .60;
        obj_ped.score = 0.95;
        obj_ped.label = PEDESTRIAN;

        test_objects.push_back(obj_cyc);
        test_objects.push_back(obj_ped);

        std::vector<int> cyc_indices;
        std::vector<int> ped_indices;
        cyc_indices.push_back(0);
        ped_indices.push_back(1);
        cross_class_merge(&cyc_indices, &ped_indices, test_objects, 0.8);
                CHECK_EQ(cyc_indices.size(), 1);
                CHECK_EQ(ped_indices.size(), 0);
    }


    std::vector<VisualObjectPtr> visual_objects;
    for (int i = 0; i < 1; i++) {
        VisualObjectPtr obj;

        obj.reset(new VisualObject);
        obj->alpha = 0;
        obj->upper_left[0] = 0.1;
        obj->upper_left[1] = 0.2;
        obj->lower_right[0] = 0.3;
        obj->lower_right[1] = 0.4;
        obj->height = 1.6;
        obj->width = 1.4;
        obj->length = 4;
        obj->center.x() = 0;
        obj->center.y() = 0;
        obj->center.z() = 0;
        obj->theta = 1.1;
        obj->score = 0.9;
        obj->type = PEDESTRIAN;
        visual_objects.push_back(obj);
    }
    recover_bbox(10, 10, 5, &visual_objects);
            CHECK_EQ(visual_objects[0]->upper_left[0], 1);
            CHECK_EQ(visual_objects[0]->upper_left[1], 7);
            CHECK_EQ(visual_objects[0]->lower_right[0], 3);
            CHECK_EQ(visual_objects[0]->lower_right[1], 9);

    {
        std::vector<NormalizedBBox> test_empty_objects;
        std::vector<float> empty_scores;
        std::vector<int> empty_indices;

        apply_softnms_fast(test_empty_objects, empty_scores, 0.1, 0.5, 20,
&empty_indices, true, 0.8); CHECK_EQ(empty_indices.size(), 0);
        apply_boxvoting_fast(test_empty_objects, empty_scores, 0.1, 0.5, 20,
&empty_indices); CHECK_EQ(empty_indices.size(), 0);

    }
}
*/
TEST(YoloCameraDetectorTest, input_tensor_test) {
  std::string yolo_config = "./data/models/yolo_camera_detector/config.pt";

  adu::perception::yolo::YoloParam yolo_param;
  adu::perception::yolo::YoloParam origin_yolo_param;

  load_text_proto_message_file(yolo_config, yolo_param);
  origin_yolo_param.CopyFrom(yolo_param);

  {
    std::string out_str;
    std::ofstream ofs(yolo_config, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
    BaseCameraDetector *camera_detector =
        BaseCameraDetectorRegisterer::get_instance_by_name(
            "YoloCameraDetector");
    CHECK(camera_detector->init());
  }
  std::string out_str;
  std::ofstream ofs(yolo_config, std::ofstream::out);
  google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
  ofs << out_str;
  ofs.close();
}
}  // namespace perception
}  // namespace apollo
