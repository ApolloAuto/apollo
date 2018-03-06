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
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include <caffe/caffe.hpp>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "modules/common/log.h
#include "modules/perception/obstacle/camera/common/flags.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo.pb.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"

namespace apollo {
namespace perception {

TEST(YoloCameraTensorRTDetectorTest, yolo_camera_detector_test) {
  /*   std::string yolo_test_config =
     "./data/models/yolo_camera_detector/config_test.pt";

     std::string yolo_config = "./data/models/yolo_camera_detector/config.pt";

     adu::perception::yolo::YoloParam yolo_param;
     adu::perception::yolo::YoloParam origin_yolo_param;
     int origin_gpu_flag = FLAGS_camera_detector_gpu;

     load_text_proto_message_file(yolo_test_config, yolo_param);
     // roipooling feature extractor
     load_text_proto_message_file(yolo_config, origin_yolo_param);
     {
         yolo_param.mutable_model_param()->set_model_type(yolo::ModelType::TensorRT);
         yolo_param.mutable_model_param()->set_proto_file("tensorrt.pt");
         std::string out_str;
         std::ofstream ofs(yolo_config, std::ofstream::out);
         google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
         ofs << out_str;
         ofs.close();

         BaseCameraDetector *camera_detector =
                 BaseCameraDetectorRegisterer::get_instance_by_name("YoloCameraDetector");
         CHECK(camera_detector->init());
                 CHECK_EQ(camera_detector->name(), "YoloCameraDetector");
         cv::Mat frame = cv::imread("test.jpg", CV_LOAD_IMAGE_COLOR);
         CHECK((frame.data != nullptr));

         CameraDetectorOptions options;
         std::vector<VisualObjectPtr> objects;
         Timer timer;
                 CHECK_EQ(camera_detector->detect(frame, options, NULL), false);
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
         }
         int obj_size = objects.size();
         objects.clear();
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
         objects.push_back(obj);
                 CHECK_EQ(camera_detector->detect(frame, options, nullptr),
     false); FLAGS_camera_detector_gpu = -1; YoloCameraDetector
     camera_detector_tmp;

                 CHECK_EQ(camera_detector_tmp.init(), false);
         FLAGS_camera_detector_gpu = origin_gpu_flag;
     }
     {
         std::string out_str;
         std::ofstream ofs(yolo_config, std::ofstream::out);
         google::protobuf::TextFormat::PrintToString(origin_yolo_param,
     &out_str); ofs << out_str; ofs.close();
     }*/
}

}  // namespace perception
}  // namespace apollo
