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
#include "modules/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/yolo.pb.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"
#include "modules/obstacle/camera/interface/base_camera_detector.h"

namespace apollo {
namespace perception {
namespace obstacle {
/*
TEST(YoloCameraAnakinDetectorTest, yolo_camera_detector_test) {
    std::string yolo_test_config =
"./data/models/yolo_camera_detector/config_test.pt";

    std::string yolo_config = "./data/models/yolo_camera_detector/config.pt";

    adu::perception::obstacle::yolo::YoloParam yolo_param;
    adu::perception::obstacle::yolo::YoloParam origin_yolo_param;

    load_text_proto_message_file(yolo_test_config, yolo_param);
    // roipooling feature extractor
    load_text_proto_message_file(yolo_config, origin_yolo_param);
    {
        yolo_param.mutable_model_param()->set_model_type(yolo::ModelType::Anakin);
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
        Timer timer;
        CHECK_EQ(camera_detector->detect(frame, options, NULL), false);
        std::vector<VisualObjectPtr> objects;
        CHECK(camera_detector->detect(frame, options, &objects));
        AINFO << "detection time: " << timer.toc() << "ms";

        // int obj_idx = 0;
        // for (const auto &obj : objects) {
        //     AINFO << "Obj-" << obj_idx++ << ": " <<
get_object_name(obj->type)
        //                << " (feat: " << obj->object_feature.size() << "-D)";
        //     if (obj->object_feature.size() > 0) {
        //         float sum_of_squares = 0.0;
        //         for (const auto &f : obj->object_feature) {
        //             sum_of_squares += f * f;
        //         }
        //         // EXPECT_LT(abs(sum_of_squares - 1.0), 1e-3);
        //     }
        // }
        // int obj_size = objects.size();
        // objects.clear();
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

        std::vector<VisualObjectPtr> mixed_objects;
        cv::Mat lane_map(frame.rows,frame.cols,CV_32FC1);

        CHECK(camera_detector->multitask(frame, options, &objects, &lane_map));
    }
    {
        std::string out_str;
        std::ofstream ofs(yolo_config, std::ofstream::out);
        google::protobuf::TextFormat::PrintToString(origin_yolo_param,
&out_str); ofs << out_str; ofs.close();
    }
}

TEST(YoloCameraAnakinDetectorTest, nms_test) {

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

}

TEST(YoloCameraAnakinDetectorTest, input_tensor_test) {
    std::string yolo_config = "./data/models/yolo_camera_detector/config.pt";

    adu::perception::obstacle::yolo::YoloParam yolo_param;
    adu::perception::obstacle::yolo::YoloParam origin_yolo_param;

    load_text_proto_message_file(yolo_config, yolo_param);
    origin_yolo_param.CopyFrom(yolo_param);
    yolo_param.mutable_model_param()->set_model_type(yolo::ModelType::Anakin);
    std::cout << yolo_param.model_param().weight_file() << std::endl;
    {
        std::string out_str;
        std::ofstream ofs(yolo_config, std::ofstream::out);
        google::protobuf::TextFormat::PrintToString(yolo_param, &out_str);
        ofs << out_str;
        ofs.close();

        YoloCameraDetector camera_detector;
        CHECK(camera_detector.init());
        cv::Mat frame = cv::imread("test.jpg", CV_LOAD_IMAGE_COLOR);
        CHECK((frame.data != nullptr));
        CameraDetectorOptions options;
        std::vector<VisualObjectPtr> objects;
        // CHECK(camera_detector.detect(frame, options, &objects));
    }
    std::string out_str;
    std::ofstream ofs(yolo_config, std::ofstream::out);
    google::protobuf::TextFormat::PrintToString(origin_yolo_param, &out_str);
    ofs << out_str;
    ofs.close();
}

TEST(YoloCameraAnakinDetectorTest, recover_bbox_test) {
    std::vector<VisualObjectPtr> visual_objects;
    for (int i = 0; i < 1; i++) {
        VisualObjectPtr obj;

        obj.reset(new VisualObject);
        obj->alpha = 0;
        obj->upper_left[0] = -1;
        obj->upper_left[1] = -1;
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
    recover_bbox(100, 100, 0, &visual_objects);
    visual_objects.clear();
    recover_bbox(100, 100, 0, &visual_objects);
}

TEST(YoloCameraAnakinDetectorTest, bbox_op_test) {
    {
        NormalizedBBox bbox1;
        NormalizedBBox bbox2;
        bbox1.xmin = 20;
        bbox1.ymin = 20;
        bbox1.xmax = 10;
        bbox1.ymax = 10;
        bbox2.xmin = 20;
        bbox2.ymin = 20;
        bbox2.xmax = 10;
        bbox2.ymax = 10;
        NormalizedBBox res_bbox;
        get_intersect_bbox(bbox1, bbox2, &res_bbox);
                CHECK_EQ(res_bbox.xmin, 0);
                CHECK_EQ(res_bbox.ymin, 0);
                CHECK_EQ(res_bbox.xmax, 0);
                CHECK_EQ(res_bbox.ymax, 0);
    }
    {
        NormalizedBBox bbox;
        bbox.xmin = 0.2;
        bbox.ymin = 0.2;
        bbox.xmax = 0.1;
        bbox.ymax = 0.1;
        float size = -1;
        size = get_bbox_size(bbox);
        EXPECT_FLOAT_EQ(0, size);

        bbox.xmax = 0.3;
        bbox.ymax = 0.3;
        bbox.size = 0.01;
        size = get_bbox_size(bbox);
        EXPECT_FLOAT_EQ(0.01, size);
        bbox.size = -1;
        size = get_bbox_size(bbox);
        EXPECT_FLOAT_EQ(0.01, size);
    }
    {
        NormalizedBBox bbox1;
        NormalizedBBox bbox2;
        bbox1.xmin = 0;
        bbox1.ymin = 0;
        bbox1.xmax = 0.1;
        bbox1.ymax = 0.1;
        bbox2.xmin = 0;
        bbox2.ymin = 0;
        bbox2.xmax = 0.1;
        bbox2.ymax = 0.1;
        float overlap = 0;
        overlap = get_jaccard_overlap(bbox1, bbox2);
        EXPECT_FLOAT_EQ(1, overlap);
        bbox2.xmin = 0.2;
        bbox2.ymin = 0.2;
        bbox2.xmax = 0.3;
        bbox2.ymax = 0.3;
        overlap = get_jaccard_overlap(bbox1, bbox2);
        EXPECT_FLOAT_EQ(0, overlap);
    }

}*/
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
