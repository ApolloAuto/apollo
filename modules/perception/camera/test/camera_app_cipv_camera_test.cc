/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "cyber/common/log.h"
#include "modules/perception/base/distortion_model.h"
#include "modules/perception/base/point.h"
#include "modules/perception/camera/app/cipv_camera.h"
// #include "modules/perception/camera/lib/lane/common/common_functions.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(CIPV, cipv_ground_test) {
  {
    // Two lane test
    Eigen::Matrix3d homography_im2car_;
    homography_im2car_ << 1.79535e-06, 9.37775e-05, 0.942375, -0.00050384,
        2.48862e-05, 0.48999, 2.7712e-06, 0.000317091, -0.191827;

    float min_laneline_length_for_cipv_ = kMinLaneLineLengthForCIPV;
    float average_lane_width_in_meter_ = kAverageLaneWidthInMeter;
    float max_vehicle_width_in_meter_ = kMaxVehicleWidthInMeter;
    float average_frame_rate_ = kAverageFrameRate;
    bool image_based_cipv_ = false;
    int debug_level = 3;

    Cipv cipv_;
    cipv_.Init(homography_im2car_, min_laneline_length_for_cipv_,
               average_lane_width_in_meter_, max_vehicle_width_in_meter_,
               average_frame_rate_, image_based_cipv_, debug_level);

    CipvOptions cipv_options;
    cipv_options.velocity = 5.0f;
    cipv_options.yaw_rate = 0.0f;
    std::vector<base::LaneLine> lane_objects;
    base::LaneLine left_lane_instance;
    base::LaneLine right_lane_instance;

    left_lane_instance.track_id = 0;

    std::vector<base::Point2DF> image_points;
    base::Point2DF ground_point;
    base::Point2DF image_point;

    image_point.x = 696.0f;
    image_point.y = 963.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 7.24724f;
    ground_point.y = 1.12886f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 828.0f;
    image_point.y = 825.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 10.3069f;
    ground_point.y = 0.917161f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 843.0f;
    image_point.y = 806.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 10.9515f;
    ground_point.y = 0.889836f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 849.0f;
    image_point.y = 788.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 11.6462f;
    ground_point.y = 0.9079f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 855.0f;
    image_point.y = 771.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 12.3902f;
    ground_point.y = 0.925199f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 873.0f;
    image_point.y = 755.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 13.1789f;
    ground_point.y = 0.862779f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 885.0f;
    image_point.y = 740.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 14.0199f;
    ground_point.y = 0.830402f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 888.0f;
    image_point.y = 728.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 14.7796f;
    ground_point.y = 0.849831f;
    left_lane_instance.curve_car_coord_point_set.push_back(ground_point);

    left_lane_instance.pos_type = base::LaneLinePositionType::EGO_LEFT;
    lane_objects.push_back(left_lane_instance);

    image_point.x = 1578.0f;
    image_point.y = 1001.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 6.59481f;
    ground_point.y = -1.79636f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1539.0f;
    image_point.y = 963.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 7.12811f;
    ground_point.y = -1.81776f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1500.0f;
    image_point.y = 929.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 7.68774f;
    ground_point.y = -1.82493f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1464.0f;
    image_point.y = 899.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 8.26296f;
    ground_point.y = -1.82594f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1434.0f;
    image_point.y = 872.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 8.86158f;
    ground_point.y = -1.83732f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1407.0f;
    image_point.y = 848.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 9.4739f;
    ground_point.y = -1.84742f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1380.0f;
    image_point.y = 825.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 10.1484f;
    ground_point.y = -1.85298f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1359.0f;
    image_point.y = 806.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 10.7841f;
    ground_point.y = -1.86509f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1344.0f;
    image_point.y = 788.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 11.4646f;
    ground_point.y = -1.90547f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1320.0f;
    image_point.y = 771.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 12.197f;
    ground_point.y = -1.8901f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1302.0f;
    image_point.y = 755.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 12.9771f;
    ground_point.y = -1.90283f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1290.0f;
    image_point.y = 740.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 13.8042f;
    ground_point.y = -1.94931f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1275.0f;
    image_point.y = 728.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 14.5505f;
    ground_point.y = -1.95259f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1266.0f;
    image_point.y = 715.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 15.4526f;
    ground_point.y = -2.01151f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1257.0f;
    image_point.y = 704.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 16.3107f;
    ground_point.y = -2.05642f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1245.0f;
    image_point.y = 692.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 17.3661f;
    ground_point.y = -2.09295f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1236.0f;
    image_point.y = 683.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 18.2532f;
    ground_point.y = -2.12367f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1224.0f;
    image_point.y = 673.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 19.3552f;
    ground_point.y = -2.14272f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1212.0f;
    image_point.y = 664.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 20.4702f;
    ground_point.y = -2.14991f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1206.0f;
    image_point.y = 656.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 21.5705f;
    ground_point.y = -2.20656f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1200.0f;
    image_point.y = 648.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 22.7972f;
    ground_point.y = -2.26971f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1188.0f;
    image_point.y = 640.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 24.1831f;
    ground_point.y = -2.26918f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);
    image_point.x = 1179.0f;
    image_point.y = 632.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    ground_point.x = 25.7449f;
    ground_point.y = -2.30662f;
    right_lane_instance.curve_car_coord_point_set.push_back(ground_point);

    right_lane_instance.pos_type = base::LaneLinePositionType::EGO_RIGHT;
    lane_objects.push_back(right_lane_instance);

    Eigen::Affine3d world2camera = Eigen::Affine3d::Identity();

    std::vector<std::shared_ptr<base::Object>> objects;
    base::ObjectPtr object1(new base::Object);
    object1->track_id = 0;
    object1->camera_supplement.box = base::RectF(83.0f, 486.73f, 15.0f, 39.6f);
    object1->center(0) = 12.0;
    object1->center(1) = 0.0;
    object1->center(2) = 6.0;
    object1->anchor_point(0) = 559664;
    object1->anchor_point(1) = 4.15741e+06;
    object1->anchor_point(2) = -14.9444;
    object1->direction(0) = 0.744489f;
    object1->direction(1) = 0.667634f;
    object1->direction(2) = 0.0f;
    object1->size(0) = 1.4886f;
    object1->size(1) = 0.4825f;
    object1->size(2) = 1.6182f;
    object1->camera_supplement.alpha = 1.48424;

    objects.push_back(object1);

    base::ObjectPtr object2(new base::Object);
    object2->track_id = 2;
    object2->camera_supplement.box =
        base::RectF(852.067f, 537.943f, 345.337f, 267.294f);
    object2->center(0) = -0.9;
    object2->center(1) = 0.9;
    object2->center(2) = 13.0;
    object2->anchor_point(0) = 559712;
    object2->anchor_point(1) = 4.15745e+06;
    object2->anchor_point(2) = -19.9879;
    object2->direction(0) = 0.783255f;
    object2->direction(1) = 0.621701f;
    object2->direction(2) = 0.0f;
    object2->size(0) = 4.12469f;
    object2->size(1) = 1.49074f;
    object2->size(2) = 1.38566f;
    object2->camera_supplement.alpha = 1.39487;

    objects.push_back(object2);

    base::ObjectPtr object3(new base::Object);
    object3->track_id = 3;
    object3->camera_supplement.box =
        base::RectF(1343.66f, 563.465f, 341.659f, 272.38f);
    object3->center(0) = -4.5;
    object3->center(1) = 0.0;
    object3->center(2) = 5.0;
    object3->anchor_point(0) = 559711;
    object3->anchor_point(1) = 4.15745e+06;
    object3->anchor_point(2) = -19.7986;
    object3->direction(0) = 0.74477f;
    object3->direction(1) = 0.667321f;
    object3->direction(2) = 0.0f;
    object3->size(0) = 4.12469f;
    object3->size(1) = 1.49074f;
    object3->size(2) = 1.38566f;
    object3->camera_supplement.alpha = 1.42243;

    objects.push_back(object3);

    cipv_.DetermineCipv(lane_objects, cipv_options, world2camera, &objects);

    EXPECT_FALSE(objects[0]->b_cipv);
    EXPECT_TRUE(objects[1]->b_cipv);
    EXPECT_FALSE(objects[2]->b_cipv);
  }
}

TEST(CIPV, cipv_image_test) {
  {
    // Two lane test
    Eigen::Matrix3d homography_im2car_;
    homography_im2car_ << 1.79535e-06, 9.37775e-05, 0.942375, -0.00050384,
        2.48862e-05, 0.48999, 2.7712e-06, 0.000317091, -0.191827;

    float min_laneline_length_for_cipv_ = kMinLaneLineLengthForCIPV;
    float average_lane_width_in_meter_ = kAverageLaneWidthInMeter;
    float max_vehicle_width_in_meter_ = kMaxVehicleWidthInMeter;
    float average_frame_rate_ = kAverageFrameRate;
    bool image_based_cipv_ = true;
    int debug_level = 3;

    Cipv cipv_;
    cipv_.Init(homography_im2car_, min_laneline_length_for_cipv_,
               average_lane_width_in_meter_, max_vehicle_width_in_meter_,
               average_frame_rate_, image_based_cipv_, debug_level);

    CipvOptions cipv_options;
    cipv_options.velocity = 5.0f;
    cipv_options.yaw_rate = 0.0f;

    std::vector<base::LaneLine> lane_objects;
    base::LaneLine left_lane_instance;
    base::LaneLine right_lane_instance;

    left_lane_instance.track_id = 0;

    std::vector<base::Point2DF> image_points;
    base::Point2DF ground_point;
    base::Point2DF image_point;

    image_point.x = 0.0f;
    image_point.y = 1000.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 100.0f;
    image_point.y = 900.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 200.0f;
    image_point.y = 800.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 300.0f;
    image_point.y = 700.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 400.0f;
    image_point.y = 600.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 500.0f;
    image_point.y = 500.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 600.0f;
    image_point.y = 400.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 700.0f;
    image_point.y = 300.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 800.0f;
    image_point.y = 200.0f;
    left_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 900.0f;
    image_point.y = 100.0f;

    left_lane_instance.pos_type = base::LaneLinePositionType::EGO_LEFT;
    lane_objects.push_back(left_lane_instance);

    image_point.x = 1900.0f;
    image_point.y = 1000.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1800.0f;
    image_point.y = 900.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1700.0f;
    image_point.y = 800.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1600.0f;
    image_point.y = 700.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1500.0f;
    image_point.y = 600.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1400.0f;
    image_point.y = 500.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1300.0f;
    image_point.y = 400.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1200.0f;
    image_point.y = 300.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1100.0f;
    image_point.y = 200.0f;
    right_lane_instance.curve_image_point_set.push_back(image_point);
    image_point.x = 1000.0f;
    image_point.y = 100.0f;

    right_lane_instance.pos_type = base::LaneLinePositionType::EGO_RIGHT;
    lane_objects.push_back(right_lane_instance);

    Eigen::Affine3d world2camera = Eigen::Affine3d::Identity();

    std::vector<std::shared_ptr<base::Object>> objects;
    base::ObjectPtr object1(new base::Object);
    object1->track_id = 0;
    object1->camera_supplement.box = base::RectF(50.0f, 200.0f, 15.0f, 39.6f);
    object1->center(0) = 6.0;
    object1->center(1) = 12.0;
    object1->center(2) = 0.0;
    object1->anchor_point(0) = 559664;
    object1->anchor_point(1) = 4.15741e+06;
    object1->anchor_point(2) = -14.9444;
    object1->direction(0) = 0.744489f;
    object1->direction(1) = 0.667634f;
    object1->direction(2) = 0.0f;
    object1->size(0) = 1.4886f;
    object1->size(1) = 0.4825f;
    object1->size(2) = 1.6182f;
    object1->camera_supplement.alpha = 1.48424;
    object1->camera_supplement.local_center(0) = 1170.0f;
    object1->camera_supplement.local_center(1) = 526.33f;
    objects.push_back(object1);

    base::ObjectPtr object2(new base::Object);
    object2->track_id = 2;
    object2->camera_supplement.box =
        base::RectF(1000.0f, 700.0f, 400.0f, 200.0f);
    object2->center(0) = 10.0;
    object2->center(1) = 0.1;
    object2->center(2) = 0.0;
    object2->anchor_point(0) = 559712;
    object2->anchor_point(1) = 4.15745e+06;
    object2->anchor_point(2) = -19.9879;
    object2->direction(0) = 0.783255f;
    object2->direction(1) = 0.621701f;
    object2->direction(2) = 0.0f;
    object2->size(0) = 4.12469f;
    object2->size(1) = 1.49074f;
    object2->size(2) = 1.38566f;
    object2->camera_supplement.alpha = 1.39487;
    object2->camera_supplement.local_center(0) = 1024.73550f;
    object2->camera_supplement.local_center(1) = 805.237f;
    objects.push_back(object2);

    base::ObjectPtr object3(new base::Object);
    object3->track_id = 3;
    object3->camera_supplement.box =
        base::RectF(1300.0f, 500.0f, 300.0f, 150.00f);
    object3->center(0) = 5.0;
    object3->center(1) = -4.5;
    object3->center(2) = 0.0;
    object3->anchor_point(0) = 559711;
    object3->anchor_point(1) = 4.15745e+06;
    object3->anchor_point(2) = -19.7986;
    object3->direction(0) = 0.74477f;
    object3->direction(1) = 0.667321f;
    object3->direction(2) = 0.0f;
    object3->size(0) = 4.12469f;
    object3->size(1) = 1.49074f;
    object3->size(2) = 1.38566f;
    object3->camera_supplement.alpha = 1.42243;
    object3->camera_supplement.local_center(0) = 1514.4895f;
    object3->camera_supplement.local_center(1) = 835.845f;
    objects.push_back(object3);

    cipv_.DetermineCipv(lane_objects, cipv_options, world2camera, &objects);

    EXPECT_FALSE(objects[0]->b_cipv);
    EXPECT_TRUE(objects[1]->b_cipv);
    EXPECT_FALSE(objects[2]->b_cipv);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
