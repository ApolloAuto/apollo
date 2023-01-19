/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "gtest/gtest.h"

#include "modules/perception/base/distortion_model.h"
#include "modules/perception/camera/lib/obstacle/detector/yolo/yolo_obstacle_detector.h"
#include "modules/perception/common/io/io_util.h"

namespace apollo {
namespace perception {
namespace camera {
void init_box(NormalizedBBox *box) {
  box->xmin = 1;
  box->xmax = 10;
  box->ymin = 1;
  box->ymax = 10;
  box->size = -1;
}

TEST(YoloCameraDetectorTest, bbox_size_gpu_test) {
  // empty bbox
  {
    float bbox[] = {1.0f, 1.0f, 1.0f, 1.0f};
    EXPECT_NEAR(bbox_size_gpu(bbox, false), 0, 1e-3);
  }
  {
    float bbox[] = {1.0f, 1.0f, 1.0f, 2.0f};
    EXPECT_NEAR(bbox_size_gpu(bbox, false), 0, 1e-3);
  }
  {
    float bbox[] = {1.0f, 1.0f, 2.0f, 1.0f};
    EXPECT_NEAR(bbox_size_gpu(bbox, false), 0, 1e-3);
  }

  // unnormalized bbox
  {
    float bbox[] = {1.0f, 1.0f, 2.0f, 2.0f};
    EXPECT_NEAR(bbox_size_gpu(bbox, false), 4, 1e-3);
  }

  // normalized bbox
  {
    float bbox[] = {.1f, .1f, .2f, .2f};
    EXPECT_NEAR(bbox_size_gpu(bbox, true), .01f, 1e-3);
  }
}

TEST(YoloCameraDetectorTest, jaccard_overlap_gpu_test) {
  // no overlap
  {
    float bbox1[] = {.1f, .1f, .2f, .2f};
    float bbox2[] = {.3f, .3f, .4f, .4f};
    EXPECT_NEAR(jaccard_overlap_gpu(bbox1, bbox2), 0, 1e-3);
  }

  // overlapped
  {
    float bbox1[] = {.1f, .1f, .2f, .2f};
    float bbox2[] = {.1f, .1f, .4f, .4f};
    EXPECT_NEAR(jaccard_overlap_gpu(bbox1, bbox2), .111111f, 1e-3);
  }
}

TEST(YoloCameraDetectorTest, apply_nms_test) {
  bool overlapped[] = {true, true, false, true, true, true, false, true, true};

  {
    std::vector<int> indices;
    apply_nms(overlapped, 0, &indices);
    EXPECT_EQ(indices.size(), 0);
  }

  {
    std::vector<int> indices;
    apply_nms(overlapped, 3, &indices);
    EXPECT_EQ(indices.size(), 2);
  }
}

TEST(YoloCameraDetectorTest, get_gpu_data_test) {
  base::Blob<float> blob;
  blob.Reshape({1, 2});
  EXPECT_NE(get_gpu_data(true, blob), nullptr);
  EXPECT_EQ(get_gpu_data(false, blob), nullptr);
}

TEST(YoloCameraDetectorTest, box_test) {
  NormalizedBBox box1;
  NormalizedBBox box2;
  NormalizedBBox box3;

  init_box(&box1);
  init_box(&box2);
  box2.xmin = 20;
  get_intersect_bbox(box1, box2, &box3);
  ASSERT_EQ(box3.xmin, 0);

  init_box(&box1);
  init_box(&box2);
  box2.xmax = 0;
  get_intersect_bbox(box1, box2, &box3);
  ASSERT_EQ(box3.xmin, 0);
  init_box(&box1);
  init_box(&box2);
  box2.ymin = 20;
  get_intersect_bbox(box1, box2, &box3);
  ASSERT_EQ(box3.xmin, 0);
  init_box(&box1);
  init_box(&box2);
  box2.ymax = 0;
  get_intersect_bbox(box1, box2, &box3);
  ASSERT_EQ(box3.xmin, 0);
  init_box(&box1);
  init_box(&box2);
  get_intersect_bbox(box1, box2, &box3);
  ASSERT_EQ(box3.xmin, 1);
  ASSERT_EQ(box3.xmax, 10);

  float size = 0.0f;
  init_box(&box1);
  box1.xmax = 0;
  size = get_bbox_size(box1);
  ASSERT_TRUE(fabs(size) < 1e-3);
  init_box(&box1);
  box1.ymax = 0;
  size = get_bbox_size(box1);
  ASSERT_TRUE(fabs(size) < 1e-3);

  init_box(&box1);
  size = get_bbox_size(box1);
  ASSERT_TRUE(fabs(size - 81) < 1e-3);

  box1.size = 10;
  size = get_bbox_size(box1);
  ASSERT_TRUE(fabs(size - 10) < 1e-3);

  init_box(&box1);
  init_box(&box2);
  size = get_jaccard_overlap(box1, box2);
  AINFO << size;
  ASSERT_TRUE(fabs(size - 1) < 1e-3);
  box2.ymax = 0;
  size = get_jaccard_overlap(box1, box2);
  ASSERT_TRUE(fabs(size) < 1e-3);
  box2.xmax = 0;
  size = get_jaccard_overlap(box1, box2);
  ASSERT_TRUE(fabs(size) < 1e-3);
}
TEST(YoloCameraDetectorTest, filter_box) {
  base::ObjectPtr obj;
  obj.reset(new base::Object);
  obj->camera_supplement.box.ymax = 50;
  obj->camera_supplement.box.ymin = 10;
  obj->size[0] = 10;  // length
  obj->size[1] = 20;  // width
  obj->size[2] = 30;  // height

  std::vector<base::ObjectPtr> v_objs;
  MinDims min_dims;

  v_objs.push_back(obj);
  filter_bbox(min_dims, &v_objs);
  ASSERT_EQ(v_objs.size(), 1);

  v_objs.clear();
  v_objs.push_back(obj);
  min_dims.min_2d_height = 20;
  min_dims.min_3d_height = -1;
  min_dims.min_3d_length = -1;
  min_dims.min_3d_width = -1;
  filter_bbox(min_dims, &v_objs);
  ASSERT_EQ(v_objs.size(), 1);

  v_objs.clear();
  v_objs.push_back(obj);
  min_dims.min_2d_height = 20;
  min_dims.min_3d_height = 15;
  min_dims.min_3d_length = 25;
  min_dims.min_3d_width = 35;
  filter_bbox(min_dims, &v_objs);
  ASSERT_EQ(v_objs.size(), 0);

  v_objs.clear();
  v_objs.push_back(obj);
  min_dims.min_2d_height = 50;
  min_dims.min_3d_height = -1;
  min_dims.min_3d_length = -1;
  min_dims.min_3d_width = -1;
  filter_bbox(min_dims, &v_objs);
  ASSERT_EQ(v_objs.size(), 0);

  v_objs.clear();
  v_objs.push_back(obj);
  min_dims.min_2d_height = 50;
  min_dims.min_3d_height = 15;
  min_dims.min_3d_length = 25;
  min_dims.min_3d_width = 35;
  filter_bbox(min_dims, &v_objs);
  ASSERT_EQ(v_objs.size(), 0);
}
TEST(YoloCameraDetectorTest, fill_results) {
  base::ObjectPtr obj;
  obj.reset(new base::Object);
  float bbox[kBoxBlockSize];
  for (int i = 0; i < kBoxBlockSize; i++) {
    bbox[i] = static_cast<float>(i);
  }

  // fill_bbox3d
  fill_bbox3d(false, obj, bbox + 4);
  ASSERT_NEAR(obj->camera_supplement.alpha, 0, 1e-5);
  ASSERT_NEAR(obj->size[2], 0, 1e-5);
  ASSERT_NEAR(obj->size[1], 0, 1e-5);
  ASSERT_NEAR(obj->size[0], 0, 1e-5);
  fill_bbox3d(true, obj, bbox + 4);
  ASSERT_NEAR(obj->camera_supplement.alpha, bbox[4], 1e-5);
  ASSERT_NEAR(obj->size[2], bbox[5], 1e-5);
  ASSERT_NEAR(obj->size[1], bbox[6], 1e-5);
  ASSERT_NEAR(obj->size[0], bbox[7], 1e-5);

  // fill_frbox
  fill_frbox(false, obj, bbox + 8);
  fill_frbox(true, obj, bbox + 8);

  ASSERT_NEAR(obj->camera_supplement.front_box.xmin, bbox[8], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.front_box.ymin, bbox[9], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.front_box.xmax, bbox[10], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.front_box.ymax, bbox[11], 1e-5);

  ASSERT_NEAR(obj->camera_supplement.back_box.xmin, bbox[12], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.back_box.ymin, bbox[13], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.back_box.xmax, bbox[14], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.back_box.ymax, bbox[15], 1e-5);

  // fill_lights
  fill_lights(false, obj, bbox + 16);
  ASSERT_NEAR(obj->car_light.brake_visible, 0, 1e-5);
  ASSERT_NEAR(obj->car_light.brake_switch_on, 0, 1e-5);
  ASSERT_NEAR(obj->car_light.left_turn_visible, 0, 1e-5);
  ASSERT_NEAR(obj->car_light.left_turn_switch_on, 0, 1e-5);
  ASSERT_NEAR(obj->car_light.right_turn_visible, 0, 1e-5);
  ASSERT_NEAR(obj->car_light.right_turn_switch_on, 0, 1e-5);
  fill_lights(true, obj, bbox + 16);
  ASSERT_NEAR(obj->car_light.brake_visible, bbox[16], 1e-5);
  ASSERT_NEAR(obj->car_light.brake_switch_on, bbox[17], 1e-5);
  ASSERT_NEAR(obj->car_light.left_turn_visible, bbox[18], 1e-5);
  ASSERT_NEAR(obj->car_light.left_turn_switch_on, bbox[19], 1e-5);
  ASSERT_NEAR(obj->car_light.right_turn_visible, bbox[20], 1e-5);
  ASSERT_NEAR(obj->car_light.right_turn_switch_on, bbox[21], 1e-5);

  // fill_ratios
  fill_ratios(false, obj, bbox + 22);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[0], 0, 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[1], 0, 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[2], 0, 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[3], 0, 1e-5);
  ASSERT_NEAR(obj->camera_supplement.cut_off_ratios[0], 0, 1e-5);
  ASSERT_NEAR(obj->camera_supplement.cut_off_ratios[1], 0, 1e-5);
  fill_ratios(true, obj, bbox + 22);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[0], bbox[22], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[1], bbox[23], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[2], bbox[24], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.visible_ratios[3], bbox[25], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.cut_off_ratios[0], bbox[26], 1e-5);
  ASSERT_NEAR(obj->camera_supplement.cut_off_ratios[1], bbox[27], 1e-5);
}

TEST(YoloCameraDetectorTest, nms_test) {
  {
    std::vector<NormalizedBBox> test_objects;
    NormalizedBBox obj_ped1;
    obj_ped1.xmin = .1f;
    obj_ped1.xmax = .3f;
    obj_ped1.ymin = .20f;
    obj_ped1.ymax = .60f;
    obj_ped1.score = 0.9f;
    obj_ped1.label = static_cast<int>(base::ObjectType::PEDESTRIAN);

    NormalizedBBox obj_ped2;
    obj_ped2.xmin = .10f;
    obj_ped2.xmax = .25f;
    obj_ped2.ymin = .30f;
    obj_ped2.ymax = .60f;
    obj_ped2.score = 0.8f;
    obj_ped2.label = static_cast<int>(base::ObjectType::PEDESTRIAN);
    NormalizedBBox obj_ped3;
    obj_ped3.xmin = .7f;
    obj_ped3.xmax = .8f;
    obj_ped3.ymin = .7f;
    obj_ped3.ymax = .8f;
    obj_ped3.score = 0.01f;
    obj_ped3.label = static_cast<int>(base::ObjectType::PEDESTRIAN);

    test_objects.push_back(obj_ped1);
    test_objects.push_back(obj_ped2);
    test_objects.push_back(obj_ped3);

    std::vector<float> scores;
    scores.push_back(obj_ped1.score);
    scores.push_back(obj_ped2.score);
    scores.push_back(obj_ped3.score);

    std::vector<int> indices;
    apply_softnms_fast(test_objects, &scores, 0.1f, 0.5f, 20, &indices, true,
                       0.4f);
    CHECK_LT(scores[1], 0.8f);
    scores[1] = 0.8f;
    scores[2] = 0.01f;
    apply_softnms_fast(test_objects, &scores, 0.1f, 0.5f, 20, &indices, false,
                       0.4f);
    CHECK_LT(scores[1], 0.8f);

    scores[1] = 0.8f;
    scores[2] = 0.01f;
    apply_boxvoting_fast(&test_objects, &scores, 0.1f, 0.5f, 0.4f, &indices);
    CHECK_LT(test_objects[0].ymin, .30f);
    CHECK_GT(test_objects[0].ymin, .20f);
    CHECK_LT(test_objects[0].xmax, .30f);
    CHECK_GT(test_objects[0].xmax, .20f);
    CHECK_LT(scores[1], 0.8f);

    scores[1] = 0.8f;
    scores[2] = 0.01f;
    apply_boxvoting_fast(&test_objects, &scores, 0.1f, 0.5f, 0, &indices);
    CHECK_LT(test_objects[0].ymin, .30f);
    CHECK_GT(test_objects[0].ymin, .20f);
    CHECK_LT(test_objects[0].xmax, .30f);
    CHECK_GT(test_objects[0].xmax, .20f);

    scores[0] = 0.00001f;
    scores[1] = 0.00001f;
    scores[2] = 0.00001f;
    apply_boxvoting_fast(&test_objects, &scores, 0, 0.5f, 0, &indices);

    std::vector<NormalizedBBox> test_empty_objects;
    std::vector<float> test_empty_scores;

    apply_boxvoting_fast(&test_empty_objects, &test_empty_scores, 0.1f, 0.5f,
                         0.4f, &indices);
  }
  {
    std::vector<NormalizedBBox> test_objects;
    NormalizedBBox obj_ped1;
    obj_ped1.xmin = .1f;
    obj_ped1.xmax = .3f;
    obj_ped1.ymin = .20f;
    obj_ped1.ymax = .60f;
    obj_ped1.score = 0.9f;
    obj_ped1.label = static_cast<int>(base::ObjectType::PEDESTRIAN);

    NormalizedBBox obj_ped2;
    obj_ped2.xmin = .10f;
    obj_ped2.xmax = .25f;
    obj_ped2.ymin = .30f;
    obj_ped2.ymax = .60f;
    obj_ped2.score = 0.8f;
    obj_ped2.label = static_cast<int>(base::ObjectType::PEDESTRIAN);
    NormalizedBBox obj_ped3;
    obj_ped3.xmin = .7f;
    obj_ped3.xmax = .8f;
    obj_ped3.ymin = .7f;
    obj_ped3.ymax = .8f;
    obj_ped3.score = 0.01f;
    obj_ped3.label = static_cast<int>(base::ObjectType::PEDESTRIAN);

    test_objects.push_back(obj_ped1);
    test_objects.push_back(obj_ped2);
    test_objects.push_back(obj_ped3);

    std::vector<float> scores;
    scores.push_back(obj_ped1.score);
    scores.push_back(obj_ped2.score);
    scores.push_back(obj_ped3.score);

    std::vector<int> indices;
    apply_nms_fast(test_objects, scores, 0.1f, 0.5f, 1.0f, 20, &indices);
    CHECK_EQ(indices.size(), size_t(1));
    apply_nms_fast(test_objects, scores, 0.1f, 0.5f, 0.7f, 20, &indices);
    CHECK_EQ(indices.size(), size_t(1));
    obj_ped3.xmin = .10f;
    obj_ped3.xmax = .25f;
    obj_ped3.ymin = .32f;
    obj_ped3.ymax = .60f;
    obj_ped3.score = 0.6f;
    apply_nms_fast(test_objects, scores, 0.1f, 0.6f, 1.2f, 20, &indices);
  }
  {
    std::vector<NormalizedBBox> test_objects;
    NormalizedBBox obj_cyc;
    obj_cyc.xmin = .1f;
    obj_cyc.xmax = .3f;
    obj_cyc.ymin = .20f;
    obj_cyc.ymax = .60f;
    obj_cyc.score = 0.9f;
    obj_cyc.label = static_cast<int>(base::ObjectType::BICYCLE);

    NormalizedBBox obj_ped;
    obj_ped.xmin = .10f;
    obj_ped.xmax = .25f;
    obj_ped.ymin = .30f;
    obj_ped.ymax = .60f;
    obj_ped.score = 0.95f;
    obj_ped.label = static_cast<int>(base::ObjectType::PEDESTRIAN);
    /*
        test_objects.push_back(obj_cyc);
        test_objects.push_back(obj_ped);

        std::vector<int> cyc_indices;
        std::vector<int> ped_indices;
        cyc_indices.push_back(0);
        ped_indices.push_back(1);
        cross_class_merge(&cyc_indices, &ped_indices, test_objects, 0.8);
        CHECK_EQ(cyc_indices.size(), 1);
        CHECK_EQ(ped_indices.size(), 0);*/
  }

  std::vector<base::ObjectPtr> visual_objects;
  for (int i = 0; i < 1; i++) {
    base::ObjectPtr obj;
    obj.reset(new base::Object);
    obj->camera_supplement.alpha = 0;
    obj->camera_supplement.box.xmin = 0.1f;
    obj->camera_supplement.box.ymin = 0.2f;
    obj->camera_supplement.box.xmax = 0.3f;
    obj->camera_supplement.box.ymax = 0.4f;
    obj->size[2] = 1.6f;
    obj->size[1] = 1.4f;
    obj->size[0] = 4.0f;
    obj->center[0] = 0;
    obj->center[1] = 0;
    obj->center[2] = 0;
    obj->theta = 1.1f;
    visual_objects.push_back(obj);
  }
  recover_bbox(10, 10, 5, &visual_objects);
  CHECK_EQ(visual_objects[0]->camera_supplement.box.xmin, 1);
  CHECK_EQ(visual_objects[0]->camera_supplement.box.ymin, 7);
  CHECK_EQ(visual_objects[0]->camera_supplement.box.xmax, 3);
  CHECK_EQ(visual_objects[0]->camera_supplement.box.ymax, 9);

  {
    std::vector<NormalizedBBox> test_empty_objects;
    std::vector<float> empty_scores;
    std::vector<int> empty_indices;

    apply_softnms_fast(test_empty_objects, &empty_scores, 0.1f, 0.5f, 20,
                       &empty_indices, true, 0.8f);
    CHECK_EQ(empty_indices.size(), size_t(0));
    apply_boxvoting_fast(&test_empty_objects, &empty_scores, 0.1f, 0.5f, 20,
                         &empty_indices);
    CHECK_EQ(empty_indices.size(), size_t(0));
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
