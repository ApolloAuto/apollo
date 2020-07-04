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
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"

#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt.pb.h"

namespace apollo {
namespace perception {
namespace camera {

TEST(TargetTest, target_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  ACHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  omt::OmtParam omt_param;
  std::string omt_config = cyber::common::GetAbsolutePath(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/tracker/omt/data/models/"
      "omt_obstacle_tracker",
      "config.pt");

  ASSERT_TRUE(cyber::common::GetProtoFromFile(omt_config, &omt_param));

  // first frame with one car object
  std::string sensor_name = "onsemi_obstacle";
  DataProvider provider;
  DataProvider::InitOptions options;
  options.image_width = 1920;
  options.image_height = 1080;
  options.do_undistortion = false;
  options.device_id = 0;
  options.sensor_name = sensor_name;
  provider.Init(options);
  CameraFrame frame;
  frame.data_provider = &provider;
  frame.timestamp = 0;
  base::ObjectPtr object(new base::Object);
  base::BBox2DF bbox(500, 500, 1000, 1000);
  object->camera_supplement.box = bbox;
  object->camera_supplement.projected_box = bbox;
  object->type = base::ObjectType::VEHICLE;
  object->sub_type = base::ObjectSubType::CAR;
  object->size << 4.12469f, 1.38566f, 1.49074f;
  object->camera_supplement.local_center << 18, 30, 24;
  frame.detected_objects.push_back(object);
  frame.frame_id = 0;

  // new target
  Target target(omt_param.target_param());
  target.id = 0;
  PatchIndicator indicator(0, 0, sensor_name);
  TrackObjectPtr track_obj(new TrackObject);
  track_obj->indicator = indicator;
  track_obj->object = object;
  track_obj->timestamp = 0;
  target.Add(track_obj);
  ASSERT_EQ(target.Size(), 1);
  // predict with delta t = 0
  target.lost_age = 0;
  target.Update2D(&frame);
  target.Update3D(&frame);
  target.UpdateType(&frame);
  ASSERT_EQ(target.get_object(-1)->object->sub_type, base::ObjectSubType::CAR);
  // auto state = target.image_center.get_state();
  // second frame

  CameraFrame frame1;
  frame1.timestamp = 1;
  frame1.data_provider = &provider;
  base::ObjectPtr object1(new base::Object);
  base::BBox2DF bbox1(540, 540, 960, 960);
  object1->camera_supplement.box = bbox1;
  object1->camera_supplement.projected_box = bbox1;
  object1->type = base::ObjectType::VEHICLE;
  object1->sub_type = base::ObjectSubType::CAR;
  object->size << 1.5f, 1.6f, 3.7f;
  object->camera_supplement.local_center << 18, 30, 24;
  frame1.detected_objects.push_back(object1);
  frame1.frame_id = 1;
  target.Predict(&frame1);
  TrackObjectPtr track_obj1(new TrackObject);
  track_obj1->indicator = PatchIndicator(1, 0, sensor_name);
  track_obj1->object = object1;
  track_obj1->timestamp = 1;
  target.Add(track_obj1);

  target.lost_age = 0;
  target.UpdateType(&frame1);
  ASSERT_EQ(target.get_object(-1)->object->sub_type, base::ObjectSubType::CAR);
  target.Update2D(&frame1);
  target.Update3D(&frame1);

  // third frame with no object

  CameraFrame frame2;
  frame2.data_provider = &provider;
  frame2.timestamp = 2;
  frame2.frame_id = 2;
  target.Predict(&frame2);
  target.lost_age = 1;
  target.UpdateType(&frame2);
  target.Update2D(&frame2);
  target.Update3D(&frame2);

  // fourth frame

  CameraFrame frame3;

  base::ObjectPtr object3(new base::Object);

  base::BBox2DF bbox3(540, 540, 960, 960);
  object3->camera_supplement.box = bbox3;
  object3->camera_supplement.projected_box = bbox3;
  object3->type = base::ObjectType::UNKNOWN_UNMOVABLE;
  object3->sub_type = base::ObjectSubType::TRAFFICCONE;
  frame3.detected_objects.push_back(object3);
  frame3.frame_id = 3;
  frame3.timestamp = 3;
  frame3.data_provider = &provider;
  target.Predict(&frame3);
  TrackObjectPtr track_obj3(new TrackObject);
  track_obj3->indicator = PatchIndicator(3, 0, sensor_name);
  track_obj3->object = object3;
  track_obj3->timestamp = 3;
  target.Add(track_obj3);
  target.UpdateType(&frame3);
  target.lost_age = 0;
  target.UpdateType(&frame3);
  ASSERT_EQ(target.get_object(-1)->object->sub_type, base::ObjectSubType::CAR);
  target.Update2D(&frame3);
  target.Update3D(&frame3);

  ASSERT_EQ(target.Size(), 3);
  target.Clear();
  ASSERT_EQ(target.Size(), 0);

  // first frame with one unknown object
  CameraFrame frame5;
  base::ObjectPtr object5(new base::Object);
  base::BBox2DF bbox5(500, 500, 1000, 1000);
  object5->camera_supplement.box = bbox5;
  object5->camera_supplement.projected_box = bbox5;
  object5->type = base::ObjectType::UNKNOWN_UNMOVABLE;
  object5->sub_type = base::ObjectSubType::TRAFFICCONE;
  object5->size << 1.6f, 1.7f, 3.8f;
  object5->camera_supplement.local_center << 18, 30, 24;
  frame5.detected_objects.push_back(object5);
  frame5.frame_id = 0;
  frame5.timestamp = 5;
  frame5.data_provider = &provider;
  // new target
  Target target1(omt_param.target_param());
  target1.id = 1;
  TrackObjectPtr track_obj5(new TrackObject);
  track_obj5->indicator = PatchIndicator(5, 0, sensor_name);
  track_obj5->object = object5;
  track_obj5->timestamp = 5;
  target1.Add(track_obj5);
  ASSERT_EQ(target1.Size(), 1);
  target1.lost_age = 0;
  target1.UpdateType(&frame5);
  ASSERT_EQ(target1.get_object(-1)->object->sub_type,
            base::ObjectSubType::TRAFFICCONE);
  target1.Update2D(&frame5);
  // unchanged template
  ASSERT_FLOAT_EQ(target1.get_object(-1)->object->size[0], 1.6);
  ASSERT_FLOAT_EQ(target1.get_object(-1)->object->size[1], 1.7);
  ASSERT_FLOAT_EQ(target1.get_object(-1)->object->size[2], 3.8);
  target1.Update3D(&frame5);
  ASSERT_EQ(target1.Size(), 1);
}

TEST(TargetTest, clapping_velocity_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  ACHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  omt::OmtParam omt_param;
  std::string omt_config = cyber::common::GetAbsolutePath(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/tracker/omt/data/models/omt_obstacle_tracker",
      "config.pt");

  ASSERT_TRUE(cyber::common::GetProtoFromFile(omt_config, &omt_param));

  auto read_pos_and_theta_vec =
      [](const std::string& fname) -> std::vector<std::vector<double> > {
    std::vector<std::vector<double> > pos_and_theta_vec;
    std::ifstream fin(fname);
    if (!fin.is_open()) {
      AERROR << "Cannot open : " << fname;
      return pos_and_theta_vec;
    }
    int obj_num = 0;
    fin >> obj_num;
    for (int i = 0; i < obj_num; ++i) {
      std::vector<double> pos_theta(4, 0.0);
      fin >> pos_theta[0] >> pos_theta[1] >> pos_theta[2] >> pos_theta[3];
      pos_and_theta_vec.push_back(pos_theta);
    }
    fin.close();
    return pos_and_theta_vec;
  };

  // case 1
  {
    std::string sensor_name = "onsemi_obstacle";
    DataProvider provider;
    DataProvider::InitOptions options;
    options.image_width = 1920;
    options.image_height = 1080;
    options.do_undistortion = false;
    options.device_id = 0;
    options.sensor_name = sensor_name;
    provider.Init(options);

    // new target
    Target target(omt_param.target_param());
    target.id = 1;
    std::string pos_theta_filename =
        "/apollo/modules/perception/testdata/"
        "camera/lib/obstacle/tracker/omt/test_data/target_pos_theta1.txt";
    auto pos_and_theta_vec = read_pos_and_theta_vec(pos_theta_filename);
    for (size_t i = 0; i < pos_and_theta_vec.size(); ++i) {
      double ts = static_cast<double>(i) * 0.066;
      CameraFrame frame;
      base::ObjectPtr object(new base::Object);
      base::BBox2DF bbox(100, 100, 200, 200);
      object->camera_supplement.box = bbox;
      object->camera_supplement.projected_box = bbox;
      object->type = base::ObjectType::VEHICLE;
      object->sub_type = base::ObjectSubType::CAR;
      object->size << 3.8f, 1.8f, 1.6f;
      object->camera_supplement.local_center << 20, 30, 30;
      object->theta = static_cast<float>(pos_and_theta_vec[i][3]);
      object->center << pos_and_theta_vec[i][0], pos_and_theta_vec[i][1],
          pos_and_theta_vec[i][2];

      frame.detected_objects.push_back(object);
      frame.frame_id = static_cast<int>(i);
      frame.timestamp = ts;
      frame.data_provider = &provider;
      if (i > 0) {
        target.Predict(&frame);
      }

      TrackObjectPtr track_obj(new TrackObject);
      track_obj->indicator =
          PatchIndicator(static_cast<int>(i), 0, sensor_name);
      track_obj->object = object;
      track_obj->timestamp = ts;
      target.Add(track_obj);
      ASSERT_EQ(target.Size(), i + 1);
      target.lost_age = 0;
      target.UpdateType(&frame);
      ASSERT_EQ(target.get_object(-1)->object->sub_type,
                base::ObjectSubType::CAR);
      target.Update2D(&frame);
      // unchanged template
      target.Update3D(&frame);
      ASSERT_EQ(target.Size(), i + 1);
    }
  }

  // case 2 -- no predict
  {
    std::string sensor_name = "onsemi_obstacle";
    DataProvider provider;
    DataProvider::InitOptions options;
    options.image_width = 1920;
    options.image_height = 1080;
    options.do_undistortion = false;
    options.device_id = 0;
    options.sensor_name = sensor_name;
    provider.Init(options);

    // new target
    Target target(omt_param.target_param());
    target.id = 1;
    std::string pos_theta_filename =
        "/apollo/modules/perception/testdata/"
        "camera/lib/obstacle/tracker/omt/test_data/target_pos_theta1.txt";
    auto pos_and_theta_vec = read_pos_and_theta_vec(pos_theta_filename);
    for (size_t i = 0; i < pos_and_theta_vec.size(); ++i) {
      CameraFrame frame;
      base::ObjectPtr object(new base::Object);
      base::BBox2DF bbox(500, 500, 1000, 1000);
      object->camera_supplement.box = bbox;
      object->camera_supplement.projected_box = bbox;
      object->type = base::ObjectType::VEHICLE;
      object->sub_type = base::ObjectSubType::CAR;
      object->size << 1.6f, 1.7f, 3.8f;
      object->camera_supplement.local_center << 18, 30, 24;
      object->theta = static_cast<float>(pos_and_theta_vec[i][3]);
      object->center << pos_and_theta_vec[i][0], pos_and_theta_vec[i][1],
          pos_and_theta_vec[i][2];

      frame.detected_objects.push_back(object);
      frame.frame_id = static_cast<int>(i);
      frame.timestamp = static_cast<double>(i) * 0.066;
      frame.data_provider = &provider;

      TrackObjectPtr track_obj(new TrackObject);
      track_obj->indicator =
          PatchIndicator(static_cast<int>(i), 0, sensor_name);
      track_obj->object = object;
      track_obj->timestamp = static_cast<double>(i) * 0.066;
      target.Add(track_obj);
      ASSERT_EQ(target.Size(), i + 1);
      target.lost_age = 0;
      target.UpdateType(&frame);
      ASSERT_EQ(target.get_object(-1)->object->sub_type,
                base::ObjectSubType::CAR);
      target.Update2D(&frame);
      // unchanged template
      target.Update3D(&frame);
      ASSERT_EQ(target.Size(), i + 1);
    }
  }

  // case 3 -- long term track
  {
    std::string sensor_name = "onsemi_obstacle";
    DataProvider provider;
    DataProvider::InitOptions options;
    options.image_width = 1920;
    options.image_height = 1080;
    options.do_undistortion = false;
    options.device_id = 0;
    options.sensor_name = sensor_name;
    provider.Init(options);

    // new target
    Target target(omt_param.target_param());
    target.id = 1;
    std::string pos_theta_filename =
        "/apollo/modules/perception/testdata/"
        "camera/lib/obstacle/tracker/omt/test_data/target_pos_theta2.txt";
    auto pos_and_theta_vec = read_pos_and_theta_vec(pos_theta_filename);
    for (size_t i = 0; i < pos_and_theta_vec.size(); ++i) {
      double ts = static_cast<double>(i) * 0.066;
      CameraFrame frame;
      base::ObjectPtr object(new base::Object);
      base::BBox2DF bbox(100, 100, 200, 200);
      object->camera_supplement.box = bbox;
      object->camera_supplement.projected_box = bbox;
      object->type = base::ObjectType::VEHICLE;
      object->sub_type = base::ObjectSubType::CAR;
      object->size << 3.8f, 1.8f, 1.6f;
      object->camera_supplement.local_center << 20, 30, 30;
      object->theta = static_cast<float>(pos_and_theta_vec[i][3]);
      object->center << pos_and_theta_vec[i][0], pos_and_theta_vec[i][1],
          pos_and_theta_vec[i][2];

      frame.detected_objects.push_back(object);
      frame.frame_id = static_cast<int>(i);
      frame.timestamp = ts;
      frame.data_provider = &provider;
      if (i > 0) {
        target.Predict(&frame);
      }

      TrackObjectPtr track_obj(new TrackObject);
      track_obj->indicator =
          PatchIndicator(static_cast<int>(i), 0, sensor_name);
      track_obj->object = object;
      track_obj->timestamp = ts;
      target.Add(track_obj);
      ASSERT_EQ(target.Size(), i + 1);
      target.lost_age = 0;
      target.UpdateType(&frame);
      ASSERT_EQ(target.get_object(-1)->object->sub_type,
                base::ObjectSubType::CAR);
      target.Update2D(&frame);
      target.Update3D(&frame);
      ASSERT_EQ(target.Size(), i + 1);
    }
  }

  // case 4 -- almost not move
  {
    std::string sensor_name = "onsemi_obstacle";
    DataProvider provider;
    DataProvider::InitOptions options;
    options.image_width = 1920;
    options.image_height = 1080;
    options.do_undistortion = false;
    options.device_id = 0;
    options.sensor_name = sensor_name;
    provider.Init(options);

    // new target
    Target target(omt_param.target_param());
    target.id = 1;
    std::string pos_theta_filename =
        "/apollo/modules/perception/testdata/"
        "camera/lib/obstacle/tracker/omt/test_data/target_pos_theta3.txt";
    auto pos_and_theta_vec = read_pos_and_theta_vec(pos_theta_filename);
    for (size_t i = 0; i < pos_and_theta_vec.size(); ++i) {
      double ts = static_cast<double>(i) * 0.033;
      CameraFrame frame;
      base::ObjectPtr object(new base::Object);
      base::BBox2DF bbox(100, 100, 200, 200);
      object->camera_supplement.box = bbox;
      object->camera_supplement.projected_box = bbox;
      object->type = base::ObjectType::VEHICLE;
      object->sub_type = base::ObjectSubType::CAR;
      object->size << 3.8f, 1.8f, 1.6f;
      object->camera_supplement.local_center << 20, 30, 30;
      object->theta = static_cast<float>(pos_and_theta_vec[i][3]);
      object->center << pos_and_theta_vec[i][0], pos_and_theta_vec[i][1],
          pos_and_theta_vec[i][2];

      frame.detected_objects.push_back(object);
      frame.frame_id = static_cast<int>(i);
      frame.timestamp = ts;
      frame.data_provider = &provider;
      if (i > 0) {
        target.Predict(&frame);
      }

      TrackObjectPtr track_obj(new TrackObject);
      track_obj->indicator =
          PatchIndicator(static_cast<int>(i), 0, sensor_name);
      track_obj->object = object;
      track_obj->timestamp = ts;
      target.Add(track_obj);
      ASSERT_EQ(target.Size(), i + 1);
      target.lost_age = 0;
      target.UpdateType(&frame);
      ASSERT_EQ(target.get_object(-1)->object->sub_type,
                base::ObjectSubType::CAR);
      target.Update2D(&frame);
      // unchanged template
      target.Update3D(&frame);
      ASSERT_EQ(target.Size(), i + 1);
    }
  }

  // case 5 -- pedestrian type
  {
    std::string sensor_name = "onsemi_obstacle";
    DataProvider provider;
    DataProvider::InitOptions options;
    options.image_width = 1920;
    options.image_height = 1080;
    options.do_undistortion = false;
    options.device_id = 0;
    options.sensor_name = sensor_name;
    provider.Init(options);

    // new target
    Target target(omt_param.target_param());
    target.id = 1;
    std::string pos_theta_filename =
        "/apollo/modules/perception/testdata/"
        "camera/lib/obstacle/tracker/omt/test_data/target_pos_theta4.txt";
    auto pos_and_theta_vec = read_pos_and_theta_vec(pos_theta_filename);
    for (size_t i = 0; i < pos_and_theta_vec.size(); ++i) {
      double ts = static_cast<double>(i) * 0.033;
      CameraFrame frame;
      base::ObjectPtr object(new base::Object);
      base::BBox2DF bbox(100, 100, 200, 200);
      object->camera_supplement.box = bbox;
      object->camera_supplement.projected_box = bbox;
      object->type = base::ObjectType::PEDESTRIAN;
      object->sub_type = base::ObjectSubType::PEDESTRIAN;
      object->size << 0.5f, 0.7f, 1.7f;
      object->camera_supplement.local_center << 20, 30, 30;
      object->theta = static_cast<float>(pos_and_theta_vec[i][3]);
      object->center << pos_and_theta_vec[i][0], pos_and_theta_vec[i][1],
          pos_and_theta_vec[i][2];

      frame.detected_objects.push_back(object);
      frame.frame_id = static_cast<int>(i);
      frame.timestamp = ts;
      frame.data_provider = &provider;
      if (i > 0) {
        target.Predict(&frame);
      }

      TrackObjectPtr track_obj(new TrackObject);
      track_obj->indicator =
          PatchIndicator(static_cast<int>(i), 0, sensor_name);
      track_obj->object = object;
      track_obj->timestamp = ts;
      target.Add(track_obj);
      ASSERT_EQ(target.Size(), i + 1);
      target.lost_age = 0;
      target.UpdateType(&frame);
      ASSERT_EQ(target.get_object(-1)->object->sub_type,
                base::ObjectSubType::PEDESTRIAN);
      target.Update2D(&frame);
      // unchanged template
      target.Update3D(&frame);
      ASSERT_EQ(target.Size(), i + 1);
    }
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
