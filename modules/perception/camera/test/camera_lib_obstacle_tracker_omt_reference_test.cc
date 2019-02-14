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
#include "modules/perception/camera/lib/obstacle/tracker/omt/obstacle_reference.h"

#include "cyber/common/file.h"
#include "gtest/gtest.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/camera/common/object_template_manager.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/frame_list.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/omt.pb.h"
#include "modules/perception/camera/lib/obstacle/tracker/omt/target.h"

namespace apollo {
namespace perception {
namespace camera {
TEST(RefTest, update_test) {
  // Init object template
  ObjectTemplateManagerInitOptions object_template_init_options;
  object_template_init_options.root_dir =
      "/apollo/modules/perception/testdata/"
      "camera/app/data/perception/camera/common/object_template/";
  object_template_init_options.conf_file = "object_template.pt";
  CHECK(ObjectTemplateManager::Instance()->Init(object_template_init_options));

  ObstacleReference ref;
  omt::OmtParam omt_param;
  std::string omt_config = apollo::cyber::common::GetAbsolutePath(
      "/apollo/modules/perception/testdata/"
      "camera/lib/obstacle/tracker/omt/data/models/omt_obstacle_tracker",
      "config.pt");
  CHECK(apollo::cyber::common::GetProtoFromFile(omt_config, &omt_param));
  ref.Init(omt_param.reference(), 1920.0f, 1080.0f);
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

  // first frame with one car object
  base::ObjectPtr object(new base::Object);
  base::BBox2DF bbox(500, 500, 1000, 1000);
  object->camera_supplement.box = bbox;
  object->type = base::ObjectType::VEHICLE;
  object->sub_type = base::ObjectSubType::CAR;
  object->size << 1.6, 1.7, 3.8;

  frame.detected_objects.push_back(object);
  // new target
  Target target(omt_param.target_param());
  std::vector<Target> targets;
  targets.push_back(target);
  targets[0].id = 0;
  PatchIndicator indicator(0, 0, sensor_name);
  TrackObjectPtr track_obj(new TrackObject);
  track_obj->indicator = indicator;
  track_obj->object = object;
  track_obj->timestamp = 0;
  targets[0].Add(track_obj);

  ref.CorrectSize(&frame);
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 0);
  targets[0].Add(track_obj);
  targets[0].Add(track_obj);
  targets[0].Add(track_obj);
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  object->sub_type = base::ObjectSubType::UNKNOWN;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  object->sub_type = base::ObjectSubType::CAR;
  object->camera_supplement.box.xmin = 0;
  object->camera_supplement.box.xmax = 100;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  object->camera_supplement.box.xmax = 500;
  object->camera_supplement.box.ymax = 500;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  track_obj->indicator.sensor_name = "asdfasdf";
  object->camera_supplement.box.ymax = 900;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  track_obj->indicator.sensor_name = sensor_name;
  object->camera_supplement.box.ymax = 599;
  frame.camera_k_matrix(1, 2) = 600;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  object->camera_supplement.box.xmin = 1700;
  object->camera_supplement.box.xmax = 1800;
  object->camera_supplement.box.ymax = 650;
  frame.camera_k_matrix(1, 2) = 600;
  ref.UpdateReference(&frame, targets);
  CHECK_EQ(ref.reference_[sensor_name].size(), 1);
  object->size << 0.6, 0.7, 1.5;
  ref.CorrectSize(&frame);
  CHECK(Equal(1.38566f, object->size[2], 0.01f));
  object->size << 4, 2, 0.8;
  ref.CorrectSize(&frame);
  CHECK(Equal(1.38566f, object->size[2], 0.01f));

  object->size << 5.6, 5.7, 1.5;
  ref.CorrectSize(&frame);
  CHECK(Equal(1.68300f, object->size[2], 0.01f));
  object->size << 4, 2, 4;
  ref.CorrectSize(&frame);
  CHECK(Equal(1.68300f, object->size[2], 0.01f));
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
