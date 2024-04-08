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

#include "modules/prediction/common/prediction_system_gflags.h"

#include <limits>

// System gflags
DEFINE_string(prediction_module_name, "prediction",
              "Default prediction module name");
DEFINE_string(prediction_conf_file,
              "/apollo/modules/prediction/conf/prediction_conf.pb.txt",
              "Default conf file for prediction");
DEFINE_string(prediction_adapter_config_filename,
              "/apollo/modules/prediction/conf/adapter.conf",
              "Default conf file for prediction");
DEFINE_string(prediction_data_dir,
              "/apollo/modules/prediction/data/prediction/",
              "Prefix of files to store feature data");
DEFINE_string(offline_feature_proto_file_name, "",
              "The bin file including a series of feature proto messages");
DEFINE_string(output_filename, "", "The filename for offline process.");
DEFINE_string(extract_feature_type, "",
              "The extract feature type, either cruise or junction");

DEFINE_bool(prediction_test_mode, false, "Set prediction to test mode");
DEFINE_double(
    prediction_test_duration, std::numeric_limits<double>::infinity(),
    "The runtime duration in test mode (in seconds). Negative value will not "
    "restrict the runtime duration.");

DEFINE_string(
    prediction_offline_bags, "",
    "a list of bag files or directories for offline mode. The items need to be "
    "separated by colon ':'.  If this value is not set, the prediction module "
    "will use the listen to published ros topic mode.");
DEFINE_int32(prediction_offline_mode, 0,
             "0: online mode, no dump file"
             "1: dump feature proto to feature.*.bin"
             "2: dump data for learning to datalearn.*.bin"
             "3: dump predicted trajectory to predict_result.*.bin"
             "4: dump frame environment info to frame_env.*.bin"
             "5: dump data for tuning to datatuning.*.bin");
DEFINE_bool(enable_multi_agent_pedestrian_evaluator, true, "If enable multi agent pedestrian evaluator.");
DEFINE_bool(enable_multi_agent_vehicle_evaluator, true, "If enable multi agent vehicle evaluator.");
DEFINE_bool(prediction_eval_mode, false, "Set prediction to eval mode");
DEFINE_bool(enable_multi_thread, true, "If enable multi-thread.");
DEFINE_int32(max_thread_num, 8, "Maximal number of threads.");
DEFINE_int32(max_caution_thread_num, 2,
             "Maximal number of threads for caution obstacles.");
DEFINE_bool(enable_async_draw_base_image, true,
            "If enable async to draw base image");
DEFINE_bool(use_cuda, true, "If use cuda for torch.");

// Bag replay timestamp gap
DEFINE_double(replay_timestamp_gap, 10.0,
              "Max timestamp gap for rosbag replay");
DEFINE_int32(max_num_dump_feature, 10000, "Max number of features to dump");
DEFINE_int32(max_num_dump_dataforlearn, 5000,
             "Max number of dataforlearn to dump");

// Submodules
DEFINE_bool(use_lego, false, "If use lego architecture");
DEFINE_string(container_topic_name, "/apollo/prediction/container",
              "Container topic name");
DEFINE_string(adccontainer_topic_name, "/apollo/prediction/adccontainer",
              "ADC Container topic name");
DEFINE_string(evaluator_topic_name, "/apollo/prediction/evaluator",
              "Evaluator topic name");
DEFINE_string(container_submodule_name, "container_submodule",
              "Container submodule name");
DEFINE_string(evaluator_submodule_name, "evaluator_submodule",
              "Evaluator submodule name");
DEFINE_string(perception_obstacles_topic_name,
              "/apollo/prediction/perception_obstacles",
              "Internal topic of perception obstacles");

// VectorNet
DEFINE_string(prediction_target_file, "/apollo/data/train/test.pb.txt",
              "VectorNet target pb file name");
DEFINE_string(world_coordinate_file, "/apollo/data/world_coord.bin",
              "VectorNet world coordinate file name");
DEFINE_string(prediction_target_dir, "/apollo/data/train/",
              "VectorNet target dir");
DEFINE_double(obstacle_x, 0.0, "obstacle position x");
DEFINE_double(obstacle_y, 0.0, "obstacle position y");
DEFINE_double(obstacle_phi, 0.0, "obstacle heading phi");
DEFINE_double(road_distance, 100.0,
              "road distance within which the points are got");
DEFINE_double(point_distance, 5.0,
              "sampling distance of two points");
