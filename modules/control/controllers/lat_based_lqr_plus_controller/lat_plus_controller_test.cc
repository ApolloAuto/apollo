/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/controllers/lat_based_lqr_plus_controller/lat_plus_controller.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::cyber::Clock;
using PlanningTrajectoryPb = planning::ADCTrajectory;
using LocalizationPb = localization::LocalizationEstimate;
using ChassisPb = canbus::Chassis;
using apollo::common::VehicleStateProvider;

class LatPlusControllerTest : public ::testing::Test, LatPlusController {
public:
    virtual void SetUp() {
        FLAGS_v = 3;
    }
};

bool test_lat_control() {
    std::string apollo_root_dir_str = "/apollo";
    std::string plugin_lib_path = "APOLLO_PLUGIN_LIB_PATH=" + apollo_root_dir_str + "/bazel-bin:/opt/apollo/neo/lib";
    std::cout << "plugin_lib_path:" << plugin_lib_path << std::endl;
    char lib_path_chars[100];
    memcpy(lib_path_chars, plugin_lib_path.c_str(), plugin_lib_path.size());
    lib_path_chars[plugin_lib_path.size()] = '\0';
    std::cout << "lib_path_chars:" << lib_path_chars << std::endl;
    putenv(lib_path_chars);
    std::string lat_controller_plugin_xml_file
            = "/apollo/modules/control/controllers/lat_based_lqr_plus_controller/plugins.xml";
    apollo::cyber::plugin_manager::PluginManager::Instance()->LoadPlugin(lat_controller_plugin_xml_file);

    std::unique_ptr<LatPlusController> lat_controller_;
    LatBaseLqrPlusControllerConf lat_based_lqr_plus_controller_conf_;
    ParamsPipeline params_pipeline_;
    std::vector<std::pair<std::string, LatBaseLqrPlusControllerConf>> params_list_;
    std::string current_param_config_name_ = "";

    double timestamp = Clock::NowInSeconds();
    lat_controller_.reset(new LatPlusController());
    std::shared_ptr<DependencyInjector> injector_ = std::make_shared<DependencyInjector>();

    std::string controllers_dir
            = "/apollo/modules/control/controllers/lat_based_lqr_plus_controller/lateral_controller_test/";
    std::string control_conf_file = controllers_dir + "conf/controller_conf.pb.txt";
    std::string params_pipeline_file = controllers_dir + "conf/params_pipeline.pb.txt";

    ACHECK(cyber::common::GetProtoFromFile(control_conf_file, &lat_based_lqr_plus_controller_conf_));
    ACHECK(cyber::common::GetProtoFromFile(params_pipeline_file, &params_pipeline_));

    lat_controller_->Init(injector_);

    return true;
}

TEST_F(LatPlusControllerTest, ComputeLateralErrors) {
    bool run_control_success = test_lat_control();
    EXPECT_TRUE(run_control_success);
}

}  // namespace control
}  // namespace apollo
