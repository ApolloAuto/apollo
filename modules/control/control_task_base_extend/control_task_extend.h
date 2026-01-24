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

/**
 * @file
 * @brief Defines the Controller base class.
 */

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/control_component/proto/control_debug.pb.h"

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/control/control_component/controller_task_base/control_task.h"
#include "modules/control/control_task_base_extend/common/exponential_smoothing.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class Controller
 *
 * @brief base class for all controllers.
 */
class ControlTaskExtend : public ControlTask {
public:
    /**
     * @brief constructor
     */
    ControlTaskExtend();

    /**
     * @brief destructor
     */
    virtual ~ControlTaskExtend() {}

    /**
     * @brief reset ControlTaskAgent
     * @return Status reset status
     */

    /**
     * @brief controller name
     * @return string controller name in string
     */
    std::string Name() const;

protected:
    template <typename T>
    bool LoadParamsPipelineConfig(T *pipeline_config);

    bool LoadCalibrationTable(const std::string &calibration_table_path, calibration_table *calibration_table_conf);

    bool VehicleStatusIdentificationUpdate(
            const localization::LocalizationEstimate *localization,
            const canbus::Chassis *chassis,
            const planning::ADCTrajectory *trajectory);
    bool IsForwardModel();
    bool IsEdgeFollow();
    bool CheckInPit(double pit_replan_check_time, double pit_replan_check_count, double vehicle_speed, bool replan);
    bool IsLargeCurvature(const double ref_curvature, const double min_large_ref_curvature, bool *is_in_large_curvature);
    bool IsLeftCurvature(const double ref_curvature);
    double GetVehicleSpeed(
            std::shared_ptr<DependencyInjector> injector,
            const bool &use_filter,
            const double &filter_coeff);
    double ThrottleCmdFilter(const bool &use_filter, const double &filter_coeff, const double &throttle);

private:
    const localization::LocalizationEstimate *localization_ = nullptr;
    const canbus::Chassis *chassis_ = nullptr;
    const planning::ADCTrajectory *trajectory_message_ = nullptr;
    const std::string name_;

    double vehicle_speed_last_ = 0.0;
    double throttle_last_ = 0.0;
};

template <typename T>
bool ControlTaskExtend::LoadParamsPipelineConfig(T *pipeline_config) {
    int status;
    std::string class_name = abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    AINFO << "class_name is " << class_name;
    // Generate the default task config path from PluginManager.
    std::string pipeline_config_path
            = apollo::cyber::plugin_manager::PluginManager::Instance()->GetPluginConfPath<ControlTask>(
                    class_name, "conf/params_pipeline.pb.txt");
    AINFO << "pipeline_config_path is " << pipeline_config_path;

    if (!apollo::cyber::common::GetProtoFromFile(pipeline_config_path, pipeline_config)) {
        AERROR << "Load config of " << class_name << " failed!";
        return false;
    }
    AINFO << "Load the [" << class_name << "] params pipeline file successfully, file path: " << pipeline_config_path;
    return true;
}

}  // namespace control
}  // namespace apollo
