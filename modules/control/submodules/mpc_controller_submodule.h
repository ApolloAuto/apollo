/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/control/controller/controller.h"
#include "modules/control/controller/mpc_controller.h"
#include "modules/control/proto/calibration_table.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace control {
class MPCControllerSubmodule final : public cyber::Component<Preprocessor> {
 public:
  /**
   * @brief Construct a new MPCControllerSubmodule object
   *
   */
  MPCControllerSubmodule();
  /**
   * @brief Destructor
   */
  ~MPCControllerSubmodule();

  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  std::string Name() const;

  /**
   * @brief Initialize the node
   * @return If initialized
   */
  bool Init() override;

  /**
   * @brief generate control command
   *
   * @return true control command is successfully generated
   * @return false fail to generate control command
   */
  bool Proc(const std::shared_ptr<Preprocessor>& preprocessor_status) override;

 private:
  common::Status ProduceControlCoreCommand(
      const LocalView& local_view, ControlCommand* control_core_command);

 private:
  bool estop_ = false;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  MPCController mpc_controller_;

  std::mutex mutex_;
  // TODO(SHU): separate conf
  ControlConf mpc_controller_conf_;
  std::shared_ptr<cyber::Writer<ControlCommand>> control_core_writer_;
};

CYBER_REGISTER_COMPONENT(MPCControllerSubmodule)

}  // namespace control
}  // namespace apollo
