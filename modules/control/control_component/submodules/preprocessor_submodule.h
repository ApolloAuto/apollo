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

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/control_component/proto/preprocessor.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/util.h"
#include "modules/control/control_component/controller_task_base/common/dependency_injector.h"

namespace apollo {
namespace control {

class PreprocessorSubmodule final : public cyber::Component<LocalView> {
 public:
  /**
   * @brief Construct a new Preprocessor Submodule object
   *
   */
  PreprocessorSubmodule();
  /**
   * @brief Destructor
   */
  ~PreprocessorSubmodule();
  /**
   * @brief Get name of the node
   * @return Name of the node
   */
  std::string Name() const;

  /**
   * @brief Initialize the submodule
   * @return If initialized
   */
  bool Init() override;

  bool Proc(const std::shared_ptr<LocalView> &local_view) override;

 private:
  /**
   * @brief check controller submodule input (local_view)
   *
   * @param local_view
   * @return common::Status
   */
  common::Status CheckInput(LocalView *local_view);

  /**
   * @brief check time stamp
   *
   * @param local_view
   * @return common::Status
   */
  common::Status CheckTimestamp(const LocalView &local_view);

  common::Status ProducePreprocessorStatus(Preprocessor *preprocessor_status);

 private:
  double init_time_ = 0.0;

  bool estop_ = false;

  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  common::Header latest_replan_trajectory_header_;

  std::mutex mutex_;

  std::shared_ptr<cyber::Reader<LocalView>> local_view_reader_;

  std::shared_ptr<cyber::Writer<Preprocessor>> preprocessor_writer_;

  std::shared_ptr<DependencyInjector> injector_;
};

CYBER_REGISTER_COMPONENT(PreprocessorSubmodule);

}  // namespace control
}  // namespace apollo
