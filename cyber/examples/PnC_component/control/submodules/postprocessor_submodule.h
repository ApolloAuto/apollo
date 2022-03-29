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

/**
 * @file postprocessor_submodule.h
 */

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/control/controller/controller.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_common_conf.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace control {
class PostprocessorSubmodule final : public cyber::Component<ControlCommand> {
 public:
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

  /**
   * @brief
   *
   * @param control_command
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<ControlCommand>& control_command) override;

 private:
  std::shared_ptr<cyber::Writer<ControlCommand>> postprocessor_writer_;
  ControlCommonConf control_common_conf_;
};

CYBER_REGISTER_COMPONENT(PostprocessorSubmodule)

}  // namespace control
}  // namespace apollo
