
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
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace control {
class PostprocessorSubmodule final
    : public cyber::Component<Preprocessor, ControlCommand> {
 public:
  /**
   * @brief Construct a new PostprocessorSubmodule object
   *
   */
  PostprocessorSubmodule();
  /**
   * @brief Destructor
   */
  ~PostprocessorSubmodule();

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
   * @param preprocessor_status
   * @param control_command
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<Preprocessor>& preprocessor_status,
            const std::shared_ptr<ControlCommand>& control_command) override;

 private:
  std::shared_ptr<cyber::Writer<ControlCommand>> postprocessor_writer_;
};
}  // namespace control
}  // namespace apollo