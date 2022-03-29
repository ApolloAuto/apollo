#include <memory>
// 使用普通component模板
#include "cyber/component/component.h"
#include "cyber/examples/component/proto/examples.pb.h"
// 使用TimerComponent中的头文件
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"

namespace apollo {

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::component::proto::Driver;
// 使用TimerComponent的命令空间
using apollo::cyber::TimerComponent;
//using apollo::cyber::Writer;

class cal1 : public Component<Driver> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0) override;
private:
  std::shared_ptr<cyber::Writer<Driver>> cal1_writer_ ;
  std::shared_ptr<cyber::Reader<Driver>> cal1_reader_;
  Driver cal1_msg_;
};

CYBER_REGISTER_COMPONENT(cal1)

} // namespace apollo