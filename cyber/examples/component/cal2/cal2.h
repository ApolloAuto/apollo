#include <memory>
// 使用普通component模板
#include "cyber/component/component.h"
#include "cyber/examples/component/proto/examples.pb.h"
// 使用TimerComponent中的头文件
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::component::proto::Driver;
// 使用TimerComponent的命令空间
using apollo::cyber::TimerComponent;
using apollo::cyber::Writer;

class cal2 : public Component<Driver, Driver> {
public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
private:
  std::shared_ptr<apollo::cyber::Reader<Driver>> cal2_reader1_ ;
  std::shared_ptr<apollo::cyber::Reader<Driver>> cal2_reader2_ ;
  std::shared_ptr<apollo::cyber::Writer<Driver>> cal2_writer_ ;
  Driver cal2_msg0_;
  Driver cal2_msg1_;
};

CYBER_REGISTER_COMPONENT(cal2)