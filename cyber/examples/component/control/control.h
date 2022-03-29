#include <memory>

#include "cyber/component/component.h"
#include "cyber/examples/component/proto/examples.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::component::proto::Driver;
using apollo::cyber::TimerComponent;
using apollo::cyber::Writer;

class control:public Component<Driver,Driver> {
public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Driver>& msg0,
            const std::shared_ptr<Driver>& msg1) override;
private:
  std::shared_ptr<apollo::cyber::Reader<Driver>> control_reader1_;
  std::shared_ptr<apollo::cyber::Reader<Driver>> control_reader2_;
  std::shared_ptr<apollo::cyber::Writer<Driver>> control_writer_ ;
  Driver control_msg0_;
  Driver control_msg1_;
};

CYBER_REGISTER_COMPONENT(control)

