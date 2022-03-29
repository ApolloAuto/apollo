#include <memory>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/examples/component/proto/examples.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::TimerComponent;
using apollo::cyber::examples::component::proto::Driver;
using apollo::cyber::Writer;

class distance: public TimerComponent {
public:
    bool Init() override;
    bool Proc() override;
private:
    std::shared_ptr<apollo::cyber::Writer<Driver>> distance_writer_;
};

CYBER_REGISTER_COMPONENT(distance)
