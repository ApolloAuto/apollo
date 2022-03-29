#include "cyber/examples/component/cal1/cal1.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/examples/component/proto/examples.pb.h"

namespace apollo {

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::component::proto::Driver;

bool cal1::Init() {
    AINFO << "cal1 component init";
    cal1_writer_ = node_ -> CreateWriter<Driver>("/carstatus/speed2");
    cal1_reader_ = node_ -> CreateReader<Driver>(
        "/carstatus/speed1",
        [this] (const std::shared_ptr<Driver>& cal1_msg) {
            AINFO << "driver msg ->" << cal1_msg->content();
            cal1_msg_.CopyFrom(*cal1_msg);
        });
    return true;
}

bool cal1::Proc(const std::shared_ptr<Driver>& msg0){
    AINFO << "Start cal1 component Proc [" << msg0->content() << " ]" ;
    static int i =0;
    auto out_msg = std::make_shared<Driver>();
    if (msg0->content() > 100) {
        out_msg->set_content(1);
    }
    else {
        out_msg->set_content(0);
    }
    out_msg->set_timestamp(i++);
    cal1_writer_ -> Write(out_msg);
    AINFO << "cal1: Write drivermsg -> " << out_msg->content();
    return true;
}

} // namespace apollo