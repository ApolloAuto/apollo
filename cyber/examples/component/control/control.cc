#include "cyber/examples/component/control/control.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/examples/component/proto/examples.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::component::proto::Driver;

bool control::Init() {
    AINFO << "control component init";
    control_writer_ = node_ -> CreateWriter<Driver>("/carstatus/control");
    control_reader1_ = node_ ->CreateReader<Driver>(
        "/carstatus/speed2",
        [this] (const std::shared_ptr<Driver>& control_msg0) {
            AINFO << "control driver msg0 ->" << control_msg0->content();
            control_msg0_.CopyFrom(*control_msg0);
        });
    control_reader2_ = node_ ->CreateReader<Driver>(
        "/carstatus/distance2",
        [this] (const std::shared_ptr<Driver>& control_msg1) {
            AINFO << "control driver msg1 ->" << control_msg1->content();
            control_msg1_.CopyFrom(*control_msg1);
        });
    return true;
}

bool control::Proc(const std::shared_ptr<Driver>& msg0,
                                const std::shared_ptr<Driver>& msg1){
    AINFO << "Start control component Proc [" << msg0->content() << "] [" << msg1->content() << "]";
    static int i =0;
    auto out_msg = std::make_shared<Driver>();
    if (msg0->content() > 0 || msg1->content() >0) {
        out_msg->set_content(1);
    }
    else {
        out_msg->set_content(1);
    }
    out_msg->set_timestamp(i++);
    control_writer_ -> Write(out_msg);
    AINFO << "control: Write drivermsg -> " << out_msg->content();
    return true;
}