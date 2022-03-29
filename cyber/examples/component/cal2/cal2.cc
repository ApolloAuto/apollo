#include "cyber/examples/component/cal2/cal2.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/examples/component/proto/examples.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::component::proto::Driver;

bool cal2::Init() {
    AINFO << "cal2 component init";
    cal2_writer_ = node_ -> CreateWriter<Driver>("/carstatus/distance2");
    cal2_reader1_ = node_ -> CreateReader<Driver>(
        "/carstatus/speed1",
        [this] (const std::shared_ptr<Driver>& cal2_msg0) {
            AINFO << "cal2 driver msg0 ->" << cal2_msg0->content();
            cal2_msg0_.CopyFrom(*cal2_msg0);
        });
    cal2_reader2_ = node_ -> CreateReader<Driver>(
        "/carstatus/distance1",
        [this] (const std::shared_ptr<Driver>& cal2_msg1) {
            AINFO << "cal2 driver msg1 ->" << cal2_msg1->content();
            cal2_msg1_.CopyFrom(*cal2_msg1);
        });
    return true;
}

bool cal2::Proc(const std::shared_ptr<Driver>& msg0,
                                const std::shared_ptr<Driver>& msg1){
    AINFO << "Start cal2 component Proc [" << msg0->content() << "] [" << msg1->content() << "]";
    static int i =0;
    auto out_msg = std::make_shared<Driver>();
    if (msg0->content() > 60 && msg1->content() < 80) {
        out_msg->set_content(1);
    }
    else {
        out_msg->set_content(0);
    }
    out_msg->set_timestamp(i++);
    cal2_writer_ -> Write(out_msg);
    AINFO << "cal2: Write drivermsg -> " << out_msg->content();
    return true;
}
