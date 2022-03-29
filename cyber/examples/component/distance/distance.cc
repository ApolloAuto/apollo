#include "cyber/examples/component/distance/distance.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/examples/component/proto/examples.pb.h"

bool distance::Init() {
    distance_writer_ = node_->CreateWriter<Driver>("/carstatus/distance1");
    return true;
}

bool distance::Proc() {
    static int i = 0;
    auto out_msg = std::make_shared<Driver>();
    out_msg ->set_msg_id(i++);
    out_msg ->set_content(70);
    distance_writer_ ->Write(out_msg);
    AINFO << "distance: Write drivermsg->" << out_msg->content() << " msg id -> " << out_msg->msg_id();
    return true;
}