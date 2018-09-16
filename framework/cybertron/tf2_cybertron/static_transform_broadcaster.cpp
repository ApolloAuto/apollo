#include "cybertron/tf2_cybertron/static_transform_broadcaster.h"
#include "cybertron/transport/qos/qos_profile_conf.h"

namespace apollo {
namespace cybertron {
namespace tf2_cybertron {

StaticTransformBroadcaster::StaticTransformBroadcaster(
    const std::shared_ptr<cybertron::Node>& node)
    : node_(node) {
  proto::RoleAttributes attr;
  attr.set_channel_name("/tf_static");
  attr.mutable_qos_profile()->CopyFrom(
      transport::QosProfileConf::QOS_PROFILE_TF_STATIC);
  publisher_ = node_->CreateWriter<adu::common::TransformStampeds>(attr);
  net_message_ = std::make_shared<adu::common::TransformStampeds>();
}

void StaticTransformBroadcaster::sendTransform(
    const adu::common::TransformStamped& msgtf) {
  std::vector<adu::common::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}

void StaticTransformBroadcaster::sendTransform(
    const std::vector<adu::common::TransformStamped>& msgtf) {
  for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in) {
    bool match_found = false;
    int size = net_message_->transforms_size();

    for (int i = 0; i < size; ++i) {
      if (it_in->child_frame_id() ==
          net_message_->mutable_transforms(i)->child_frame_id()) {
        auto it_msg = net_message_->mutable_transforms(i);
        *it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (!match_found) {
      auto ts = net_message_->add_transforms();
      *ts = *it_in;
    }
  }
  publisher_->Write(net_message_);
}
}  // namespace tf2_cybertron
}  // namespace cybertron
}  // namespace apollo
