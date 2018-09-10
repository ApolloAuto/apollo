#include "cybertron/time/time.h"
#include "cybertron/tf2_cybertron/transform_broadcaster.h"

namespace apollo {
namespace cybertron {
namespace tf2_cybertron {

TransformBroadcaster::TransformBroadcaster() {
  node_ = cybertron::CreateNode(
      "transform_broadcast_" +
      std::to_string(cybertron::Time::Now().ToNanosecond()));
  if (node_ != nullptr) {
    proto::RoleAttributes attr;
    attr.set_channel_name("/tf");
    publisher_ = node_->CreateWriter<::adu::common::TransformStampeds>(attr);
  }
};

void TransformBroadcaster::sendTransform(
    const ::adu::common::TransformStamped& msgtf) {
  std::vector<::adu::common::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}

void TransformBroadcaster::sendTransform(
    const std::vector<::adu::common::TransformStamped>& msgtf) {
  std::shared_ptr<::adu::common::TransformStampeds> message(
      new ::adu::common::TransformStampeds);
  for (std::vector<::adu::common::TransformStamped>::const_iterator it =
           msgtf.begin();
       it != msgtf.end(); ++it) {
    auto transform = message->add_transforms();
    *transform = *it;
  }
  publisher_->Write(message);
}

}
}
}
