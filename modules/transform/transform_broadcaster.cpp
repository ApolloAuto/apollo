#include "cybertron/transform/transform_broadcaster.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace transform {

TransformBroadcaster::TransformBroadcaster() {
  node_ = cybertron::CreateNode(
      "transform_broadcast_" +
      std::to_string(cybertron::Time::Now().ToNanosecond()));
  if (node_ != nullptr) {
    proto::RoleAttributes attr;
    attr.set_channel_name("/tf");
    publisher_ = node_->CreateWriter<::apollo::transform::TransformStampeds>(attr);
  }
}

void TransformBroadcaster::sendTransform(
    const ::apollo::transform::TransformStamped& msgtf) {
  std::vector<::apollo::transform::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}

void TransformBroadcaster::sendTransform(
    const std::vector<::apollo::transform::TransformStamped>& msgtf) {
  std::shared_ptr<::apollo::transform::TransformStampeds> message(
      new ::apollo::transform::TransformStampeds);
  for (std::vector<::apollo::transform::TransformStamped>::const_iterator it =
           msgtf.begin();
       it != msgtf.end(); ++it) {
    auto transform = message->add_transforms();
    *transform = *it;
  }
  publisher_->Write(message);
}

}  // namespace transform
}  // namespace apollo
