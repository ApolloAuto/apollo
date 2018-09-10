
#ifndef CYBERTRON_TF2_CYBERTRON_TRANSFORM_LISTENER_H_
#define CYBERTRON_TF2_CYBERTRON_TRANSFORM_LISTENER_H_

#include <iostream>
#include <unistd.h>
#include <vector>

#include <geometry_msgs/transform_stamped.h>
#include <tf2/time.h>
#include "cybertron/tf2_cybertron/buffer.h"

namespace apollo {
namespace cybertron {
namespace tf2_cybertron {

class TransformListener {
 public:
  /**@brief Constructor for transform listener */
  TransformListener(tf2::BufferCore& buffer, bool spin_thread = true);
  // TransformListener(tf2::BufferCore& buffer, const cybertron::NodeHandle& nh,
  // bool
  // spin_thread = true);

  ~TransformListener();

 private:
  /// Initialize this transform listener, subscribing, advertising services,
  /// etc.
  void Init();
  void initWithThread();

  /// Callback function for cybertron message subscriptoin
  void SubscriptionCallback(
      const std::shared_ptr<adu::common::TransformStampeds>& transform);
  void StaticSubscriptionCallback(
      const std::shared_ptr<adu::common::TransformStampeds>& transform);
  void SubscriptionCallbackImpl(
      const std::shared_ptr<adu::common::TransformStampeds>& transform,
      bool is_static);

  // cybertron::CallbackQueue tf_message_callback_queue_;
  // cybertron::NodeHandle node_;
  // cybertron::Subscriber message_subscriber_tf_;
  // cybertron::Subscriber message_subscriber_tf_static_;
  std::shared_ptr<cybertron::Node> node_;
  std::shared_ptr<cybertron::Reader<adu::common::TransformStampeds>>
      message_subscriber_tf_;
  std::shared_ptr<cybertron::Reader<adu::common::TransformStampeds>> message_subscriber_tf_static_;

  tf2::BufferCore& buffer_;
  cybertron::Time last_update_;

  std::vector<geometry_msgs::TransformStamped> static_msgs_;
};
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_CYBERTRON_TRANSFORM_LISTENER_H_
