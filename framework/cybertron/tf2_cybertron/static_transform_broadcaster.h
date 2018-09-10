
#ifndef CYBERTRON_TF2_CYBERTRON_STATIC_TRANSFORM_BROADCASTER_H_
#define CYBERTRON_TF2_CYBERTRON_STATIC_TRANSFORM_BROADCASTER_H_

#include <memory>

#include "cybertron/cybertron.h"
#include "cybertron/node/node.h"
#include "cybertron/time/time.h"

#include "cybertron/proto/common_geometry.pb.h"

namespace apollo{
namespace cybertron {
namespace tf2_cybertron {

/** \brief This class provides an easy way to publish coordinate frame transform
 * information.
 * It will handle all the messaging and stuffing of messages.  And the function
 * prototypes lay out all the
 * necessary data needed for each message.  */

class StaticTransformBroadcaster {
 public:
  /** \brief Constructor (needs a cybertron::Node reference) */
  StaticTransformBroadcaster(const std::shared_ptr<cybertron::Node>& node);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void sendTransform(const adu::common::TransformStamped& transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void sendTransform(
      const std::vector<adu::common::TransformStamped>& transforms);

 private:
  std::shared_ptr<cybertron::Node> node_;
  std::shared_ptr<cybertron::Writer<adu::common::TransformStampeds>> publisher_;
  std::shared_ptr<adu::common::TransformStampeds> net_message_;
};
}
}
}

#endif  // INCLUDE_CYBERTRON_TF2_CYBERTRON_STATIC_TRANSFORM_BROADCASTER_H_
