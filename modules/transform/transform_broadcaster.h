
#ifndef CYBERTRON_TF2_CYBERTRON_TRANSFORM_BROADCASTER_H_
#define CYBERTRON_TF2_CYBERTRON_TRANSFORM_BROADCASTER_H_

#include <memory>
#include <vector>

#include "cybertron/cybertron.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace transform {

/** \brief This class provides an easy way to publish coordinate frame transform
 * information.
 * It will handle all the messaging and stuffing of messages.  And the function
 * prototypes lay out all the
 * necessary data needed for each message.  */

class TransformBroadcaster {
 public:
  /** \brief Constructor (needs a cybertron::Node reference) */
  TransformBroadcaster();

  /** \brief Send a StampedTransform
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  //  void sendTransform(const StampedTransform & transform);

  /** \brief Send a vector of StampedTransforms
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  // void sendTransform(const std::vector<StampedTransform> & transforms);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void sendTransform(const apollo::transform::TransformStamped& transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id
   * already.  */
  void sendTransform(
      const std::vector<apollo::transform::TransformStamped>& transforms);

 private:
  std::unique_ptr<cybertron::Node> node_;
  std::shared_ptr<cybertron::Writer<apollo::transform::TransformStampeds>> publisher_;
};
}  // namespace transform
}  // namespace apollo

#endif  // INCLUDE_CYBERTRON_TF2_CYBERTRON_TRANSFORM_BROADCASTER_H_
