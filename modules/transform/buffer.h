/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBERTRON_TF2_CYBERTRON_BUFFER_H_
#define CYBERTRON_TF2_CYBERTRON_BUFFER_H_

#include <vector>

#include "tf2/transform_stamped.h"
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/time.h>

#include "cybertron/node/node.h"
#include "modules/transform/buffer_interface.h"
#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace cybertron {
namespace transform {

// extend the BufferInterface class and BufferCore class
class Buffer : public BufferInterface, public tf2::BufferCore {
 public:
  using tf2::BufferCore::canTransform;
  using tf2::BufferCore::lookupTransform;

  /**
   * @brief  Constructor for a Buffer object
   * @param cache_time How long to keep a history of transforms
   * @param debug Whether to advertise the view_frames service that exposes
   * debugging information from the buffer
   * @return
   */
  int Init();

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0
   *will get the latest)
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual apollo::transform::TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const cybertron::Time& time, const float timeout_second = 0.01) const;

  /** \brief Get the transform between two frames by frame ID assuming fixed
   *frame.
   * \param target_frame The frame to which data should be transformed
   * \param target_time The time to which the data should be transformed. (0
   *will get the latest)
   * \param source_frame The frame where the data originated
   * \param source_time The time at which the source_frame should be evaluated.
   *(0 will get the latest)
   * \param fixed_frame The frame in which to assume the transform is constant
   *in time.
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  virtual apollo::transform::TransformStamped lookupTransform(
      const std::string& target_frame, const cybertron::Time& target_time,
      const std::string& source_frame, const cybertron::Time& source_time,
      const std::string& fixed_frame, const float timeout_second = 0.01) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param target_time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const std::string& source_frame,
                            const cybertron::Time& target_time,
                            const float timeout_second = 0.01,
                            std::string* errstr = NULL) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in
   * time
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not NULL
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const cybertron::Time& target_time,
                            const std::string& source_frame,
                            const cybertron::Time& source_time,
                            const std::string& fixed_frame,
                            const float timeout_second = 0.01,
                            std::string* errstr = NULL) const;

 private:
  void SubscriptionCallback(
      const std::shared_ptr<const apollo::transform::TransformStampeds>&
          transform);
  void StaticSubscriptionCallback(
      const std::shared_ptr<const apollo::transform::TransformStampeds>&
          transform);
  void SubscriptionCallbackImpl(
      const std::shared_ptr<const apollo::transform::TransformStampeds>& transform,
      bool is_static);

  void tf2MsgToCyber(const geometry_msgs::TransformStamped& tf2_trans_stamped,
                     apollo::transform::TransformStamped& trans_stamped) const;

  std::unique_ptr<cybertron::Node> node_;
  std::shared_ptr<cybertron::Reader<apollo::transform::TransformStampeds>>
      message_subscriber_tf_;
  std::shared_ptr<cybertron::Reader<apollo::transform::TransformStampeds>>
      message_subscriber_tf_static_;

  cybertron::Time last_update_;
  std::vector<geometry_msgs::TransformStamped> static_msgs_;
  DECLARE_SINGLETON(Buffer)
};  // class

static const std::string threading_error =
    "Do not call canTransform or lookupTransform with a timeout unless you are "
    "using another thread for populating data. Without a dedicated thread it "
    "will always timeout.  If you have a seperate thread servicing tf "
    "messages, call setUsingDedicatedThread(true) on your Buffer instance.";

}  // namespace transform
}  // namespace cybertron
}  // namespace apollo

#endif  // INCLUDE_CYBERTRON_TF2_CYBERTRON_BUFFER_H_
