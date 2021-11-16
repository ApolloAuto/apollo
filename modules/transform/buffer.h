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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "tf2/buffer_core.h"
#include "tf2/convert.h"

#include "cyber/node/node.h"
#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {

// extend the BufferInterface class and BufferCore class
class Buffer : public BufferInterface, public tf2::BufferCore {
 public:
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
  virtual TransformStamped lookupTransform(
      const std::string& target_frame, const std::string& source_frame,
      const cyber::Time& time, const float timeout_second = 0.01f) const;

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
  virtual TransformStamped lookupTransform(
      const std::string& target_frame, const cyber::Time& target_time,
      const std::string& source_frame, const cyber::Time& source_time,
      const std::string& fixed_frame, const float timeout_second = 0.01f) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param target_time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const std::string& source_frame,
                            const cyber::Time& target_time,
                            const float timeout_second = 0.01f,
                            std::string* errstr = nullptr) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param target_time The time into which to transform
   * \param source_frame The frame from which to transform
   * \param source_time The time from which to transform
   * \param fixed_frame The frame in which to treat the transform as constant in
   * time
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const cyber::Time& target_time,
                            const std::string& source_frame,
                            const cyber::Time& source_time,
                            const std::string& fixed_frame,
                            const float timeout_second = 0.01f,
                            std::string* errstr = nullptr) const;

  bool GetLatestStaticTF(const std::string& frame_id,
                         const std::string& child_frame_id,
                         TransformStamped* tf);

 private:
  void SubscriptionCallback(
      const std::shared_ptr<const TransformStampeds>& transform);
  void StaticSubscriptionCallback(
      const std::shared_ptr<const TransformStampeds>& transform);
  void SubscriptionCallbackImpl(
      const std::shared_ptr<const TransformStampeds>& transform,
      bool is_static);

  void TF2MsgToCyber(const geometry_msgs::TransformStamped& tf2_trans_stamped,
                     TransformStamped& trans_stamped) const;  // NOLINT

  std::unique_ptr<cyber::Node> node_;
  std::shared_ptr<cyber::Reader<TransformStampeds>> message_subscriber_tf_;
  std::shared_ptr<cyber::Reader<TransformStampeds>>
      message_subscriber_tf_static_;

  cyber::Time last_update_;
  std::vector<geometry_msgs::TransformStamped> static_msgs_;

  DECLARE_SINGLETON(Buffer)
};  // class

}  // namespace transform
}  // namespace apollo
