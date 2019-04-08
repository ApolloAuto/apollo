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

#include <algorithm>
#include <string>

#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace transform {

// extend the TFCore class and the TFCpp class
class BufferInterface {
 public:
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
      const cyber::Time& time, const float timeout_second = 0.01f) const = 0;

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
      const std::string& fixed_frame,
      const float timeout_second = 0.01f) const = 0;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the
   * transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  virtual bool canTransform(const std::string& target_frame,
                            const std::string& source_frame,
                            const cyber::Time& time,
                            const float timeout_second = 0.01f,
                            std::string* errstr = nullptr) const = 0;

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
                            std::string* errstr = nullptr) const = 0;

  // Transform, simple api, with pre-allocation
  template <typename T>
  T& transform(const T& in, T& out, const std::string& target_frame,  // NOLINT
               float timeout = 0.0f) const {
    // do the transform
    tf2::doTransform(in, out,
                     lookupTransform(target_frame, tf2::getFrameId(in),
                                     tf2::getTimestamp(in), timeout));
    return out;
  }

  // transform, simple api, no pre-allocation
  template <typename T>
  T transform(const T& in, const std::string& target_frame,
              float timeout = 0.0f) const {
    T out;
    return transform(in, out, target_frame, timeout);
  }

  // transform, simple api, different types, pre-allocation
  template <typename A, typename B>
  B& transform(const A& in, B& out, const std::string& target_frame,  // NOLINT
               float timeout = 0.0f) const {
    A copy = transform(in, target_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }

  // Transform, advanced api, with pre-allocation
  template <typename T>
  T& transform(const T& in, T& out, const std::string& target_frame,  // NOLINT
               const cyber::Time& target_time, const std::string& fixed_frame,
               float timeout = 0.0f) const {
    // do the transform
    tf2::doTransform(
        in, out,
        lookupTransform(target_frame, target_time, tf2::getFrameId(in),
                        tf2::getTimestamp(in), fixed_frame, timeout));
    return out;
  }

  // transform, advanced api, no pre-allocation
  template <typename T>
  T transform(const T& in, const std::string& target_frame,
              const cyber::Time& target_time, const std::string& fixed_frame,
              float timeout = 0.0f) const {
    T out;
    return transform(in, out, target_frame, target_time, fixed_frame, timeout);
  }

  // Transform, advanced api, different types, with pre-allocation
  template <typename A, typename B>
  B& transform(const A& in, B& out, const std::string& target_frame,  // NOLINT
               const cyber::Time& target_time, const std::string& fixed_frame,
               float timeout = 0.0f) const {
    // do the transform
    A copy = transform(in, target_frame, target_time, fixed_frame, timeout);
    tf2::convert(copy, out);
    return out;
  }
};  // class

}  // namespace transform
}  // namespace apollo
