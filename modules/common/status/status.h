/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once

#include <string>

#include "google/protobuf/descriptor.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common/util/future.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class Status
 *
 * @brief A general class to denote the return status of an API call. It
 * can either be an OK status for success, or a failure status with error
 * message and error code enum.
 */
class Status {
 public:
  /**
   * @brief Create a status with the specified error code and msg as a
   * human-readable string containing more detailed information.
   * @param code the error code.
   * @param msg the message associated with the error.
   */
  explicit Status(ErrorCode code = ErrorCode::OK, std::string_view msg = "")
      : code_(code), msg_(msg.data()) {}

  ~Status() = default;

  /**
   * @brief generate a success status.
   * @returns a success status
   */
  static Status OK() { return Status(); }

  /**
   * @brief check whether the return status is OK.
   * @returns true if the code is ErrorCode::OK
   *          false otherwise
   */
  bool ok() const { return code_ == ErrorCode::OK; }

  /**
   * @brief get the error code
   * @returns the error code
   */
  ErrorCode code() const { return code_; }

  /**
   * @brief defines the logic of testing if two Status are equal
   */
  bool operator==(const Status &rh) const {
    return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
  }

  /**
   * @brief defines the logic of testing if two Status are unequal
   */
  bool operator!=(const Status &rh) const { return !(*this == rh); }

  /**
   * @brief returns the error message of the status, empty if the status is OK.
   * @returns the error message
   */
  const std::string &error_message() const { return msg_; }

  /**
   * @brief returns a string representation in a readable format.
   * @returns the string "OK" if success.
   *          the internal error message otherwise.
   */
  std::string ToString() const {
    if (ok()) {
      return "OK";
    }
    return ErrorCode_Name(code_) + ": " + msg_;
  }

  /**
   * @brief save the error_code and error message to protobuf
   * @param the Status protobuf that will store the message.
   */
  void Save(StatusPb *status_pb) {
    if (!status_pb) {
      return;
    }
    status_pb->set_error_code(code_);
    if (!msg_.empty()) {
      status_pb->set_msg(msg_);
    }
  }

 private:
  ErrorCode code_;
  std::string msg_;
};

inline std::ostream &operator<<(std::ostream &os, const Status &s) {
  os << s.ToString();
  return os;
}

}  // namespace common
}  // namespace apollo
