/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <cstring>
#include <string>
#include "modules/bridge/common/bridge_header_item.h"

namespace apollo {
namespace bridge {

const char BRIDGE_HEADER_FLAG[] = "ApolloBridgeHeader";

class BridgeHeader {
 public:
  BridgeHeader() = default;
  ~BridgeHeader() = default;

 public:
  bool Serialize(char *buf, size_t size);
  bool Diserialize(const char *buf);
  bool IsAvailable(const char *buf);

  uint32_t GetHeaderVer() const { return header_ver_.value_; }
  size_t GetHeaderSize() const { return header_size_; }
  std::string GetMsgName() const { return msg_name_.value_; }
  uint32_t GetMsgID() const { return msg_id_.value_; }
  uint32_t GetTotalFrames() const { return total_frames_.value_; }
  uint32_t GetIndex() const { return index_.value_; }
  double GetTimeStamp() const { return time_stamp_.value_; }
  size_t GetMsgSize() const { return msg_size_.value_; }
  size_t GetFrameSize() const { return frame_size_.value_; }
  size_t GetFramePos() const { return frame_pos_.value_; }

  void SetHeaderVer(uint32_t header_ver) { header_ver_ = header_ver; }
  void SetMsgName(const std::string &msg_name) { msg_name_ = msg_name; }
  void SetMsgID(uint32_t msg_id) { msg_id_ = msg_id; }
  void SetTotalFrames(uint32_t total_frames) { total_frames_ = total_frames; }
  void SetFrameSize(size_t frame_size) { frame_size_ = frame_size; }
  void SetFramePos(size_t frame_pos) { frame_pos_ = frame_pos; }
  void SetIndex(uint32_t index) { index_ = index; }
  void SetTimeStamp(double time_stamp) { time_stamp_ = time_stamp; }
  void SetMsgSize(size_t msg_size) { msg_size_ = msg_size; }

 private:
  template <typename T, size_t S>
  char *SerializeBasicType(const T *value, char *buf, size_t size) {
    if (!buf || size < S) {
      return nullptr;
    }
    header_size_ += S + 1;
    char *res = buf;
    memcpy(res, value, S);
    res[S] = '\n';
    res += S + 1;
    return res;
  }

  template <typename T, size_t S>
  bool DiserializeBasicType(T *value, const char *buf) {
    if (!buf) {
      return false;
    }
    char temp[S] = {0};
    memcpy(temp, buf, S);
    *value = *(reinterpret_cast<T *>(temp));
    return true;
  }

  char *SerializeHeaderFlag(char *buf, size_t size);
  char *SerializeHeaderSize(char *buf, size_t size);

 private:
  HeaderItem<Header_Ver, uint32_t> header_ver_;
  HeaderItem<Msg_Name, std::string> msg_name_;
  HeaderItem<Msg_ID, uint32_t> msg_id_;
  HeaderItem<Msg_Size, size_t> msg_size_;
  HeaderItem<Msg_Frames, uint32_t> total_frames_;
  HeaderItem<Frame_Size, size_t> frame_size_;
  HeaderItem<Frame_Pos, size_t> frame_pos_;
  HeaderItem<Frame_Index, uint32_t> index_;
  HeaderItem<Time_Stamp, double> time_stamp_;
  size_t header_size_ = 0;
  HeaderItemBase *header_item[Header_Tail] = {
    &header_ver_,
    &msg_name_,
    &msg_id_,
    &msg_size_,
    &total_frames_,
    &frame_size_,
    &frame_pos_,
    &index_,
    &time_stamp_,
  };
};

}  // namespace bridge
}  // namespace apollo
