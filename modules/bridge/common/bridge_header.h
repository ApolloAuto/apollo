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

typedef uint32_t hsize;

constexpr char BRIDGE_HEADER_FLAG[] = "ApolloBridgeHeader";
constexpr size_t HEADER_FLAG_SIZE = sizeof(BRIDGE_HEADER_FLAG);
constexpr size_t Item_Header_Size = sizeof(HType) + sizeof(bsize) + 2;

class BridgeHeader {
 public:
  BridgeHeader() = default;
  ~BridgeHeader() = default;

 public:
  bool Serialize(char *buf, size_t size);
  bool Diserialize(const char *buf, size_t buf_size);
  bool IsAvailable(const char *buf);

  uint32_t GetHeaderVer() const { return header_ver_.value_; }
  hsize GetHeaderSize() const {
    return static_cast<hsize>(header_body_size_ + HEADER_FLAG_SIZE +
                              sizeof(hsize) + 2);
  }
  bsize GetHeaderBodySize() const { return header_body_size_; }
  std::string GetMsgName() const { return msg_name_.value_; }
  uint32_t GetMsgID() const { return msg_id_.value_; }
  uint32_t GetTotalFrames() const { return total_frames_.value_; }
  uint32_t GetIndex() const { return index_.value_; }
  double GetTimeStamp() const { return time_stamp_.value_; }
  bsize GetMsgSize() const { return msg_size_.value_; }
  bsize GetFrameSize() const { return frame_size_.value_; }
  bsize GetFramePos() const { return frame_pos_.value_; }

  void SetHeaderVer(uint32_t header_ver) {
    header_ver_ = header_ver;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(uint32_t));
  }
  void SetMsgName(const std::string &msg_name) {
    msg_name_ = msg_name;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + msg_name.length() + 1);
  }
  void SetMsgID(uint32_t msg_id) {
    msg_id_ = msg_id;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(uint32_t));
  }
  void SetTotalFrames(uint32_t total_frames) {
    total_frames_ = total_frames;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(uint32_t));
  }
  void SetFrameSize(bsize frame_size) {
    frame_size_ = frame_size;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(bsize));
  }
  void SetFramePos(bsize frame_pos) {
    frame_pos_ = frame_pos;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(bsize));
  }
  void SetIndex(uint32_t index) {
    index_ = index;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(uint32_t));
  }
  void SetTimeStamp(double time_stamp) {
    time_stamp_ = time_stamp;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(double));
  }
  void SetMsgSize(bsize msg_size) {
    msg_size_ = msg_size;
    header_body_size_ +=
        static_cast<hsize>(Item_Header_Size + 1 + sizeof(bsize));
  }

 private:
  template <typename T, size_t S>
  char *SerializeBasicType(const T *value, char *buf, size_t size) {
    if (!buf || size < S) {
      return nullptr;
    }
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
  HeaderItem<Msg_Size, bsize> msg_size_;
  HeaderItem<Msg_Frames, uint32_t> total_frames_;
  HeaderItem<Frame_Size, bsize> frame_size_;
  HeaderItem<Frame_Pos, bsize> frame_pos_;
  HeaderItem<Frame_Index, uint32_t> index_;
  HeaderItem<Time_Stamp, double> time_stamp_;
  hsize header_body_size_ = 0;
  HeaderItemBase *header_item[Header_Tail] = {
      &header_ver_, &msg_name_,  &msg_id_, &msg_size_,   &total_frames_,
      &frame_size_, &frame_pos_, &index_,  &time_stamp_,
  };
};

}  // namespace bridge
}  // namespace apollo
