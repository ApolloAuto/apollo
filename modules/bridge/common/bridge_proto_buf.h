/******************************************************************************der
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

#include <vector>
#include <string>
#include <memory>
#include "modules/bridge/common/macro.h"
#include "modules/bridge/common/bridge_header.h"

namespace apollo {
namespace bridge {

constexpr size_t FRAME_SIZE = 1024;

template <typename T>
class BridgeProtoBuf {
 public:
  BridgeProtoBuf() {}
  ~BridgeProtoBuf();

  char *GetFrame(size_t index);
  bool Serialize(const std::shared_ptr<T> &proto, const std::string &msg_name);
  bool Diserialized(std::shared_ptr<T> proto);
  bool IsReadyDiserialize() const { return is_ready_diser; }
  void UpdateStatus(size_t frame_index);
  bool IsTheProto(const BridgeHeader &header);

  bool Initialize(const BridgeHeader &header);
  char *GetBuf(size_t offset) { return proto_buf_ + offset; }

  const char *GetSerializedBuf(size_t index) const {
    return frames_[index].buf_;
  }
  size_t GetSerializedBufCount() const { return frames_.size(); }
  size_t GetSerializedBufSize(size_t index) const {
    return frames_[index].buf_len_;
  }

 private:
  struct Buf {
    char *buf_;
    size_t buf_len_;
  };

 private:
  size_t total_frames_ = 0;
  size_t total_size_ = 0;
  std::vector<Buf> frames_;
  std::string proto_name_ = "";
  uint32_t status = 0;
  char *proto_buf_ = nullptr;
  bool is_ready_diser = false;
  uint32_t sequence_num_ = 0;
};

template <typename T>
BridgeProtoBuf<T>::~BridgeProtoBuf() {
  for (auto frame : frames_) {
    FREE_ARRY(frame.buf_);
  }
  FREE_ARRY(proto_buf_);
}

template <typename T>
bool BridgeProtoBuf<T>::Serialize(const std::shared_ptr<T> &proto,
                                  const std::string &msg_name) {
  size_t msg_len = proto->ByteSize();
  char *tmp = new char[msg_len];
  memset(tmp, 0, sizeof(char) * msg_len);
  if (!proto->SerializeToArray(tmp, static_cast<int>(msg_len))) {
    FREE_ARRY(tmp);
    return false;
  }
  size_t offset = 0;

  BridgeHeader header;
  header.SetHeaderVer(0);
  header.SetMsgName(msg_name);
  header.SetMsgID(proto->header().sequence_num());
  header.SetTimeStamp(proto->header().timestamp_sec());
  header.SetMsgSize(msg_len);

  uint32_t total_frames = static_cast<uint32_t>(msg_len / FRAME_SIZE +
                                                (msg_len % FRAME_SIZE ? 0 : 1));
  uint32_t frame_index = 0;
  header.SetTotalFrames(total_frames);
  header.SetFrameSize(FRAME_SIZE);

  size_t header_size =
      header.GetHeaderSize() + sizeof(BRIDGE_HEADER_FLAG) + sizeof(size_t) + 2;

  while (offset < msg_len) {
    header.SetIndex(frame_index);
    header.SetFramePos(frame_index * FRAME_SIZE);
    Buf buf;
    buf.buf_ = new char[FRAME_SIZE + header_size];
    buf.buf_len_ = FRAME_SIZE + header_size;
    header.Serialize(buf.buf_, buf.buf_len_);
    memcpy(buf.buf_ + buf.buf_len_, tmp + frame_index * FRAME_SIZE, FRAME_SIZE);
    frames_.push_back(buf);
    frame_index++;
  }
  FREE_ARRY(tmp);
  return true;
}

template <typename T>
bool BridgeProtoBuf<T>::Diserialized(std::shared_ptr<T> proto) {
  if (!proto_buf_ || !proto) {
    return false;
  }
  proto->ParseFromArray(proto_buf_, static_cast<int>(total_size_));
  return true;
}

template <typename T>
void BridgeProtoBuf<T>::UpdateStatus(size_t frame_index) {
  status |= (1 << frame_index);
  is_ready_diser = false;
  if (status == ((1 << total_frames_) - 1)) {
    is_ready_diser = true;
  }
}

template <typename T>
bool BridgeProtoBuf<T>::IsTheProto(const BridgeHeader &header) {
  if (strcmp(proto_name_.c_str(), header.GetMsgName().c_str()) == 0 &&
      sequence_num_ == header.GetMsgID()) {
    return true;
  }
  return false;
}

template <typename T>
bool BridgeProtoBuf<T>::Initialize(const BridgeHeader &header) {
  total_size_ = header.GetMsgSize();
  total_frames_ = header.GetTotalFrames();
  if (!proto_buf_) {
    proto_buf_ = new char[total_size_];
  }
  return true;
}

}  // namespace bridge
}  // namespace apollo
