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

#include <memory>
#include <string>
#include <vector>

#include "modules/bridge/common/bridge_header.h"
#include "modules/bridge/common/macro.h"

namespace apollo {
namespace bridge {

template <typename T>
class BridgeProtoSerializedBuf {
 public:
  BridgeProtoSerializedBuf() {}
  ~BridgeProtoSerializedBuf();

  char *GetFrame(size_t index);
  bool Serialize(const std::shared_ptr<T> &proto, const std::string &msg_name);

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
  std::vector<Buf> frames_;
};

template <typename T>
BridgeProtoSerializedBuf<T>::~BridgeProtoSerializedBuf() {
  for (auto frame : frames_) {
    FREE_ARRY(frame.buf_);
  }
}

template <typename T>
bool BridgeProtoSerializedBuf<T>::Serialize(const std::shared_ptr<T> &proto,
                                            const std::string &msg_name) {
  bsize msg_len = static_cast<bsize>(proto->ByteSizeLong());
  char *tmp = new char[msg_len]();
  if (!proto->SerializeToArray(tmp, static_cast<int>(msg_len))) {
    FREE_ARRY(tmp);
    return false;
  }
  bsize offset = 0;
  bsize frame_index = 0;
  uint32_t total_frames = static_cast<uint32_t>(msg_len / FRAME_SIZE +
                                                (msg_len % FRAME_SIZE ? 1 : 0));

  while (offset < msg_len) {
    bsize left = msg_len - frame_index * FRAME_SIZE;
    bsize cpy_size = (left > FRAME_SIZE) ? FRAME_SIZE : left;

    BridgeHeader header;
    header.SetHeaderVer(0);
    header.SetMsgName(msg_name);
    header.SetMsgID(proto->header().sequence_num());
    header.SetTimeStamp(proto->header().timestamp_sec());
    header.SetMsgSize(msg_len);
    header.SetTotalFrames(total_frames);
    header.SetFrameSize(cpy_size);
    header.SetIndex(frame_index);
    header.SetFramePos(frame_index * FRAME_SIZE);
    hsize header_size = header.GetHeaderSize();
    Buf buf;
    buf.buf_ = new char[cpy_size + header_size];
    buf.buf_len_ = cpy_size + header_size;
    header.Serialize(buf.buf_, buf.buf_len_);
    memcpy(buf.buf_ + header_size, tmp + frame_index * FRAME_SIZE, cpy_size);
    frames_.push_back(buf);
    frame_index++;
    offset += cpy_size;
  }
  FREE_ARRY(tmp);
  return true;
}

}  // namespace bridge
}  // namespace apollo
