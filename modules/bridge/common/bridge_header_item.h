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

#include <string>

namespace apollo {
namespace bridge {

typedef uint32_t bsize;

enum HType {
  Header_Ver,
  Msg_Name,
  Msg_ID,
  Msg_Size,
  Msg_Frames,
  Frame_Size,
  Frame_Pos,
  Frame_Index,
  Time_Stamp,

  Header_Tail,
};

class HeaderItemBase {
 public:
  HeaderItemBase() = default;
  virtual ~HeaderItemBase() {}

 public:
  virtual char *SerializeItem(char *buf, size_t buf_size) = 0;
  virtual const char *DiserializeItem(const char *buf, const size_t buf_size,
                                      size_t *diserialized_size) = 0;
  virtual HType GetType() const = 0;
};

template <enum HType t, typename T>
struct HeaderItem;

template <enum HType t, typename T>
char *SerializeItemImp(const HeaderItem<t, T> &item, char *buf,
                       size_t buf_size) {
  if (!buf || buf_size == 0 ||
      buf_size < size_t(sizeof(t) + sizeof(bsize) + item.ValueSize() + 3)) {
    return nullptr;
  }
  char *res = buf;

  // item.ValueSize() get the size of T type data,
  // the maximum of which is proto_name
  // when transfer data, bsize can save sizeof(proto_name).
  // The type needs to be kept consistent during serialize and diserialize.
  bsize item_size = static_cast<bsize>(item.ValueSize());

  HType type = t;
  memcpy(res, &type, sizeof(HType));
  res[sizeof(HType)] = ':';
  res = res + sizeof(HType) + 1;

  memcpy(res, &item_size, sizeof(bsize));
  res[sizeof(bsize)] = ':';
  res = res + sizeof(bsize) + 1;

  memcpy(res, item.GetValuePtr(), item.ValueSize());
  res[item.ValueSize()] = '\n';
  res += item.ValueSize() + 1;
  return res;
}

template <enum HType t, typename T>
const char *DiserializeItemImp(HeaderItem<t, T> *item, const char *buf,
                               const size_t buf_size,
                               size_t *diserialized_size) {
  if (!buf || !diserialized_size ||
      buf_size < size_t(sizeof(HType) + sizeof(bsize) + 2)) {
    return nullptr;
  }
  const char *res = buf;

  char p_type[sizeof(HType)] = {0};
  memcpy(p_type, buf, sizeof(HType));
  HType type = *(reinterpret_cast<HType *>(p_type));
  if (type != t) {
    return nullptr;
  }
  res += sizeof(HType) + 1;
  *diserialized_size += sizeof(HType) + 1;

  char p_size[sizeof(bsize)] = {0};
  memcpy(p_size, res, sizeof(bsize));
  bsize size = *(reinterpret_cast<bsize *>(p_size));
  res += sizeof(bsize) + 1;
  *diserialized_size += sizeof(bsize) + 1;

  if (buf_size < size_t(sizeof(HType) + sizeof(bsize) + size + 3)) {
    return nullptr;
  }
  item->SetValue(res);
  res += size + 1;
  *diserialized_size += size + 1;
  return res;
}

template <enum HType t, typename T>
struct HeaderItem : public HeaderItemBase {
  T value_;

  operator T() { return value_; }
  HeaderItem &operator=(const T &val) {
    value_ = val;
    return *this;
  }
  HType GetType() const override { return t; }
  size_t ValueSize() const { return sizeof(value_); }
  const T *GetValuePtr() const { return &value_; }
  void SetValue(const char *buf) {
    if (!buf) {
      return;
    }
    value_ = *(reinterpret_cast<const T *>(buf));
  }

  char *SerializeItem(char *buf, size_t buf_size) override {
    return SerializeItemImp(*this, buf, buf_size);
  }

  const char *DiserializeItem(const char *buf, size_t buf_size,
                              size_t *diserialized_size) override {
    return DiserializeItemImp(this, buf, buf_size, diserialized_size);
  }
};

template <enum HType t>
struct HeaderItem<t, std::string> : public HeaderItemBase {
  std::string value_;
  operator std::string() { return value_; }
  HeaderItem &operator=(const std::string &val) {
    value_ = val;
    return *this;
  }
  size_t ValueSize() const { return value_.length() + 1; }
  HType GetType() const override { return t; }
  const char *GetValuePtr() const { return value_.c_str(); }
  void SetValue(const char *buf) {
    if (!buf) {
      return;
    }
    value_ = std::string(buf);
  }

  char *SerializeItem(char *buf, size_t buf_size) override {
    return SerializeItemImp(*this, buf, buf_size);
  }

  const char *DiserializeItem(const char *buf, size_t buf_size,
                              size_t *diserialized_size) override {
    return DiserializeItemImp(this, buf, buf_size, diserialized_size);
  }
};

}  // namespace bridge
}  // namespace apollo
