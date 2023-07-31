/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cyber/cyber.h"

namespace apollo {
namespace perception {

class MsgConverter {
 public:
  template <class U, class V>
  using Callback = bool(*)(const std::shared_ptr<U> &, V *);

 public:
  MsgConverter() = default;
  explicit MsgConverter(const std::shared_ptr<cyber::Node> &node)
      : node_(node) {}

  virtual ~MsgConverter() = default;

  template <class From, class To>
  bool Add(const std::string &from_topic, const std::string &to_topic,
           Callback<From, To> convert);

 private:
  std::vector<std::shared_ptr<cyber::ReaderBase>> readers_;
  std::vector<std::shared_ptr<cyber::WriterBase>> writers_;

  std::shared_ptr<cyber::Node> node_;
};

template <class From, class To>
bool MsgConverter::Add(const std::string &from_topic,
                       const std::string &to_topic,
                       Callback<From, To> convert) {
  AINFO << "Convert: " << from_topic << " to " << to_topic;
  auto writer = node_->CreateWriter<To>(to_topic);
  auto reader = node_->CreateReader<From>(
      from_topic, [=](const std::shared_ptr<From> &from) {
        std::shared_ptr<To> to(new To());
        bool res = convert(from, to.get());
        if (res) {
          writer->Write(to);
        }
      });

  writers_.push_back(writer);
  readers_.push_back(reader);
  return true;
}

}  // namespace perception
}  // namespace apollo
