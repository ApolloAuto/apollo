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
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

namespace apollo {
namespace cyber {
namespace context {

class Context {
 public:
  Context() = default;
  ~Context() = default;

  /**
   * @brief Set value to context.
   *
   * @tparam T type of value
   * @param key string key
   * @param value shared_ptr of T
   */
  template <typename T>
  void Set(const std::string &key, const std::shared_ptr<T> &value);

  /**
   * @brief Set value to context.
   *
   * @tparam T type of value
   * @param key string key
   * @param value T value
   */
  template <typename T>
  void Set(const std::string &key, const T &value);

  /**
   * @brief Get value from context.
   *
   * @tparam T type of value
   * @param key string key
   * @return std::shared_ptr<T> value of key in context or nullptr if not exist
   */
  template <typename T>
  std::shared_ptr<T> Get(const std::string &key) const;

  /**
   * @brief Set value to context safely.
   *
   * @tparam T type of value
   * @param key string key
   * @param value shared_ptr of T
   */
  template <typename T>
  void SafeSet(const std::string &key, const std::shared_ptr<T> &value);

  /**
   * @brief Set value to context safely.
   *
   * @tparam T type of value
   * @param key string key
   * @param value T value
   */
  template <typename T>
  void SafeSet(const std::string &key, const T &value);

  /**
   * @brief Get value from context safely.
   *
   * @tparam T type of value
   * @param key string key
   * @return std::shared_ptr<T> value of key in context or nullptr if not exist
   */
  template <typename T>
  std::shared_ptr<T> SafeGet(const std::string &key) const;

 private:
  std::map<std::string, std::shared_ptr<void>> m_map_;
  std::mutex m_mutex_;
};

template <typename T>
void Context::Set(const std::string &key, const std::shared_ptr<T> &value) {
  m_map_[key] = value;
}

template <typename T>
void Context::Set(const std::string &key, const T &value) {
  m_map_[key] = std::make_shared<T>(value);
}
template <typename T>
std::shared_ptr<T> Context::Get(const std::string &key) const {
  auto it = m_map_.find(key);
  if (it != m_map_.end()) {
    return std::static_pointer_cast<T>(it->second);
  }
  return nullptr;
}

template <typename T>
void Context::SafeSet(const std::string &key, const std::shared_ptr<T> &value) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return Set(key, value);
}

template <typename T>
void Context::SafeSet(const std::string &key, const T &value) {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return Set(key, value);
}

template <typename T>
std::shared_ptr<T> Context::SafeGet(const std::string &key) const {
  std::lock_guard<std::mutex> lock(m_mutex_);
  return Get<T>(key);
}

}  // namespace context
}  // namespace cyber
}  // namespace apollo
