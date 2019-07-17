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

#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

#define MD5LENTH 16

/**@brief The map node config info. */
class BaseMapNodeConfig {
 public:
  BaseMapNodeConfig();
  virtual ~BaseMapNodeConfig();

  /**@brief Alloc a new map node config.*/
  // virtual BaseMapNodeConfig* alloc_new_map_node_config();
  /**@brief Clone a new map node config.*/
  virtual std::shared_ptr<BaseMapNodeConfig> Clone();
  /**@brief Load the map node config from a binary chunk.
   * @param <return> The size read (the real size of config).
   */
  virtual unsigned int LoadBinary(const unsigned char *buf);
  /**@brief Create the binary map node config.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size.
   */
  virtual unsigned int CreateBinary(unsigned char *buf, size_t buf_size) const;
  /**@brief Get the size of the config in bytes. */
  virtual unsigned int GetBinarySize() const;

 public:
  MapNodeIndex node_index_;
  MapVersion map_version_ = MapVersion::UNKNOWN;
  unsigned char body_md5_[MD5LENTH] = {0};
  size_t body_size_ = 0;
  bool has_map_version_ = true;
  bool has_body_md5_ = true;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
