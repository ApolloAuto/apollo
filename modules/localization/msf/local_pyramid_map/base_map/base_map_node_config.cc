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

#include "modules/localization/msf/local_map/base_map/base_map_node_config.h"

namespace apollo {
namespace localization {
namespace msf {

BaseMapNodeConfig::BaseMapNodeConfig() {}

BaseMapNodeConfig::~BaseMapNodeConfig() {}

std::shared_ptr<BaseMapNodeConfig> BaseMapNodeConfig::Clone() {
  std::shared_ptr<BaseMapNodeConfig> map_node_config(new BaseMapNodeConfig());
  map_node_config->node_index_ = node_index_;
  map_node_config->map_version_ = map_version_;
  memcpy(map_node_config->body_md5_, body_md5_, sizeof(body_md5_));
  map_node_config->body_size_ = body_size_;
  map_node_config->has_map_version_ = has_map_version_;
  map_node_config->has_body_md5_ = has_body_md5_;

  return map_node_config;
}

unsigned int BaseMapNodeConfig::LoadBinary(const unsigned char *buf) {
  size_t binary_size = 0;

  // map_version
  const uint16_t *us_p = reinterpret_cast<const uint16_t *>(buf);
  if (has_map_version_) {
    binary_size += sizeof(uint16_t);
    map_version_ = static_cast<MapVersion>(*us_p);
    ++us_p;
  }

  // body_md5
  const unsigned char *uc_p = reinterpret_cast<const unsigned char *>(us_p);
  if (has_body_md5_) {
    binary_size += sizeof(body_md5_);
    memcpy(body_md5_, uc_p, sizeof(body_md5_));
    uc_p += MD5LENTH;
  }

  binary_size += sizeof(unsigned int) + sizeof(int) + sizeof(unsigned int) +
                 sizeof(unsigned int) + sizeof(unsigned int);
  // node_index._resolution_id
  const unsigned int *ui_p = reinterpret_cast<const unsigned int *>(us_p);
  node_index_.resolution_id_ = *ui_p;
  ++ui_p;

  // node_index._zone_id
  const int *int_p = reinterpret_cast<const int *>(ui_p);
  node_index_.zone_id_ = *int_p;
  ++int_p;

  // node_index._m
  ui_p = reinterpret_cast<const unsigned int *>(int_p);
  node_index_.m_ = *ui_p;
  ++ui_p;

  // node_index._n
  node_index_.n_ = *ui_p;
  ++ui_p;

  // the body size
  body_size_ = *ui_p;

  return static_cast<unsigned int>(binary_size);
}

unsigned int BaseMapNodeConfig::CreateBinary(unsigned char *buf,
                                             size_t buf_size) const {
  unsigned int target_size = GetBinarySize();

  if (buf_size < target_size) {
    return 0;
  }

  // map_version
  uint16_t *us_p = reinterpret_cast<uint16_t *>(buf);
  if (has_map_version_) {
    *us_p = static_cast<uint16_t>(map_version_);
    ++us_p;
  }

  // body_md5
  unsigned char *uc_p = reinterpret_cast<unsigned char *>(us_p);
  if (has_body_md5_) {
    memcpy(uc_p, body_md5_, sizeof(body_md5_));
    uc_p += MD5LENTH;
  }

  // node_index._resolution_id
  unsigned int *ui_p = reinterpret_cast<unsigned int *>(us_p);
  *ui_p = node_index_.resolution_id_;
  ++ui_p;

  // node_index._zone_id
  int *int_p = reinterpret_cast<int *>(ui_p);
  *int_p = node_index_.zone_id_;
  ++int_p;

  // node_index._m
  ui_p = reinterpret_cast<unsigned int *>(int_p);
  *ui_p = node_index_.m_;
  ++ui_p;

  // node_index._n
  *ui_p = node_index_.n_;
  ++ui_p;

  // the body size
  *ui_p = static_cast<unsigned int>(body_size_);

  return target_size;
}

unsigned int BaseMapNodeConfig::GetBinarySize() const {
  size_t binary_size = 0;

  // map_version
  if (has_map_version_) {
    binary_size += sizeof(uint16_t);
  }

  // body_md5
  if (has_body_md5_) {
    binary_size += sizeof(body_md5_);
  }

  // node_index._resolution_id
  // node_index._zone_id
  // node_index._m
  // node_index._n
  binary_size += sizeof(unsigned int) + sizeof(int) + sizeof(unsigned int) +
                 sizeof(unsigned int);

  // the body size
  binary_size += sizeof(unsigned int);
  return static_cast<unsigned int>(binary_size);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
