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

#include "modules/localization/msf/local_map/base_map/base_map_node.h"

#include <cstdio>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

using apollo::common::util::DirectoryExists;
using apollo::common::util::EnsureDirectory;

BaseMapNode::BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy)
    : map_matrix_(matrix), compression_strategy_(strategy) {}

BaseMapNode::~BaseMapNode() {
  if (map_matrix_ != nullptr) {
    delete map_matrix_;
  }
  if (compression_strategy_ != nullptr) {
    delete compression_strategy_;
  }
}

void BaseMapNode::Init(const BaseMapConfig* map_config,
                       const MapNodeIndex& index, bool create_map_cells) {
  map_config_ = map_config;
  index_ = index;
  left_top_corner_ = GetLeftTopCorner(*map_config_, index_);
  is_reserved_ = false;
  data_is_ready_ = false;
  is_changed_ = false;
  if (create_map_cells) {
    InitMapMatrix(map_config_);
  }
  return;
}

void BaseMapNode::InitMapMatrix(const BaseMapConfig* map_config) {
  map_config_ = map_config;
  map_matrix_->Init(map_config);
}

void BaseMapNode::Finalize() {
  if (is_changed_) {
    Save();
    AERROR << "Save Map Node to disk: " << index_ << ".";
  }
}

void BaseMapNode::ResetMapNode() {
  is_changed_ = false;
  data_is_ready_ = false;
  is_reserved_ = false;
  map_matrix_->Reset(map_config_);
}

// void BaseMapNode::SetCompressionStrategy(compression::CompressionStrategy*
// strategy) {
//     compression_strategy_ = strategy;
//     return;
// }

bool BaseMapNode::Save() {
  SaveIntensityImage();
  char buf[1024];
  std::string path = map_config_->map_folder_path_;
  if (!EnsureDirectory(path)) {
    return false;
  }
  path = path + "/map";
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%03u", index_.resolution_id_);
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  if (index_.zone_id_ > 0) {
    path = path + "/north";
  } else {
    path = path + "/south";
  }
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%02d", abs(index_.zone_id_));
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u", index_.m_);
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u", index_.n_);
  path = path + buf;

  AINFO << "Save node: " << path;

  FILE* file = fopen(path.c_str(), "wb");
  if (file) {
    CreateBinary(file);
    fclose(file);
    is_changed_ = false;
    return true;
  } else {
    AERROR << "Can't write to file: " << path << ".";
    return false;
  }
}

bool BaseMapNode::Load() {
  char buf[1024];
  std::string path = map_config_->map_folder_path_;
  if (!DirectoryExists(path)) {
    return false;
  }
  path = path + "/map";
  if (!DirectoryExists(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%03u", index_.resolution_id_);
  path = path + buf;
  if (!DirectoryExists(path)) {
    return false;
  }
  if (index_.zone_id_ > 0) {
    path = path + "/north";
  } else {
    path = path + "/south";
  }
  if (!DirectoryExists(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%02d", abs(index_.zone_id_));
  path = path + buf;
  if (!DirectoryExists(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u", index_.m_);
  path = path + buf;
  if (!DirectoryExists(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u", index_.n_);
  path = path + buf;

  return Load(path.c_str());
}

bool BaseMapNode::Load(const char* filename) {
  data_is_ready_ = false;
  // char buf[1024];

  FILE* file = fopen(filename, "rb");
  if (file) {
    LoadBinary(file);
    fclose(file);
    is_changed_ = false;
    data_is_ready_ = true;
    return true;
  } else {
    AERROR << "Can't find the file: " << filename;
    return false;
  }
}

unsigned int BaseMapNode::LoadBinary(FILE* file) {
  // Load the header
  unsigned int header_size = GetHeaderBinarySize();
  std::vector<unsigned char> buf(header_size);
  size_t read_size = fread(&buf[0], 1, header_size, file);
  CHECK_EQ(read_size, header_size);
  unsigned int processed_size = LoadHeaderBinary(&buf[0]);
  CHECK_EQ(processed_size, header_size);

  // Load the body
  buf.resize(file_body_binary_size_);
  read_size = fread(&buf[0], 1, file_body_binary_size_, file);
  CHECK_EQ(read_size, file_body_binary_size_);
  processed_size += LoadBodyBinary(&buf);
  return processed_size;
}

unsigned int BaseMapNode::CreateBinary(FILE* file) const {
  unsigned int buf_size = GetBinarySize();
  std::vector<unsigned char> buffer;
  buffer.resize(buf_size);

  unsigned int binary_size = 0;
  std::vector<unsigned char> body_buffer;
  CreateBodyBinary(&body_buffer);
  // CHECK_EQ(processed_size, buf_size);

  // Create header
  unsigned int header_size = GetHeaderBinarySize();
  unsigned int processed_size = CreateHeaderBinary(&buffer[0], buf_size);
  CHECK_EQ(processed_size, header_size);

  unsigned int buffer_bias = processed_size;
  buf_size -= processed_size;
  binary_size += processed_size;
  // Create body
  CHECK_GE(buf_size, body_buffer.size());
  memcpy(&buffer[buffer_bias], &body_buffer[0], body_buffer.size());
  binary_size += body_buffer.size();
  fwrite(&buffer[0], 1, binary_size, file);
  return binary_size;
}

unsigned int BaseMapNode::GetBinarySize() const {
  // It is uncompressed binary size.
  return GetBodyBinarySize() + GetHeaderBinarySize();
}

unsigned int BaseMapNode::LoadHeaderBinary(unsigned char* buf) {
  unsigned int target_size = GetHeaderBinarySize();
  unsigned int* p = reinterpret_cast<unsigned int*>(buf);
  index_.resolution_id_ = *p;
  ++p;
  int* pp = reinterpret_cast<int*>(p);
  index_.zone_id_ = *pp;
  ++pp;
  p = reinterpret_cast<unsigned int*>(pp);
  index_.m_ = *p;
  ++p;
  index_.n_ = *p;
  ++p;
  // left_top_corner_ = GetLeftTopCorner(*map_config_, index_);
  left_top_corner_ = GetLeftTopCorner(*map_config_, index_);
  file_body_binary_size_ = *p;
  return target_size;
}

unsigned int BaseMapNode::CreateHeaderBinary(unsigned char* buf,
                                             unsigned int buf_size) const {
  unsigned int target_size = GetHeaderBinarySize();
  if (buf_size >= target_size) {
    unsigned int* p = reinterpret_cast<unsigned int*>(buf);
    *p = index_.resolution_id_;
    ++p;
    int* pp = reinterpret_cast<int*>(p);
    *pp = index_.zone_id_;
    ++pp;
    p = reinterpret_cast<unsigned int*>(pp);
    *p = index_.m_;
    ++p;
    *p = index_.n_;
    ++p;
    *p = file_body_binary_size_;  // Set it before call this function!
  }
  return target_size;
}

unsigned int BaseMapNode::GetHeaderBinarySize() const {
  return sizeof(unsigned int)     // index_.resolution_id_
         + sizeof(int)            // index_.zone_id_
         + sizeof(unsigned int)   // index_.m_
         + sizeof(unsigned int)   // index_.n_
         + sizeof(unsigned int);  // the body size in file.
}

// unsigned int BaseMapNode::CreateBodyBinary(
//         std::vector<unsigned char> &buf) const {
//     // Compute the binary body size
//     unsigned int body_size = GetBodyBinarySize();
//     buf.resize(body_size);
//     return map_matrix_->CreateBinary(&buf[0], body_size);
// }

// unsigned int BaseMapNode::GetBodyBinarySize() const {
//     return map_matrix_->GetBinarySize();
// }

unsigned int BaseMapNode::LoadBodyBinary(std::vector<unsigned char>* buf) {
  if (compression_strategy_ == nullptr) {
    return map_matrix_->LoadBinary(&((*buf)[0]));
  }
  std::vector<unsigned char> buf_uncompressed;
  compression_strategy_->Decode(buf, &buf_uncompressed);
  AERROR << "map node compress ratio: "
         << static_cast<float>(buf->size()) / buf_uncompressed.size();
  return map_matrix_->LoadBinary(&buf_uncompressed[0]);
}

unsigned int BaseMapNode::CreateBodyBinary(
    std::vector<unsigned char>* buf) const {
  if (compression_strategy_ == nullptr) {
    unsigned int body_size = GetBodyBinarySize();
    buf->resize(body_size);
    map_matrix_->CreateBinary(&((*buf)[0]), body_size);
    file_body_binary_size_ = buf->size();
    return buf->size();
  }
  std::vector<unsigned char> buf_uncompressed;
  // Compute the uncompression binary body size
  unsigned int body_size = GetBodyBinarySize();
  buf_uncompressed.resize(body_size);
  map_matrix_->CreateBinary(&buf_uncompressed[0], body_size);
  compression_strategy_->Encode(&buf_uncompressed, buf);
  file_body_binary_size_ = buf->size();
  return buf->size();
}

unsigned int BaseMapNode::GetBodyBinarySize() const {
  return map_matrix_->GetBinarySize();
}

// bool BaseMapNode::GetCoordinate(const idl::car::core::numerical::Vector2D&
// coordinate,
//         unsigned int& x, unsigned int& y) const {
//     const idl::car::core::numerical::Vector2D& left_top_corner =
//     GetLeftTopCorner(); int off_x = static_cast<int>((coordinate[0] -
//     left_top_corner[0])/GetMapResolution()); int off_y =
//     static_cast<int>((coordinate[1] -
//     left_top_corner[1])/GetMapResolution()); if (off_x >= 0 && off_x <
//     this->map_config_->map_node_size_x_ &&
//         off_y >= 0 && off_y < this->map_config_->map_node_size_y_) {
//         x = static_cast<unsigned int>(off_x);
//         y = static_cast<unsigned int>(off_y);
//         return true;
//     }
//     else {
//         return false;
//     }
// }

bool BaseMapNode::GetCoordinate(const Eigen::Vector2d& coordinate,
                                unsigned int* x, unsigned int* y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  int off_x = static_cast<int>((coordinate[0] - left_top_corner[0]) /
                               GetMapResolution());
  int off_y = static_cast<int>((coordinate[1] - left_top_corner[1]) /
                               GetMapResolution());
  if (off_x >= 0 &&
      off_x < static_cast<int>(this->map_config_->map_node_size_x_) &&
      off_y >= 0 &&
      off_y < static_cast<int>(this->map_config_->map_node_size_y_)) {
    *x = static_cast<unsigned int>(off_x);
    *y = static_cast<unsigned int>(off_y);
    return true;
  } else {
    return false;
  }
}

// bool BaseMapNode::GetCoordinate(const idl::car::core::numerical::Vector3D&
// coordinate,
//         unsigned int& x, unsigned int& y) const {
//     idl::car::core::numerical::Vector2D coord2d;
//     coord2d.init(coordinate.get_data());
//     return GetCoordinate(coord2d, x, y);
// }

bool BaseMapNode::GetCoordinate(const Eigen::Vector3d& coordinate,
                                unsigned int* x, unsigned int* y) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  return GetCoordinate(coord2d, x, y);
}

// idl::car::core::numerical::Vector2D BaseMapNode::GetCoordinate(
//         unsigned int x, unsigned int y) const {
//     const idl::car::core::numerical::Vector2D& left_top_corner =
//     GetLeftTopCorner(); idl::car::core::numerical::Vector2D coord;
//     coord[0] = left_top_corner[0] + x * GetMapResolution();
//     coord[1] = left_top_corner[1] + y * GetMapResolution();
//     return coord;
// }

Eigen::Vector2d BaseMapNode::GetCoordinate(unsigned int x,
                                           unsigned int y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord(left_top_corner[0] + x * GetMapResolution(),
                        left_top_corner[1] + y * GetMapResolution());
  return coord;
}

// idl::car::core::numerical::Vector2D BaseMapNode::GetLeftTopCorner(
//     const BaseMapConfig& config, const MapNodeIndex& index) {
//     idl::car::core::numerical::Vector2D coord;
//     coord[0] = config.map_range_.get_min_x() +
//             config.map_node_size_x_*config.map_resolutions_[index.resolution_id_]*index.n_;
//     coord[1] = config.map_range_.get_min_y() +
//             config.map_node_size_y_*config.map_resolutions_[index.resolution_id_]*index.m_;
//     assert(coord[0] < config.map_range_.get_max_x());
//     assert(coord[1] < config.map_range_.get_max_y());
//     return coord;
// }

Eigen::Vector2d BaseMapNode::GetLeftTopCorner(const BaseMapConfig& config,
                                              const MapNodeIndex& index) {
  Eigen::Vector2d coord;
  coord[0] = config.map_range_.GetMinX() +
             config.map_node_size_x_ *
                 config.map_resolutions_[index.resolution_id_] * index.n_;
  coord[1] = config.map_range_.GetMinY() +
             config.map_node_size_y_ *
                 config.map_resolutions_[index.resolution_id_] * index.m_;
  DCHECK_LT(coord[0], config.map_range_.GetMaxX());
  DCHECK_LT(coord[1], config.map_range_.GetMaxY());
  return coord;
}

bool BaseMapNode::SaveIntensityImage() const {
  char buf[1024];
  std::string path = map_config_->map_folder_path_;
  if (!EnsureDirectory(path)) {
    return false;
  }
  path = path + "/image";
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%03u", index_.resolution_id_);
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  if (index_.zone_id_ > 0) {
    path = path + "/north";
  } else {
    path = path + "/south";
  }
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%02d", abs(index_.zone_id_));
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u", index_.m_);
  path = path + buf;
  if (!EnsureDirectory(path)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08u.png", index_.n_);
  path = path + buf;
  bool success0 = SaveIntensityImage(path);
  return success0;
}

bool BaseMapNode::SaveIntensityImage(const std::string& path) const {
  cv::Mat image;
  map_matrix_->GetIntensityImg(&image);
  bool success = cv::imwrite(path, image);
  return success;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
