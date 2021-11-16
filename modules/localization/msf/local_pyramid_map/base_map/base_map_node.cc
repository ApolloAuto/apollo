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

#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node.h"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "modules/localization/msf/common/util/file_utility.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

BaseMapNode::BaseMapNode()
    : map_config_(NULL),
      map_matrix_(NULL),
      map_matrix_handler_(NULL),
      compression_strategy_(NULL) {
  is_changed_ = false;
  data_is_ready_ = false;
  is_reserved_ = false;
  file_body_binary_size_ = 0;
  uncompressed_file_body_size_ = 0;
}

BaseMapNode::BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy)
    : map_config_(NULL),
      map_matrix_(matrix),
      map_matrix_handler_(NULL),
      compression_strategy_(strategy) {
  is_changed_ = false;
  data_is_ready_ = false;
  is_reserved_ = false;
  file_body_binary_size_ = 0;
}

BaseMapNode::~BaseMapNode() {}

void BaseMapNode::InitMapMatrix(const BaseMapConfig* map_config) {
  map_config_ = map_config;
  map_matrix_->Init(*map_config);
}

void BaseMapNode::Finalize() {
  if (is_changed_) {
    Save();
    AINFO << "Save Map Node to disk: " << map_node_config_->node_index_ << ".";
  }
}

void BaseMapNode::ResetMapNode() {
  is_changed_ = false;
  data_is_ready_ = false;
  is_reserved_ = false;
  file_body_binary_size_ = 0;
  uncompressed_file_body_size_ = 0;
  map_matrix_->Reset();
}

bool BaseMapNode::Save() {
  SaveIntensityImage();
  SaveAltitudeImage();

  std::string path = map_config_->map_folder_path_;

  char buf[1024];
  std::vector<std::string> paths;

  paths.push_back(path);

  snprintf(buf, sizeof(buf), "/map");
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%03u",
           map_node_config_->node_index_.resolution_id_);
  paths.push_back(buf);
  path = path + buf;

  paths.push_back(map_node_config_->node_index_.zone_id_ > 0 ? "/north"
                                                             : "/south");
  path = path + paths.back();

  snprintf(buf, sizeof(buf), "/%02d",
           abs(map_node_config_->node_index_.zone_id_));
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.m_);
  paths.push_back(buf);
  path = path + buf;

  if (!CreateMapDirectoryRecursively(paths)) {
    return false;
  }

  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.n_);
  path = path + buf;

  FILE* file = fopen(path.c_str(), "wb");
  if (file) {
    bool success = CreateBinary(file);
    fclose(file);
    is_changed_ = false;
    return success;
  } else {
    AERROR << "Can't write to file: " << path << ".";
    return false;
  }
}

bool BaseMapNode::Load() {
  std::string path = map_config_->map_folder_path_;
  char buf[1024];
  std::vector<std::string> paths;
  paths.push_back(path);
  snprintf(buf, sizeof(buf), "/map");
  paths.push_back(buf);
  path = path + buf;
  snprintf(buf, sizeof(buf), "/%03u",
           map_node_config_->node_index_.resolution_id_);
  paths.push_back(buf);
  path = path + buf;
  paths.push_back(map_node_config_->node_index_.zone_id_ > 0 ? "/north"
                                                             : "/south");
  path = path + paths.back();
  snprintf(buf, sizeof(buf), "/%02d",
           abs(map_node_config_->node_index_.zone_id_));
  paths.push_back(buf);
  path = path + buf;
  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.m_);
  paths.push_back(buf);
  path = path + buf;
  if (!CheckMapDirectoryRecursively(paths)) {
    return false;
  }
  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.n_);
  path = path + buf;
  return Load(path.c_str());
}

bool BaseMapNode::Load(const char* filename) {
  data_is_ready_ = false;

  FILE* file = fopen(filename, "rb");
  if (file) {
    bool success = LoadBinary(file);
    fclose(file);
    is_changed_ = false;
    data_is_ready_ = success;
    return success;
  } else {
    AERROR << "Can't find the file: " << filename;
    return false;
  }
}

bool BaseMapNode::LoadBinary(FILE* file) {
  // Load the header
  size_t header_size = GetHeaderBinarySize();
  std::vector<unsigned char> buf(header_size);
  size_t read_size = fread(&buf[0], 1, header_size, file);
  if (read_size != header_size) {
    return false;
  }
  size_t processed_size = LoadHeaderBinary(&buf[0]);
  if (processed_size != header_size) {
    return false;
  }
  // Load the body
  buf.resize(file_body_binary_size_);
  read_size = fread(&buf[0], 1, file_body_binary_size_, file);
  if (read_size != file_body_binary_size_) {
    return false;
  }
  processed_size = LoadBodyBinary(&buf);
  if (processed_size != uncompressed_file_body_size_) {
    return false;
  }
  return true;
}

bool BaseMapNode::CreateBinary(FILE* file) const {
  size_t buf_size = GetBinarySize();
  std::vector<unsigned char> buffer;
  buffer.resize(buf_size);

  size_t binary_size = 0;
  std::vector<unsigned char> body_buffer;
  size_t processed_size = CreateBodyBinary(&body_buffer);

  if (map_node_config_->has_body_md5_) {
    FileUtility::ComputeBinaryMd5(&body_buffer[0], body_buffer.size(),
                                  map_node_config_->body_md5_);
  }

  if (processed_size == 0) {
    return false;
  }

  // Create header
  size_t header_size = GetHeaderBinarySize();
  std::vector<unsigned char> header_buf(header_size);
  processed_size = CreateHeaderBinary(&buffer[0], buf_size);

  if (header_size != processed_size) {
    return false;
  }

  size_t buffer_bias = processed_size;
  buf_size -= processed_size;
  binary_size += processed_size;
  // Create body
  memcpy(&buffer[buffer_bias], &body_buffer[0], body_buffer.size());
  // write binary
  binary_size += static_cast<unsigned int>(body_buffer.size());
  fwrite(&buffer[0], 1, binary_size, file);

  return true;
}

size_t BaseMapNode::GetBinarySize() const {
  // It is uncompressed binary size.
  return GetBodyBinarySize() + GetHeaderBinarySize();
}

size_t BaseMapNode::LoadHeaderBinary(const unsigned char* buf) {
  std::shared_ptr<BaseMapNodeConfig> node_config_tem =
      map_node_config_->Clone();

  size_t target_size = map_node_config_->LoadBinary(buf);

  // check if header is valid
  if (node_config_tem->map_version_ != map_node_config_->map_version_ ||
      node_config_tem->node_index_ != map_node_config_->node_index_) {
    return 0;
  }

  file_body_binary_size_ = map_node_config_->body_size_;

  return target_size;
}

size_t BaseMapNode::CreateHeaderBinary(unsigned char* buf,
                                       size_t buf_size) const {
  map_node_config_->body_size_ = file_body_binary_size_;
  return map_node_config_->CreateBinary(buf, buf_size);
}

size_t BaseMapNode::GetHeaderBinarySize() const {
  return map_node_config_->GetBinarySize();
}

size_t BaseMapNode::LoadBodyBinary(std::vector<unsigned char>* buf) {
  if (compression_strategy_ == nullptr) {
    return map_matrix_handler_->LoadBinary(&(*buf)[0], map_matrix_);
  }
  std::vector<unsigned char> buf_uncompressed;
  int ret = compression_strategy_->Decode(buf, &buf_uncompressed);
  if (ret < 0) {
    AERROR << "compression Decode error: " << ret;
    return 0;
  }
  uncompressed_file_body_size_ = buf_uncompressed.size();
  AINFO << "map node compress ratio: "
        << static_cast<float>(buf->size()) /
               static_cast<float>(uncompressed_file_body_size_);

  return map_matrix_handler_->LoadBinary(&buf_uncompressed[0], map_matrix_);
}

size_t BaseMapNode::CreateBodyBinary(std::vector<unsigned char>* buf) const {
  if (compression_strategy_ == nullptr) {
    size_t body_size = GetBodyBinarySize();
    buf->resize(body_size);
    file_body_binary_size_ =
        map_matrix_handler_->CreateBinary(map_matrix_, &(*buf)[0], body_size);
    return file_body_binary_size_;
  }
  std::vector<unsigned char> buf_uncompressed;
  // Compute the uncompression binary body size
  size_t body_size = GetBodyBinarySize();
  buf_uncompressed.resize(body_size);
  size_t binary_size = map_matrix_handler_->CreateBinary(
      map_matrix_, &buf_uncompressed[0], body_size);
  if (binary_size == 0) {
    return 0;
  }

  compression_strategy_->Encode(&buf_uncompressed, buf);
  file_body_binary_size_ = buf->size();

  return buf->size();
}

size_t BaseMapNode::GetBodyBinarySize() const {
  return map_matrix_handler_->GetBinarySize(map_matrix_);
}

bool BaseMapNode::GetCoordinate(const Eigen::Vector2d& coordinate,
                                unsigned int* x, unsigned int* y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  unsigned int off_x = static_cast<unsigned int>(
      (coordinate[0] - left_top_corner[0]) / GetMapResolution());
  unsigned int off_y = static_cast<unsigned int>(
      (coordinate[1] - left_top_corner[1]) / GetMapResolution());
  if (off_x < this->map_config_->map_node_size_x_ &&
      off_y < this->map_config_->map_node_size_y_) {
    *x = off_x;
    *y = off_y;
    return true;
  }
  return false;
}

bool BaseMapNode::GetCoordinate(const Eigen::Vector3d& coordinate,
                                unsigned int* x, unsigned int* y) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  return GetCoordinate(coord2d, x, y);
}

Eigen::Vector2d BaseMapNode::GetCoordinate(unsigned int x,
                                           unsigned int y) const {
  const Eigen::Vector2d& left_top_corner = GetLeftTopCorner();
  Eigen::Vector2d coord(
      left_top_corner[0] + static_cast<float>(x) * GetMapResolution(),
      left_top_corner[1] + static_cast<float>(y) * GetMapResolution());
  return coord;
}

void BaseMapNode::SetMapNodeIndex(const MapNodeIndex& index) {
  map_node_config_->node_index_ = index;
  left_top_corner_ =
      ComputeLeftTopCorner(*map_config_, map_node_config_->node_index_);
}

Eigen::Vector2d BaseMapNode::ComputeLeftTopCorner(const BaseMapConfig& config,
                                                  const MapNodeIndex& index) {
  Eigen::Vector2d coord;
  coord[0] = config.map_range_.GetMinX() +
             static_cast<float>(config.map_node_size_x_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.n_);
  coord[1] = config.map_range_.GetMinY() +
             static_cast<float>(config.map_node_size_y_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.m_);
  if (coord[0] >= config.map_range_.GetMaxX()) {
    throw "[BaseMapNode::ComputeLeftTopCorner] coord[0]"
                " >= config.map_range_.GetMaxX()";
  }
  if (coord[1] >= config.map_range_.GetMaxY()) {
    throw "[BaseMapNode::compute_left_top_corner] coord[1]"
                " >= config.map_range_.GetMaxY()";
  }
  return coord;
}

Eigen::Vector2d BaseMapNode::GetLeftTopCorner(const BaseMapConfig& config,
                                              const MapNodeIndex& index) {
  Eigen::Vector2d coord;
  coord[0] = config.map_range_.GetMinX() +
             static_cast<float>(config.map_node_size_x_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.n_);
  coord[1] = config.map_range_.GetMinY() +
             static_cast<float>(config.map_node_size_y_) *
                 config.map_resolutions_[index.resolution_id_] *
                 static_cast<float>(index.m_);
  DCHECK_LT(coord[0], config.map_range_.GetMaxX());
  DCHECK_LT(coord[1], config.map_range_.GetMaxY());
  return coord;
}

bool BaseMapNode::CreateMapDirectory(const std::string& path) const {
  if (!cyber::common::DirectoryExists(path)) {
    return cyber::common::EnsureDirectory(path);
  }
  return true;
}

bool BaseMapNode::CreateMapDirectoryRecursively(
    const std::vector<std::string>& paths) const {
  std::string path = "";

  for (unsigned int i = 0; i < paths.size(); ++i) {
    path = path + paths[i];
    if (!CreateMapDirectory(path)) {
      return false;
    }
  }
  return true;
}

bool BaseMapNode::CheckMapDirectoryRecursively(
    const std::vector<std::string>& paths) const {
  std::string path = "";

  for (unsigned int i = 0; i < paths.size(); ++i) {
    path = path + paths[i];
    if (!(cyber::common::DirectoryExists(path))) {
      return false;
    }
  }

  return true;
}

bool BaseMapNode::SaveIntensityImage() const {
  std::string path = map_config_->map_folder_path_;

  char buf[1024];
  std::vector<std::string> paths;

  paths.push_back(path);

  snprintf(buf, sizeof(buf), "/image");
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%03u",
           map_node_config_->node_index_.resolution_id_);
  paths.push_back(buf);
  path = path + buf;

  paths.push_back(map_node_config_->node_index_.zone_id_ > 0 ? "/north"
                                                             : "/south");
  path = path + paths.back();

  snprintf(buf, sizeof(buf), "/%02d",
           abs(map_node_config_->node_index_.zone_id_));
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.m_);
  paths.push_back(buf);
  path = path + buf;

  if (!CreateMapDirectoryRecursively(paths)) {
    return false;
  }

  snprintf(buf, sizeof(buf), "/%08d.png", map_node_config_->node_index_.n_);
  path = path + buf;

  bool success = SaveIntensityImage(path);
  return success;
}

bool BaseMapNode::SaveIntensityImage(const std::string& path) const {
  cv::Mat image;
  map_matrix_->GetIntensityImg(&image);
  bool success = cv::imwrite(path, image);
  return success;
}

bool BaseMapNode::SaveAltitudeImage() const {
  std::string path = map_config_->map_folder_path_;

  char buf[1024];
  std::vector<std::string> paths;

  paths.push_back(path);

  snprintf(buf, sizeof(buf), "/image_alt");
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%03u",
           map_node_config_->node_index_.resolution_id_);
  paths.push_back(buf);
  path = path + buf;

  paths.push_back(map_node_config_->node_index_.zone_id_ > 0 ? "/north"
                                                             : "/south");
  path = path + paths.back();

  snprintf(buf, sizeof(buf), "/%02d",
           abs(map_node_config_->node_index_.zone_id_));
  paths.push_back(buf);
  path = path + buf;

  snprintf(buf, sizeof(buf), "/%08d", map_node_config_->node_index_.m_);
  paths.push_back(buf);
  path = path + buf;

  if (!CreateMapDirectoryRecursively(paths)) {
    return false;
  }

  snprintf(buf, sizeof(buf), "/%08d.png", map_node_config_->node_index_.n_);
  path = path + buf;

  bool success = SaveAltitudeImage(path);
  return success;
}

bool BaseMapNode::SaveAltitudeImage(const std::string& path) const {
  cv::Mat image;
  map_matrix_->GetAltitudeImg(&image);
  if (image.empty()) {
    return false;
  }

  bool success = cv::imwrite(path, image);
  return success;
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
