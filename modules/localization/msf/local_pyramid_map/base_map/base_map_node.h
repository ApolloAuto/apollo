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

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

#include "modules/localization/msf/common/util/compression.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_matrix_handler.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_config.h"
#include "modules/localization/msf/local_pyramid_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {

/**@brief The data structure of a Node in the map. */
class BaseMapNode {
 public:
  /**@brief Construct a map node. */
  BaseMapNode();
  /**@brief Construct a map node. */
  BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy);
  /**@brief Destruct a map node. */
  virtual ~BaseMapNode();

  /**@brief Initialize the map node. Call this function first before use it! */
  virtual void Init(const BaseMapConfig* map_config) = 0;
  virtual void Init(const BaseMapConfig* map_config, const MapNodeIndex& index,
                    bool create_map_cells = true) = 0;

  /**@brief Initialize the map matrix. */
  virtual void InitMapMatrix(const BaseMapConfig* map_config);
  /**@brief call before deconstruction or reset. */
  virtual void Finalize();
  /**@brief Reset map cells data. */
  virtual void ResetMapNode();

  /**@brief Save the map node to the disk. */
  bool Save();
  /**@brief Save intensity image of node. */
  bool SaveIntensityImage() const;
  /**@brief Save altitude image of node. */
  bool SaveAltitudeImage() const;
  /**@brief Load the map node from the disk. */
  bool Load();
  bool Load(const char* filename);

  /**@brief Given the global coordinate, get the local 2D coordinate of the map
   * cell matrix.
   * <return> If global coordinate (x, y) belongs to this map node. */
  virtual bool GetCoordinate(const Eigen::Vector2d& coordinate, unsigned int* x,
                             unsigned int* y) const;
  virtual bool GetCoordinate(const Eigen::Vector3d& coordinate, unsigned int* x,
                             unsigned int* y) const;
  /**@brief Given the local 2D coordinate, return the global coordinate. */
  virtual Eigen::Vector2d GetCoordinate(unsigned int x, unsigned int y) const;

  /**@brief Set the map node index. */
  void SetMapNodeIndex(const MapNodeIndex& index);

  /**@brief Save intensity image of node. */
  bool SaveIntensityImage(const std::string& path) const;
  /**@brief Save altitude image of node. */
  bool SaveAltitudeImage(const std::string& path) const;

  static Eigen::Vector2d ComputeLeftTopCorner(const BaseMapConfig& config,
                                              const MapNodeIndex& index);

  /**@brief Get map cell matrix. */
  inline const BaseMapMatrix& GetMapCellMatrix() const { return *map_matrix_; }
  inline BaseMapMatrix& GetMapCellMatrix() { return *map_matrix_; }

  /**@brief Get the map settings. */
  inline const BaseMapConfig& GetMapConfig() const { return *map_config_; }

  /**@brief Get the map node config. */
  inline const BaseMapNodeConfig& GetMapNodeConfig() const {
    return *map_node_config_;
  }

  /**@brief Get the map node index. */
  inline const MapNodeIndex& GetMapNodeIndex() const {
    return map_node_config_->node_index_;
  }

  /**@brief Set if the map node is reserved. */
  inline void SetIsReserved(bool is_reserved) { is_reserved_ = is_reserved; }

  /**@brief Get if the map node is reserved. */
  inline bool GetIsReserved() const { return is_reserved_; }

  /**@brief Get if the map data has changed. */
  inline bool GetIsChanged() const { return is_changed_; }

  /**@brief Set if the map node data has changed. */
  inline void SetIsChanged(bool is) { is_changed_ = is; }

  /**@brief Get if the map node data is ready*/
  inline bool GetIsReady() const { return data_is_ready_; }

  inline const Eigen::Vector2d& GetLeftTopCorner() const {
    return left_top_corner_;
  }

  /**@brief Set the left top corner of the map node. */
  inline void SetLeftTopCorner(double x, double y) {
    left_top_corner_[0] = x;
    left_top_corner_[1] = y;
  }

  /**@brief Get the resolution of this map nodex. */
  inline float GetMapResolution() const {
    return this->map_config_
        ->map_resolutions_[map_node_config_->node_index_.resolution_id_];
  }

  static Eigen::Vector2d GetLeftTopCorner(const BaseMapConfig& option,
                                          const MapNodeIndex& index);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  /**@brief Try to create the map directory. */
  bool CreateMapDirectory(const std::string& path) const;
  /**@brief Try to create the map directory recursively. */
  bool CreateMapDirectoryRecursively(
      const std::vector<std::string>& paths) const;
  /**@brief Try to check the map directory recursively. */
  bool CheckMapDirectoryRecursively(
      const std::vector<std::string>& paths) const;

  /**@brief Load the map cell from a binary chunk.
   */
  virtual bool LoadBinary(FILE* file);
  /**@brief Create the binary. Serialization of the object.
   */
  virtual bool CreateBinary(FILE* file) const;
  /**@brief Get the binary size of the object. */
  virtual size_t GetBinarySize() const;

  /**@brief Load the map node header from a binary chunk.
   * @param <return> The size read (the real size of header).
   */
  virtual size_t LoadHeaderBinary(const unsigned char* buf);
  /**@brief Create the binary header.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual size_t CreateHeaderBinary(unsigned char* buf, size_t buf_size) const;
  /**@brief Get the size of the header in bytes. */
  virtual size_t GetHeaderBinarySize() const;

  /**@brief Load the map node body from a binary chunk.
   * @param <return> The size read (the real size of body).
   */
  virtual size_t LoadBodyBinary(std::vector<unsigned char>* buf);
  /**@brief Create the binary body.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual size_t CreateBodyBinary(std::vector<unsigned char>* buf) const;
  /**@brief Get the size of the body in bytes. */
  virtual size_t GetBodyBinarySize() const;

  /**@brief The map settings. */
  const BaseMapConfig* map_config_ = nullptr;

  /**@brief The index of this node*/
  MapNodeIndex index_;

  /**@brief The left top corner of the map node in the global coordinate system.
   */
  Eigen::Vector2d left_top_corner_;

  /**@brief The map node config. */
  std::shared_ptr<BaseMapNodeConfig> map_node_config_ = nullptr;
  /**@brief The data structure of the map datas, which is a matrix. */
  std::shared_ptr<BaseMapMatrix> map_matrix_ = nullptr;
  /**@brief The class to load and create map matrix binary. */
  std::shared_ptr<BaseMapMatrixHandler> map_matrix_handler_ = nullptr;
  /**@brief If the node is reserved in map. */
  bool is_reserved_ = false;
  /**@brief Has the map node been changed. */
  bool is_changed_ = false;
  /* *@brief Indicate map node data is ready*/
  bool data_is_ready_ = false;
  /**@brief The body binary size in file. */
  mutable size_t file_body_binary_size_ = 0;

  mutable size_t uncompressed_file_body_size_ = 0;
  /**@bried The compression strategy. */
  std::shared_ptr<CompressionStrategy> compression_strategy_ = nullptr;
};

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
