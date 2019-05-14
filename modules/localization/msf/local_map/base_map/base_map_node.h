/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "modules/localization/msf/common/util/compression.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of a Node in the map. */
class BaseMapNode {
 public:
  /**@brief Construct a map node. */
  BaseMapNode(BaseMapMatrix* matrix, CompressionStrategy* strategy);
  /**@brief Destruct a map node. */
  virtual ~BaseMapNode();

  /**@brief Initialize the map node. Call this function first before use it! */
  virtual void Init(const BaseMapConfig* map_config, const MapNodeIndex& index,
                    bool create_map_cells = true);
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
  /**@brief Load the map node from the disk. */
  bool Load();
  bool Load(const char* filename);

  // /**@brief Set compression strategy. */
  // void SetCompressionStrategy(compression::CompressionStrategy* strategy);
  /**@brief Get map cell matrix. */
  inline const BaseMapMatrix& GetMapCellMatrix() const { return *map_matrix_; }

  inline BaseMapMatrix& GetMapCellMatrix() { return *map_matrix_; }

  /**@brief Get the map settings. */
  inline const BaseMapConfig& GetMapConfig() const { return *map_config_; }
  /**@brief Set the map node index. */
  inline void SetMapNodeIndex(const MapNodeIndex& index) { index_ = index; }
  /**@brief Get the map node index. */
  inline const MapNodeIndex& GetMapNodeIndex() const { return index_; }
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
  /**@brief Get the left top corner of the map node. */
  // inline const idl::car::core::numerical::Vector2D& GetLeftTopCorner()
  // const {
  //     return left_top_corner_;
  // }

  inline const Eigen::Vector2d& GetLeftTopCorner() const {
    return left_top_corner_;
  }

  inline void SetLeftTopCorner(double x, double y) {
    // left_top_corner_[0] = x;
    // left_top_corner_[1] = y;

    left_top_corner_[0] = x;
    left_top_corner_[1] = y;
  }
  /**@brief Get the resolution of this map nodex. */
  inline float GetMapResolution() const {
    return this->map_config_->map_resolutions_[this->index_.resolution_id_];
  }
  /**@brief Given the global coordinate, get the local 2D coordinate of the map
   * cell matrix. <return> If global coordinate (x, y) belongs to this map node.
   */
  // bool GetCoordinate(const idl::car::core::numerical::Vector2D& coordinate,
  //                     unsigned int& x, unsigned int& y) const;
  // bool GetCoordinate(const idl::car::core::numerical::Vector3D& coordinate,
  //                     unsigned int& x, unsigned int& y) const;
  /**@brief Given the local 2D coordinate, return the global coordinate. */
  // idl::car::core::numerical::Vector2D GetCoordinate(unsigned int x, unsigned
  // int y) const;

  /**@brief Given the global coordinate, get the local 2D coordinate of the map
   * cell matrix. <return> If global coordinate (x, y) belongs to this map node,
   * eigen version. */
  bool GetCoordinate(const Eigen::Vector2d& coordinate, unsigned int* x,
                     unsigned int* y) const;
  bool GetCoordinate(const Eigen::Vector3d& coordinate, unsigned int* x,
                     unsigned int* y) const;
  /**@brief Given the local 2D coordinate, return the global coordinate, eigen
   * version. */
  Eigen::Vector2d GetCoordinate(unsigned int x, unsigned int y) const;

  // static idl::car::core::numerical::Vector2D GetLeftTopCorner(const
  // BaseMapConfig& option,
  //                                                      const MapNodeIndex&
  //                                                      index);
  static Eigen::Vector2d GetLeftTopCorner(const BaseMapConfig& option,
                                          const MapNodeIndex& index);

 protected:
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual unsigned int LoadBinary(FILE* file);
  /**@brief Create the binary. Serialization of the object.
   * @param <return> The the used size of binary is returned.
   */
  virtual unsigned int CreateBinary(FILE* file) const;
  /**@brief Get the binary size of the object. */
  virtual unsigned int GetBinarySize() const;
  /**@brief Load the map node header from a binary chunk.
   * @param <return> The size read (the real size of header).
   */
  virtual unsigned int LoadHeaderBinary(unsigned char* buf);
  /**@brief Create the binary header.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int CreateHeaderBinary(unsigned char* buf,
                                          unsigned int buf_size) const;
  /**@brief Get the size of the header in bytes. */
  virtual unsigned int GetHeaderBinarySize() const;
  /**@brief Load the map node body from a binary chunk.
   * @param <return> The size read (the real size of body).
   */
  virtual unsigned int LoadBodyBinary(std::vector<unsigned char>* buf);
  /**@brief Create the binary body.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int CreateBodyBinary(std::vector<unsigned char>* buf) const;
  /**@brief Get the size of the body in bytes. */
  virtual unsigned int GetBodyBinarySize() const;
  /**@brief Save intensity image of node. */
  bool SaveIntensityImage(const std::string& path) const;

  /**@brief The map settings. */
  const BaseMapConfig* map_config_ = nullptr;
  /**@brief The index of this node. */
  MapNodeIndex index_;
  /**@brief The left top corner of the map node in the global coordinate system.
   */
  // idl::car::core::numerical::Vector2D left_top_corner_;
  Eigen::Vector2d left_top_corner_;
  /**@brief The data structure of the map datas, which is a matrix. */
  BaseMapMatrix* map_matrix_;
  /**@brief If the node is reserved in map. */
  bool is_reserved_ = false;
  /**@brief Has the map node been changed. */
  bool is_changed_ = false;
  /* *@brief Indicate map node data is ready*/
  bool data_is_ready_ = false;
  /**@brief The body binary size in file. It's useful when reading and writing
   * files. */
  mutable unsigned int file_body_binary_size_ = 0;
  /**@bried The compression strategy. */
  CompressionStrategy* compression_strategy_ = nullptr;
  /**@brief The min altitude of point cloud in the node. */
  float min_altitude_ = 1e6;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
