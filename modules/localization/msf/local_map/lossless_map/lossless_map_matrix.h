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

#include <vector>

#include "cyber/common/log.h"
#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The first layer (layer 0) includes all the intensities from any
 * layers. Other layers only include the samples from a layer. */
#define IDL_CAR_NUM_RESERVED_MAP_LAYER 2U

struct LosslessMapSingleCell {
  /**@brief The default constructor. */
  LosslessMapSingleCell();
  /**@brief Reset to default value. */
  inline void Reset();
  /**@brief Add a sample.*/
  void AddSample(const float new_altitude, const float new_intensity);
  /**@brief Overloading the assign operator. */
  LosslessMapSingleCell& operator=(const LosslessMapSingleCell& ref);
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  unsigned int CreateBinary(unsigned char* buf, unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  unsigned int GetBinarySize() const;

  /**@brief The average intensity value. */
  float intensity;
  /**@brief The variance intensity value. */
  float intensity_var;
  /**@brief The average altitude of the cell. */
  float altitude;
  /**@brief The variance altitude value of the cell. */
  float altitude_var;
  /**@brief The number of samples in the cell. */
  unsigned int count;
};

/**@brief The multiple layers of the cell. */
struct LosslessMapCell {
  /**@brief The default constructor. */
  LosslessMapCell();
  /**@brief Reset to default value. */
  void Reset();
  /**@brief Set the value of a layer that layer_id > 0.
   * The target layer is found according to the altitude. */
  void SetValueLayer(double altitude, unsigned char intensity,
                     double altitude_thres = 10.0);
  /**@brief Set the value.
   * @param <altitude> The altitude of the cell.
   * @param <intensity> The reflectance intensity.
   */
  void SetValue(double altitude, unsigned char intensity);
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  unsigned int CreateBinary(unsigned char* buf, unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  unsigned int GetBinarySize() const;

  /**@brief Match a layer in the map cell given an altitude.
   * @return The valid layer ID is 1 ~ N (The layer 0 is the layer includes all
   * the samples). If there is no existing layer, return 0. */
  unsigned int GetLayerId(double altitude) const;
  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  /**@brief Get the average intensity of all layers in the map cell. */
  void GetValue(std::vector<unsigned char>* values) const;
  /**@brief Get the variance of the intensity of all layers in the map cell. */
  void GetVar(std::vector<float>* vars) const;
  /**@brief Get the average altitude of all layers in the map cell. */
  void GetAlt(std::vector<float>* alts) const;
  /**@brief Get the variance of the altitude of all layers in the map cell. */
  void GetAltVar(std::vector<float>* alt_vars) const;
  /**@brief Get the count of the samples of all layers in the map cell. */
  void GetCount(std::vector<unsigned int>* counts) const;
  /**@brief Get the average intensity of the map cell. */
  inline unsigned char GetValue() const {
    return static_cast<unsigned char>(map_cells[0].intensity);
  }
  /**@brief Get the variance of the intensity of the map cell. */
  inline float GetVar() const { return map_cells[0].intensity_var; }
  /**@brief Get the average altitude of the map cell. */
  inline float GetAlt() const { return map_cells[0].altitude; }
  /**@brief Get the variance of the altitude of the map cell. */
  inline float GetAltVar() const { return map_cells[0].altitude_var; }
  /**@brief Get the count of the samples in the map cell. */
  inline unsigned int GetCount() const { return map_cells[0].count; }
  /**@brief Get a particular layer in the map cell. The layer 0 is the layer
   * includes all the samples. */
  LosslessMapSingleCell& GetLayer(unsigned int layer_id) {
    DCHECK_LT(layer_id, layer_num);
    return map_cells[layer_id];
  }
  /**@brief Get a perticular layer in the map cell. The layer 0 is the layer
   * includes all the samples. */
  const LosslessMapSingleCell& GetLayer(unsigned int layer_id) const {
    DCHECK_LT(layer_id, layer_num);
    return map_cells[layer_id];
  }

  /**@brief The layers of the cell. */
  unsigned int layer_num;
  /**@brief The multiple layers of the cell.
   * The first layer (layer 0) includes all the intensities from any layers.
   * Other layers only include the samples from a layer. */
  LosslessMapSingleCell map_cells[IDL_CAR_NUM_RESERVED_MAP_LAYER];
};

class LosslessMapMatrix : public BaseMapMatrix {
 public:
  LosslessMapMatrix();
  ~LosslessMapMatrix();
  LosslessMapMatrix(const LosslessMapMatrix& matrix);

  virtual void Init(const BaseMapConfig* config);
  /**@brief Reset map cells data. */
  virtual void Reset(const BaseMapConfig* config);

  void Init(unsigned int rows, unsigned int cols);
  void Reset(unsigned int rows, unsigned int cols);

  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual unsigned int LoadBinary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int CreateBinary(unsigned char* buf,
                                    unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  virtual unsigned int GetBinarySize() const;
  /**@brief get intensity image of node. */
  virtual void GetIntensityImg(cv::Mat* intensity_img) const;

  /**@brief Get a map cell. */
  inline const LosslessMapCell& GetMapCell(unsigned int row,
                                           unsigned int col) const {
    DCHECK_LT(row, rows_);
    DCHECK_LT(col, cols_);
    return map_cells_[row * cols_ + col];
  }
  /**@brief Get a map cell. */
  inline LosslessMapCell& GetMapCell(unsigned int row, unsigned int col) {
    DCHECK_LT(row, rows_);
    DCHECK_LT(col, cols_);
    return map_cells_[row * cols_ + col];
  }

  inline LosslessMapCell* operator[](int row) {
    return map_cells_ + row * cols_;
  }
  inline const LosslessMapCell* operator[](int row) const {
    return map_cells_ + row * cols_;
  }

 protected:
  /**@brief The number of rows. */
  unsigned int rows_;
  /**@brief The number of columns. */
  unsigned int cols_;
  /**@brief The matrix data structure. */
  LosslessMapCell* map_cells_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
