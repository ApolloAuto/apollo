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

#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

struct LossyMapCell2D {
  /**@brief The default constructor. */
  LossyMapCell2D();
  /**@brief Reset to default value. */
  inline void Reset();
  // /**@brief Load the map cell from a binary chunk.
  //  * @param <return> The size read (the real size of object).
  //  */
  // inline unsigned int LoadBinary(unsigned char * buf);
  // /**@brief Create the binary. Serialization of the object.
  //  * @param <buf, buf_size> The buffer and its size.
  //  * @param <return> The required or the used size of is returned.
  //  */
  // inline unsigned int CreateBinary(unsigned char * buf, unsigned int
  // buf_size) const;
  // /**@brief Get the binary size of the object. */
  // inline unsigned int GetBinarySize() const;
  /**@brief Overloading the assign operator. */
  LossyMapCell2D& operator=(const LossyMapCell2D& ref);
  /**@brief The number of samples in the cell. */
  unsigned int count;
  /**@brief The average intensity value. */
  float intensity;
  /**@brief The variance intensity value. */
  float intensity_var;
  /**@brief The average altitude of the cell. */
  float altitude;
  /**@brief The ground altitude of the cell. */
  float altitude_ground;
  /**@brief is ground altitude usefu */
  bool is_ground_useful;
};

class LossyMapMatrix2D : public BaseMapMatrix {
 public:
  LossyMapMatrix2D();
  ~LossyMapMatrix2D();
  LossyMapMatrix2D(const LossyMapMatrix2D& matrix);

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

  inline LossyMapCell2D* operator[](int row) {
    return map_cells_ + row * cols_;
  }
  inline const LossyMapCell2D* operator[](int row) const {
    return map_cells_ + row * cols_;
  }

  LossyMapMatrix2D& operator=(const LossyMapMatrix2D& matrix);

 protected:
  /**@brief The number of rows. */
  unsigned int rows_;
  /**@brief The number of columns. */
  unsigned int cols_;
  /**@brief The matrix data structure. */
  LossyMapCell2D* map_cells_;

 protected:
  inline unsigned char EncodeIntensity(const LossyMapCell2D& cell) const;
  inline void DecodeIntensity(unsigned char data, LossyMapCell2D* cell) const;
  inline uint16_t EncodeVar(const LossyMapCell2D& cell) const;
  inline void DecodeVar(uint16_t data, LossyMapCell2D* cell) const;
  inline uint16_t EncodeAltitudeGround(const LossyMapCell2D& cell) const;
  inline void DecodeAltitudeGround(uint16_t data, LossyMapCell2D* cell) const;
  inline uint16_t EncodeAltitudeAvg(const LossyMapCell2D& cell) const;
  inline void DecodeAltitudeAvg(uint16_t data, LossyMapCell2D* cell) const;
  inline unsigned char EncodeCount(const LossyMapCell2D& cell) const;
  inline void DecodeCount(unsigned char data, LossyMapCell2D* cell) const;
  const int var_range_ = 1023;  // 65535;
  const int var_ratio_ = 4;     // 256;
  // const unsigned int _alt_range = 1023;//65535;
  const float alt_ground_interval_ = 0.04f;
  const uint16_t ground_void_flag_ = 0xffff;
  const float alt_avg_interval_ = 0.04f;
  const int count_range_ = 2;  // 30;
  mutable float alt_avg_min_;
  mutable float alt_avg_max_;
  mutable float alt_ground_min_;
  mutable float alt_ground_max_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
