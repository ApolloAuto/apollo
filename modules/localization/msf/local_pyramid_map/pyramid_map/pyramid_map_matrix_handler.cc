
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
#include "modules/localization/msf/local_pyramid_map/pyramid_map/pyramid_map_matrix_handler.h"

#include <memory>
#include "cyber/common/log.h"

namespace apollo {
namespace localization {
namespace msf {
namespace pyramid_map {
// =================PyramidMapMatrixHandlerSelector=================
PyramidMapMatrixHandlerSelector::PyramidMapMatrixHandlerSelector() {}

PyramidMapMatrixHandlerSelector::~PyramidMapMatrixHandlerSelector() {}

BaseMapMatrixHandler*
PyramidMapMatrixHandlerSelector::AllocPyramidMapMatrixHandler(
    MapVersion version) {
  switch (version) {
    case MapVersion::LOSSY_FULL_ALT_MAP:
      return new LossyMapFullAltMatrixHandler();
    case MapVersion::LOSSLESS_MAP:
      return new LosslessMapMatrixHandler();
    case MapVersion::PYRAMID_LOSSY_MAP:
      return new PyramidLossyMapMatrixHandler();
    case MapVersion::PYRAMID_LOSSLESS_MAP:
      return new PyramidLosslessMapMatrixHandler();
    case MapVersion::UNKNOWN:
    default:
      AINFO << "Unknown map version!";
  }
  return nullptr;
}

// =================LossyMapMatrixHandler=================
LossyMapMatrixHandler::LossyMapMatrixHandler() {}

LossyMapMatrixHandler::~LossyMapMatrixHandler() {}

unsigned char LossyMapMatrixHandler::EncodeIntensity(float intensity) const {
  unsigned char encoded_intensity = 0;
  if (intensity > 255) {
    encoded_intensity = 255;
  } else if (intensity < 0) {
    encoded_intensity = 0;
  } else {
    encoded_intensity = static_cast<unsigned char>(intensity);
  }
  return encoded_intensity;
}

void LossyMapMatrixHandler::DecodeIntensity(unsigned char data,
                                            float* intensity) const {
  *intensity = static_cast<float>(data);
}

uint16_t LossyMapMatrixHandler::EncodeIntensityVar(float var) const {
  var = std::sqrt(var);
  unsigned int encoded_var =
      static_cast<unsigned int>(static_cast<float>(var_range_) /
                                (var * static_cast<float>(var_ratio_) + 1.0));
  if (encoded_var > var_range_) {
    encoded_var = var_range_;
  }
  if (encoded_var < 1) {
    encoded_var = 1;
  }
  return static_cast<uint16_t>(encoded_var);
}

void LossyMapMatrixHandler::DecodeIntensityVar(uint16_t data,
                                               float* var) const {
  *var = static_cast<float>(data);
  *var = (static_cast<float>(var_range_) / (*var) - 1.0f) /
         static_cast<float>(var_ratio_);
  *var = (*var) * (*var);
}

uint16_t LossyMapMatrixHandler::EncodeAltitude(float altitude,
                                               float min_altitude,
                                               float altitude_interval) const {
  float delta_alt = altitude - min_altitude;
  delta_alt /= altitude_interval;
  int encoded_altitude = static_cast<int>(delta_alt + 0.5f);
  if (encoded_altitude > 0xffff) {
    encoded_altitude = 0xffff;
  }
  if (encoded_altitude < 0) {
    encoded_altitude = 0;
  }
  return static_cast<uint16_t>(encoded_altitude);
}

void LossyMapMatrixHandler::DecodeAltitude(uint16_t data, float min_altitude,
                                           float altitude_interval,
                                           float* altitude) const {
  *altitude = min_altitude + data * altitude_interval;
}

unsigned char LossyMapMatrixHandler::EncodeCount(
    unsigned int count, unsigned int count_range) const {
  unsigned int encoded_count = 0;
  while (count > 0) {
    ++encoded_count;
    count /= 2;
  }
  if (encoded_count > count_range) {
    encoded_count = count_range;
  }
  return static_cast<unsigned char>(encoded_count);
}

void LossyMapMatrixHandler::DecodeCount(unsigned char data,
                                        unsigned int* count) const {
  if (data == 0) {
    *count = data;
  } else {
    *count = 1 << (data - 1);
  }
}

// =================LossyMapFullAltMatrixHandler=================
LossyMapFullAltMatrixHandler::LossyMapFullAltMatrixHandler() {}

LossyMapFullAltMatrixHandler::~LossyMapFullAltMatrixHandler() {}

size_t LossyMapFullAltMatrixHandler::LoadBinary(
    const unsigned char* buf, std::shared_ptr<BaseMapMatrix> base_matrix) {
  std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t binary_size = sizeof(unsigned int) * 2;
  const unsigned int* uint_p = reinterpret_cast<const unsigned int*>(buf);
  unsigned int rows = *uint_p;  // rows in first level
  ++uint_p;
  unsigned int cols = *uint_p;  // cols in first level
  ++uint_p;

  // reset or init matrix
  if (  // matrix->get_resolution_num() == 1 &&
        // matrix->get_resolution_ratio() == 2 &&
      rows == matrix->GetRows() && cols == matrix->GetCols() &&
      matrix->HasIntensity() && matrix->HasIntensityVar() &&
      matrix->HasAltitude() && matrix->HasGroundAltitude() &&
      matrix->HasCount()) {
    matrix->Reset();
  } else {
    matrix->Init(rows, cols, true, true, true, false, true, true, false);
  }

  // alt min max & ground alt min max
  binary_size += sizeof(float) * 4;
  const float* float_p =
      reinterpret_cast<const float*>(reinterpret_cast<const void*>(uint_p));
  alt_avg_min_ = *float_p;
  ++float_p;
  alt_avg_max_ = *float_p;
  ++float_p;
  ground_alt_min_ = *float_p;
  ++float_p;
  ground_alt_max_ = *float_p;
  ++float_p;

  // load matrix
  const unsigned char* uc_p = reinterpret_cast<const unsigned char*>(float_p);
  unsigned int matrix_size = rows * cols;

  if (matrix->HasCount()) {
    binary_size += sizeof(unsigned char) * matrix_size;
    UIntMatrix* count_matrix = matrix->GetCountMatrixSafe();
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        DecodeCount(uc_p[r * cols + c], &(*count_matrix)[r][c]);
      }
    }
    uc_p += matrix_size;
  }

  if (matrix->HasIntensity()) {
    binary_size += sizeof(unsigned char) * matrix_size;
    FloatMatrix* intensity_matrix = matrix->GetIntensityMatrixSafe();
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        DecodeIntensity(uc_p[r * cols + c], &(*intensity_matrix)[r][c]);
      }
    }
    uc_p += matrix_size;
  }

  if (matrix->HasIntensityVar()) {
    binary_size += sizeof(uint16_t) * matrix_size;
    FloatMatrix* intensity_var_matrix = matrix->GetIntensityVarMatrixSafe();
    const unsigned char* p_low = uc_p + matrix_size;
    const unsigned char* p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        unsigned int var = static_cast<unsigned int>(p_high[r * cols + c]);
        var = var * 256u + static_cast<unsigned int>(p_low[r * cols + c]);
        DecodeIntensityVar(static_cast<uint16_t>(var),
                           &(*intensity_var_matrix)[r][c]);
      }
    }
    uc_p += 2 * matrix_size;
  }

  if (matrix->HasAltitude()) {
    binary_size += sizeof(uint16_t) * matrix_size;
    FloatMatrix* altitude_matrix = matrix->GetAltitudeMatrixSafe();
    const unsigned char* p_low = uc_p + matrix_size;
    const unsigned char* p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        unsigned int alt = static_cast<unsigned int>(p_high[r * cols + c]);
        alt = alt * 256u + static_cast<unsigned int>(p_low[r * cols + c]);
        DecodeAltitude(static_cast<uint16_t>(alt), alt_avg_min_,
                       alt_avg_interval_, &(*altitude_matrix)[r][c]);
      }
    }
    uc_p += 2 * matrix_size;
  }

  if (matrix->HasGroundAltitude()) {
    binary_size += sizeof(uint16_t) * matrix_size;
    FloatMatrix* ground_altitude_matrix = matrix->GetGroundAltitudeMatrixSafe();
    const unsigned char* p_low = uc_p + matrix_size;
    const unsigned char* p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        unsigned int alt = static_cast<unsigned int>(p_high[r * cols + c]);
        alt = alt * 256u + static_cast<unsigned int>(p_low[r * cols + c]);
        DecodeAltitude(static_cast<uint16_t>(alt), ground_alt_min_,
                       ground_alt_interval_, &(*ground_altitude_matrix)[r][c]);
      }
    }
    uc_p += 2 * matrix_size;
  }

  return binary_size;
}

size_t LossyMapFullAltMatrixHandler::CreateBinary(
    const std::shared_ptr<BaseMapMatrix> base_matrix, unsigned char* buf,
    size_t buf_size) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t target_size = GetBinarySize(matrix);
  if (buf_size >= target_size) {
    unsigned int rows = matrix->GetRowsSafe();
    unsigned int cols = matrix->GetColsSafe();
    unsigned int matrix_size = rows * cols;

    unsigned int* uint_p = reinterpret_cast<unsigned int*>(buf);
    *uint_p = rows;
    ++uint_p;
    *uint_p = cols;
    ++uint_p;
    // buf_size -= sizeof(unsigned int) * 2;

    float* float_p = reinterpret_cast<float*>(reinterpret_cast<void*>(uint_p));
    if (matrix->HasAltitude() && matrix->HasCount()) {
      alt_avg_min_ = 1e8;
      alt_avg_max_ = -1e8;
      for (unsigned int y = 0; y < rows; ++y) {
        for (unsigned int x = 0; x < cols; ++x) {
          const float* altitude = matrix->GetAltitudeSafe(y, x);
          const unsigned int* count = matrix->GetCountSafe(y, x);
          if (*count == 0) {
            continue;
          }

          if (*altitude < alt_avg_min_) {
            alt_avg_min_ = *altitude;
          }
          if (*altitude > alt_avg_max_) {
            alt_avg_max_ = *altitude;
          }
        }
      }
    } else {
      alt_avg_min_ = 0.0;
      alt_avg_max_ = 0.0;
    }

    if (matrix->HasGroundAltitude() && matrix->HasGroundCount()) {
      ground_alt_min_ = 1e8;
      ground_alt_max_ = -1e8;
      for (unsigned int y = 0; y < rows; ++y) {
        for (unsigned int x = 0; x < cols; ++x) {
          const float* ground_altitude = matrix->GetGroundAltitudeSafe(y, x);
          const unsigned int* ground_count = matrix->GetGroundCountSafe(y, x);
          if (*ground_count == 0) {
            continue;
          }
          if (*ground_altitude < ground_alt_min_) {
            ground_alt_min_ = *ground_altitude;
          }
          if (*ground_altitude > ground_alt_max_) {
            ground_alt_max_ = *ground_altitude;
          }
        }
      }
    } else {
      ground_alt_min_ = 0.0;
      ground_alt_max_ = 0.0;
    }

    *float_p = alt_avg_min_;
    ++float_p;
    *float_p = alt_avg_max_;
    ++float_p;
    *float_p = ground_alt_min_;
    ++float_p;
    *float_p = ground_alt_max_;
    ++float_p;
    // buf_size -= sizeof(float) * 4;

    unsigned char* uc_p = reinterpret_cast<unsigned char*>(float_p);
    // unsigned int processed_size = 0;

    // count
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        const unsigned int* count = matrix->GetCountSafe(r, c);
        unsigned int ct = (count != nullptr) ? *count : 0;
        uc_p[r * cols + c] = EncodeCount(ct, count_range_);
      }
    }
    uc_p += matrix_size;
    // processed_size += matrix_size * sizeof(unsigned char);

    // intensity
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        const float* intensity =
            static_cast<const float*>(matrix->GetIntensitySafe(r, c));
        float ity = (intensity != nullptr) ? *intensity : 0.0f;
        uc_p[r * cols + c] = EncodeIntensity(ity);
      }
    }
    uc_p += matrix_size;
    // processed_size += matrix_size * sizeof(unsigned char);

    // intensiy
    unsigned char* p_low = uc_p + matrix_size;
    unsigned char* p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        const float* intensity_var =
            static_cast<const float*>(matrix->GetIntensityVarSafe(r, c));
        float iv = (intensity_var != nullptr) ? *intensity_var : 0.0f;
        uint16_t var = EncodeIntensityVar(iv);
        p_high[r * cols + c] = static_cast<uint8_t>(var / 256);
        p_low[r * cols + c] = static_cast<uint8_t>(var % 256);
      }
    }
    uc_p += 2 * matrix_size;
    // processed_size += matrix_size * sizeof(uint16_t);

    // altitude
    p_low = uc_p + matrix_size;
    p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        const float* altitude = matrix->GetAltitudeSafe(r, c);
        float a = (altitude != nullptr) ? *altitude : 0.0f;
        uint16_t alt = EncodeAltitude(a, alt_avg_min_, alt_avg_interval_);
        p_high[r * cols + c] = static_cast<unsigned char>(alt / 256);
        p_low[r * cols + c] = static_cast<unsigned char>(alt % 256);
      }
    }
    uc_p += 2 * matrix_size;
    // processed_size += matrix_size * sizeof(uint16_t);

    // ground altitude
    p_low = uc_p + matrix_size;
    p_high = uc_p;
    for (unsigned int r = 0; r < rows; ++r) {
      for (unsigned int c = 0; c < cols; ++c) {
        const float* ground_altitude = matrix->GetGroundAltitudeSafe(r, c);
        float ga = (ground_altitude != nullptr) ? *ground_altitude : 0.0f;
        uint16_t alt =
            EncodeAltitude(ga, ground_alt_min_, ground_alt_interval_);
        p_high[r * cols + c] = static_cast<unsigned char>(alt / 256);
        p_low[r * cols + c] = static_cast<unsigned char>(alt % 256);
      }
    }
    uc_p += 2 * matrix_size;
    // processed_size += matrix_size * sizeof(uint16_t);

    // assert(buf_size >= processed_size);
    // buf_size -= processed_size;
    return target_size;
  }

  return 0;
}

size_t LossyMapFullAltMatrixHandler::GetBinarySize(
    const std::shared_ptr<BaseMapMatrix> base_matrix) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);
  // assert(matrix->get_resolution_num() > 0);
  if (matrix->GetResolutionNum() == 0) {
    throw "[LossyMapFullAltMatrixHandler::get_binary_size]"
        "matrix->get_resolution_num() == 0";
  }

  // rows and cols
  // altitude min max & ground altitude min max
  size_t target_size = sizeof(unsigned int) * 2 + sizeof(float) * 4;
  // count, intensity, intensity_var, altitude_avg, altitude_ground
  target_size += matrix->GetRowsSafe() * matrix->GetColsSafe() *
                 (sizeof(unsigned char) + sizeof(unsigned char) +
                  sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t));
  return target_size;
}

// =================LosslessMapMatrixHandler====================
LosslessMapMatrixHandler::LosslessMapMatrixHandler() {}

LosslessMapMatrixHandler::~LosslessMapMatrixHandler() {}

size_t LosslessMapMatrixHandler::LoadBinary(
    const unsigned char* buf, std::shared_ptr<BaseMapMatrix> base_matrix) {
  std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t binary_size = sizeof(unsigned int) * 2;  // rows and cols

  const unsigned int* uint_p = reinterpret_cast<const unsigned int*>(buf);
  unsigned int rows = *uint_p;
  ++uint_p;
  unsigned int cols = *uint_p;
  ++uint_p;

  // reset or init matrix
  if (  // matrix->get_resolution_num() == 1 &&
        // matrix->get_resolution_ratio() == 2 &&
      rows == matrix->GetRowsSafe() && cols == matrix->GetColsSafe() &&
      matrix->HasIntensity() && matrix->HasIntensityVar() &&
      matrix->HasAltitude() && matrix->HasAltitudeVar() &&
      matrix->HasGroundAltitude() && matrix->HasCount() &&
      matrix->HasGroundCount()) {
    matrix->Reset();
  } else {
    matrix->Init(rows, cols);
  }

  // load matrix
  for (unsigned int y = 0; y < rows; y++) {
    for (unsigned int x = 0; x < cols; x++) {
      binary_size += sizeof(unsigned int);  // layer

      unsigned int size = *uint_p;
      ++uint_p;

      for (unsigned int i = 0; i < size; i++) {
        if (i == 0) {  // all points layer
          binary_size += sizeof(float) * 4 + sizeof(unsigned int);

          const float* float_p = reinterpret_cast<const float*>(
              reinterpret_cast<const void*>(uint_p));
          matrix->SetIntensitySafe(*float_p, y, x);
          ++float_p;
          matrix->SetIntensityVarSafe(*float_p, y, x);
          ++float_p;
          matrix->SetAltitudeSafe(*float_p, y, x);
          ++float_p;
          matrix->SetAltitudeVarSafe(*float_p, y, x);
          ++float_p;
          uint_p = reinterpret_cast<const unsigned int*>(
              reinterpret_cast<const void*>(float_p));
          matrix->SetCountSafe(*uint_p, y, x);
          ++uint_p;
        } else if (i == 1) {  // ground points layer
          binary_size += sizeof(float) * 4 + sizeof(unsigned int);

          const float* float_p = reinterpret_cast<const float*>(
              reinterpret_cast<const void*>(uint_p));
          ++float_p;
          ++float_p;
          matrix->SetGroundAltitudeSafe(*float_p, y, x);
          ++float_p;
          ++float_p;
          uint_p = reinterpret_cast<const unsigned int*>(
              reinterpret_cast<const void*>(float_p));
          matrix->SetGroundCountSafe(*uint_p, y, x);
          ++uint_p;
        }
      }
    }
  }

  return binary_size;
}

size_t LosslessMapMatrixHandler::CreateBinary(
    const std::shared_ptr<BaseMapMatrix> base_matrix, unsigned char* buf,
    size_t buf_size) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t target_size = GetBinarySize(matrix);
  if (buf_size >= target_size) {
    unsigned int rows = matrix->GetRowsSafe();
    unsigned int cols = matrix->GetColsSafe();

    unsigned int* uint_p = reinterpret_cast<unsigned int*>(buf);
    *uint_p = rows;
    ++uint_p;
    *uint_p = cols;
    ++uint_p;
    // buf_size -= sizeof(unsigned int) * 2;
    for (unsigned int y = 0; y < rows; y++) {
      for (unsigned int x = 0; x < cols; x++) {
        const unsigned int* ground_count = matrix->GetGroundCountSafe(y, x);
        unsigned int layer_num = 0;
        if (ground_count != NULL && *ground_count > 0) {
          layer_num = 2;
          *uint_p = 2;
          ++uint_p;
        } else {
          layer_num = 1;
          *uint_p = 1;
          ++uint_p;
        }

        // buf_size -= sizeof(unsigned int);
        for (unsigned int i = 0; i < layer_num; i++) {
          if (i == 0) {  // all points layer
            const float* intensity = matrix->GetIntensitySafe(y, x);
            const float* intensity_var = matrix->GetIntensityVarSafe(y, x);
            const float* altitude = matrix->GetAltitudeSafe(y, x);
            const float* altitude_var = matrix->GetAltitudeVarSafe(y, x);
            const unsigned int* count = matrix->GetCountSafe(y, x);

            float* float_p =
                reinterpret_cast<float*>(reinterpret_cast<void*>(uint_p));
            *float_p = (intensity != nullptr) ? *intensity : 0.0f;
            ++float_p;
            *float_p = (intensity_var != nullptr) ? *intensity_var : 0.0f;
            ++float_p;
            *float_p = (altitude != nullptr) ? *altitude : 0.0f;
            ++float_p;
            *float_p = (altitude_var != nullptr) ? *altitude_var : 0.0f;
            ++float_p;
            uint_p = reinterpret_cast<unsigned int*>(
                reinterpret_cast<void*>(float_p));
            *uint_p = (count != nullptr) ? *count : 0;
            ++uint_p;
          } else if (i == 1) {  // ground points layer
            const float* ground_altitude = matrix->GetGroundAltitudeSafe(y, x);
            const unsigned int* ground_count = matrix->GetGroundCountSafe(y, x);

            float* float_p =
                reinterpret_cast<float*>(reinterpret_cast<void*>(uint_p));
            *float_p = 0.0f;
            ++float_p;
            *float_p = 0.0f;
            ++float_p;
            *float_p = (ground_altitude != nullptr) ? *ground_altitude : 0.0f;
            ++float_p;
            *float_p = 0.0f;
            ++float_p;
            uint_p = reinterpret_cast<unsigned int*>(
                reinterpret_cast<void*>(float_p));
            *uint_p = (ground_count != nullptr) ? *ground_count : 0;
            ++uint_p;
          }

          // unsigned int processed_size += sizeof(float) * 4 + sizeof(unsigned
          // int); assert(buf_size >= processed_size); buf_size -=
          // processed_size;
        }
      }
    }

    return target_size;
  }

  return 0;
}

size_t LosslessMapMatrixHandler::GetBinarySize(
    const std::shared_ptr<BaseMapMatrix> base_matrix) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);
  // assert(matrix->get_resolution_num() > 0);
  if (matrix->GetResolutionNum() == 0) {
    throw "[LosslessMapMatrixHandler::get_binary_size]"
        "matrix->get_resolution_num() == 0";
  }

  size_t target_size = sizeof(unsigned int) * 2;  // rows and cols
  for (unsigned int y = 0; y < matrix->GetRowsSafe(); y++) {
    for (unsigned int x = 0; x < matrix->GetColsSafe(); x++) {
      target_size += sizeof(unsigned int);  // layer

      const unsigned int* ground_count = matrix->GetGroundCountSafe(y, x);
      if (ground_count != NULL && *ground_count > 0) {
        target_size += (sizeof(float) * 4 + sizeof(unsigned int)) * 2;
      } else {
        target_size += sizeof(float) * 4 + sizeof(unsigned int);
      }
    }
  }

  return target_size;
}

// =================PyramidLossyMapMatrixHandler====================
PyramidLossyMapMatrixHandler::PyramidLossyMapMatrixHandler() {}

PyramidLossyMapMatrixHandler::~PyramidLossyMapMatrixHandler() {}

size_t PyramidLossyMapMatrixHandler::LoadBinary(
    const unsigned char* buf, std::shared_ptr<BaseMapMatrix> base_matrix) {
  std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t binary_size = sizeof(unsigned int) * 4;
  const unsigned int* uint_p =
      reinterpret_cast<const unsigned int*>(buf);  // resolution num
  unsigned int resolution_num = *uint_p;
  ++uint_p;
  unsigned int ratio = *uint_p;  // ratio
  ++uint_p;
  unsigned int rows = *uint_p;  // rows in first level
  ++uint_p;
  unsigned int cols = *uint_p;  // cols in first level
  ++uint_p;

  // set has_*
  binary_size += sizeof(unsigned char) * 4;
  const unsigned char* uc_p = reinterpret_cast<const unsigned char*>(uint_p);
  ++uc_p;
  ++uc_p;
  ++uc_p;
  unsigned char has_flag = *uc_p;
  ++uc_p;
  bool has_intensity = (has_flag & 1);
  bool has_intensity_var = (has_flag & 2);
  bool has_altitude = (has_flag & 4);
  bool has_altitude_var = (has_flag & 8);
  bool has_ground_altitude = (has_flag & 16);
  bool has_count = (has_flag & 32);
  bool has_ground_count = (has_flag & 64);

  // reset or init matrix
  if (resolution_num == matrix->GetResolutionNum() &&
      ratio == matrix->GetResolutionRatio() && rows == matrix->GetRowsSafe() &&
      cols == matrix->GetColsSafe() &&
      has_intensity == matrix->HasIntensity() &&
      has_intensity_var == matrix->HasIntensityVar() &&
      has_altitude == matrix->HasAltitude() &&
      has_altitude_var == matrix->HasAltitudeVar() &&
      has_ground_altitude == matrix->HasGroundAltitude() &&
      has_count == matrix->HasCount() &&
      has_ground_count == matrix->HasGroundCount()) {
    matrix->Reset();
  } else {
    matrix->Init(rows, cols, has_intensity, has_intensity_var, has_altitude,
                 has_altitude_var, has_ground_altitude, has_count,
                 has_ground_count, resolution_num, ratio);
  }

  // alt min & ground alt min
  binary_size += sizeof(float) * 2;
  const float* float_p =
      reinterpret_cast<const float*>(reinterpret_cast<const void*>(uc_p));
  alt_avg_min_ = *float_p;
  ++float_p;
  ground_alt_min_ = *float_p;
  ++float_p;

  // load matrix
  uc_p = reinterpret_cast<const unsigned char*>(float_p);
  for (unsigned int l = 0; l < resolution_num; ++l) {
    unsigned int rows = matrix->GetRowsSafe(l);
    unsigned int cols = matrix->GetColsSafe(l);
    unsigned int matrix_size = matrix->GetRowsSafe(l) * matrix->GetColsSafe(l);

    if (matrix->HasCount()) {
      binary_size += sizeof(unsigned char) * matrix_size;
      UIntMatrix* count_matrix = matrix->GetCountMatrixSafe(l);
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          DecodeCount(uc_p[r * cols + c], &(*count_matrix)[r][c]);
        }
      }
      uc_p += matrix_size;
    }
    if (matrix->HasGroundCount()) {
      binary_size += sizeof(unsigned char) * matrix_size;
      UIntMatrix* ground_count_matrix = matrix->GetGroundCountMatrixSafe(l);
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          DecodeCount(uc_p[r * cols + c], &(*ground_count_matrix)[r][c]);
        }
      }
      uc_p += matrix_size;
    }

    if (matrix->HasIntensity()) {
      binary_size += sizeof(unsigned char) * matrix_size;
      FloatMatrix* intensity_matrix = matrix->GetIntensityMatrixSafe(l);
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          DecodeIntensity(uc_p[r * cols + c], &(*intensity_matrix)[r][c]);
        }
      }
      uc_p += matrix_size;
    }

    if (matrix->HasIntensityVar()) {
      binary_size += sizeof(uint16_t) * matrix_size;
      FloatMatrix* intensity_var_matrix = matrix->GetIntensityVarMatrixSafe(l);
      const unsigned char* p_low = uc_p + matrix_size;
      const unsigned char* p_high = uc_p;
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          uint16_t var = p_high[r * cols + c];
          var = static_cast<uint16_t>(var * 256 + p_low[r * cols + c]);
          DecodeIntensityVar(var, &(*intensity_var_matrix)[r][c]);
        }
      }
      uc_p += 2 * matrix_size;
    }

    if (matrix->HasAltitude()) {
      binary_size += sizeof(uint16_t) * matrix_size;
      FloatMatrix* altitude_matrix = matrix->GetAltitudeMatrixSafe(l);
      const unsigned char* p_low = uc_p + matrix_size;
      const unsigned char* p_high = uc_p;
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          uint16_t alt = p_high[r * cols + c];
          alt = static_cast<uint16_t>(alt * 256 + p_low[r * cols + c]);
          DecodeAltitude(alt, alt_avg_min_, alt_avg_interval_,
                         &(*altitude_matrix)[r][c]);
        }
      }
      uc_p += 2 * matrix_size;
    }

    if (matrix->HasAltitudeVar()) {
      // skip
      binary_size += sizeof(uint16_t) * matrix_size;
      uc_p += 2 * matrix_size;
    }

    if (matrix->HasGroundAltitude()) {
      binary_size += sizeof(uint16_t) * matrix_size;
      FloatMatrix* ground_altitude_matrix =
          matrix->GetGroundAltitudeMatrixSafe(l);
      const unsigned char* p_low = uc_p + matrix_size;
      const unsigned char* p_high = uc_p;
      for (unsigned int r = 0; r < rows; ++r) {
        for (unsigned int c = 0; c < cols; ++c) {
          uint16_t alt = p_high[r * cols + c];
          alt = static_cast<uint16_t>(alt * 256 + p_low[r * cols + c]);
          DecodeAltitude(alt, ground_alt_min_, ground_alt_interval_,
                         &(*ground_altitude_matrix)[r][c]);
        }
      }
      uc_p += 2 * matrix_size;
    }
  }

  return binary_size;
}

size_t PyramidLossyMapMatrixHandler::CreateBinary(
    const std::shared_ptr<BaseMapMatrix> base_matrix, unsigned char* buf,
    size_t buf_size) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t target_size = GetBinarySize(matrix);
  if (buf_size >= target_size) {
    unsigned int resolution_num = matrix->GetResolutionNum();
    unsigned int ratio = matrix->GetResolutionRatio();
    unsigned int rows = matrix->GetRowsSafe();
    unsigned int cols = matrix->GetColsSafe();

    unsigned int* uint_p =
        reinterpret_cast<unsigned int*>(buf);  // resolution num
    *uint_p = resolution_num;
    ++uint_p;
    *uint_p = ratio;
    ++uint_p;
    *uint_p = rows;  // rows in first level
    ++uint_p;
    *uint_p = cols;  // cols in first level
    ++uint_p;
    // buf_size -= sizeof(unsigned int) * 4;

    // set has_*
    unsigned char* uc_p = reinterpret_cast<unsigned char*>(uint_p);
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    if (matrix->HasIntensity()) {
      *uc_p |= 1;
    }
    if (matrix->HasIntensityVar()) {
      *uc_p |= 2;
    }
    if (matrix->HasAltitude()) {
      *uc_p |= 4;
    }
    if (matrix->HasAltitudeVar()) {
      *uc_p |= 8;
    }
    if (matrix->HasGroundAltitude()) {
      *uc_p |= 16;
    }
    if (matrix->HasCount()) {
      *uc_p |= 32;
    }
    if (matrix->HasGroundCount()) {
      *uc_p |= 64;
    }
    ++uc_p;
    // buf_size -= sizeof(unsigned char) * 4;

    // altitude min
    float* float_p = reinterpret_cast<float*>(reinterpret_cast<void*>(uc_p));
    if (matrix->HasAltitude() && matrix->HasCount()) {
      alt_avg_min_ = 1e8;
      for (unsigned int y = 0; y < rows; ++y) {
        for (unsigned int x = 0; x < cols; ++x) {
          const float* altitude = matrix->GetAltitudeSafe(y, x);
          const unsigned int* count = matrix->GetCountSafe(y, x);
          if (*count == 0) {
            continue;
          }
          if (*altitude < alt_avg_min_) {
            alt_avg_min_ = *altitude;
          }
        }
      }
    } else {
      alt_avg_min_ = 0.0;
    }

    if (matrix->HasGroundAltitude() && matrix->HasGroundCount()) {
      ground_alt_min_ = 1e8;
      for (unsigned int y = 0; y < rows; ++y) {
        for (unsigned int x = 0; x < cols; ++x) {
          const float* ground_altitude = matrix->GetGroundAltitudeSafe(y, x);
          const unsigned int* ground_count = matrix->GetGroundCountSafe(y, x);
          if (*ground_count == 0) {
            continue;
          }
          if (*ground_altitude < ground_alt_min_) {
            ground_alt_min_ = *ground_altitude;
          }
        }
      }
    } else {
      ground_alt_min_ = 0.0;
    }

    *float_p = alt_avg_min_;
    ++float_p;
    *float_p = ground_alt_min_;
    ++float_p;
    // buf_size -= sizeof(float)*2;

    uc_p = reinterpret_cast<unsigned char*>(float_p);
    for (unsigned int l = 0; l < resolution_num; ++l) {
      // unsigned int processed_size = 0;
      unsigned int rows = matrix->GetRowsSafe(l);
      unsigned int cols = matrix->GetColsSafe(l);
      unsigned int matrix_size = rows * cols;

      if (matrix->HasCount()) {
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const unsigned int* count = matrix->GetCountSafe(r, c, l);
            uc_p[r * cols + c] = EncodeCount(*count, count_range_);
          }
        }
        uc_p += matrix_size;
        // processed_size += matrix_size * sizeof(unsigned char);
      }

      if (matrix->HasGroundCount()) {
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const unsigned int* ground_count =
                matrix->GetGroundCountSafe(r, c, l);
            uc_p[r * cols + c] = EncodeCount(*ground_count, count_range_);
          }
        }
        uc_p += matrix_size;
        // processed_size += matrix_size * sizeof(unsigned char);
      }

      if (matrix->HasIntensity()) {
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const float* intensity = matrix->GetIntensitySafe(r, c, l);
            uc_p[r * cols + c] = EncodeIntensity(*intensity);
          }
        }
        uc_p += matrix_size;
        // processed_size += matrix_size * sizeof(unsigned char);
      }

      if (matrix->HasIntensityVar()) {
        unsigned char* p_low = uc_p + matrix_size;
        unsigned char* p_high = uc_p;
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const float* intensity_var = matrix->GetIntensityVarSafe(r, c, l);
            uint16_t var = EncodeIntensityVar(*intensity_var);
            p_high[r * cols + c] = static_cast<unsigned char>(var / 256);
            p_low[r * cols + c] = static_cast<unsigned char>(var % 256);
          }
        }
        uc_p += 2 * matrix_size;
        // processed_size += matrix_size * sizeof(uint16_t);
      }

      if (matrix->HasAltitude()) {
        unsigned char* p_low = uc_p + matrix_size;
        unsigned char* p_high = uc_p;
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const float* altitude = matrix->GetAltitudeSafe(r, c, l);
            uint16_t alt =
                EncodeAltitude(*altitude, alt_avg_min_, alt_avg_interval_);
            p_high[r * cols + c] = static_cast<unsigned char>(alt / 256);
            p_low[r * cols + c] = static_cast<unsigned char>(alt % 256);
          }
        }
        uc_p += 2 * matrix_size;
        // processed_size += matrix_size * sizeof(uint16_t);
      }
      if (matrix->HasAltitudeVar()) {
        // default 0
        memset(uc_p, 0, 2 * matrix_size * sizeof(unsigned char));
        uc_p += 2 * matrix_size;
      }

      if (matrix->HasGroundAltitude()) {
        unsigned char* p_low = uc_p + matrix_size;
        unsigned char* p_high = uc_p;
        for (unsigned int r = 0; r < rows; ++r) {
          for (unsigned int c = 0; c < cols; ++c) {
            const float* ground_altitude =
                matrix->GetGroundAltitudeSafe(r, c, l);
            uint16_t alt = EncodeAltitude(*ground_altitude, ground_alt_min_,
                                          ground_alt_interval_);
            p_high[r * cols + c] = static_cast<unsigned char>(alt / 256);
            p_low[r * cols + c] = static_cast<unsigned char>(alt % 256);
          }
        }
        uc_p += 2 * matrix_size;
        // processed_size += matrix_size * sizeof(uint16_t);
      }

      // assert(buf_size >= processed_size);
      // buf_size -= processed_size;
    }

    return target_size;
  }

  return 0;
}

size_t PyramidLossyMapMatrixHandler::GetBinarySize(
    const std::shared_ptr<BaseMapMatrix> base_matrix) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  unsigned int resolution_num = matrix->GetResolutionNum();
  // assert(resolution_num > 0);
  if (resolution_num == 0) {
    throw "[PyramidLossyMapMatrixHandler::get_binary_size] resolution_num == 0";
  }

  // resolution_num and ratio
  // rows and cols in first level
  // space for has_*
  // altitude min & ground altitude min
  size_t target_size = sizeof(unsigned int) * 2 + sizeof(unsigned int) * 2 +
                       sizeof(unsigned char) * 4 + sizeof(float) * 2;

  for (unsigned int l = 0; l < resolution_num; ++l) {
    unsigned int matrix_size = matrix->GetRowsSafe(l) * matrix->GetColsSafe(l);
    if (matrix->HasCount()) {
      target_size += sizeof(unsigned char) * matrix_size;
    }
    if (matrix->HasGroundCount()) {
      target_size += sizeof(unsigned char) * matrix_size;
    }
    if (matrix->HasIntensity()) {
      target_size += sizeof(unsigned char) * matrix_size;
    }
    if (matrix->HasIntensityVar()) {
      target_size += sizeof(uint16_t) * matrix_size;
    }
    if (matrix->HasAltitude()) {
      target_size += sizeof(uint16_t) * matrix_size;
    }
    if (matrix->HasAltitudeVar()) {
      target_size += sizeof(uint16_t) * matrix_size;
    }
    if (matrix->HasGroundAltitude()) {
      target_size += sizeof(uint16_t) * matrix_size;
    }
  }

  return target_size;
}

// =================PyramidLosslessMapMatrixHandler====================
PyramidLosslessMapMatrixHandler::PyramidLosslessMapMatrixHandler() {}

PyramidLosslessMapMatrixHandler::~PyramidLosslessMapMatrixHandler() {}

size_t PyramidLosslessMapMatrixHandler::LoadBinary(
    const unsigned char* buf, std::shared_ptr<BaseMapMatrix> base_matrix) {
  std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t binary_size = sizeof(unsigned int) * 4;
  const unsigned int* uint_p =
      reinterpret_cast<const unsigned int*>(buf);  // resolution num
  unsigned int resolution_num = *uint_p;
  ++uint_p;
  unsigned int ratio = *uint_p;  // ratio
  ++uint_p;
  unsigned int rows = *uint_p;  // rows in first level
  ++uint_p;
  unsigned int cols = *uint_p;  // cols in first level
  ++uint_p;

  // set has_*
  binary_size += sizeof(unsigned char) * 4;
  const unsigned char* uc_p = reinterpret_cast<const unsigned char*>(uint_p);
  ++uc_p;
  ++uc_p;
  ++uc_p;
  unsigned char has_flag = *uc_p;
  ++uc_p;
  bool has_intensity = (has_flag & 1);
  bool has_intensity_var = (has_flag & 2);
  bool has_altitude = (has_flag & 4);
  bool has_altitude_var = (has_flag & 8);
  bool has_ground_altitude = (has_flag & 16);
  bool has_count = (has_flag & 32);
  bool has_ground_count = (has_flag & 64);

  // reset or init matrix
  if (resolution_num == matrix->GetResolutionNum() &&
      ratio == matrix->GetResolutionRatio() && rows == matrix->GetRowsSafe() &&
      cols == matrix->GetColsSafe() &&
      has_intensity == matrix->HasIntensity() &&
      has_intensity_var == matrix->HasIntensityVar() &&
      has_altitude == matrix->HasAltitude() &&
      has_altitude_var == matrix->HasAltitudeVar() &&
      has_ground_altitude == matrix->HasGroundAltitude() &&
      has_count == matrix->HasCount() &&
      has_ground_count == matrix->HasGroundCount()) {
    matrix->Reset();
  } else {
    matrix->Init(rows, cols, has_intensity, has_intensity_var, has_altitude,
                 has_altitude_var, has_ground_altitude, has_count,
                 has_ground_count, resolution_num, ratio);
  }

  // load matrix
  const float* float_p =
      reinterpret_cast<const float*>(reinterpret_cast<const void*>(uc_p));
  for (unsigned int l = 0; l < resolution_num; ++l) {
    unsigned int matrix_size = matrix->GetRowsSafe(l) * matrix->GetColsSafe(l);
    if (matrix->HasIntensity()) {
      binary_size += sizeof(float) * matrix_size;
      matrix->SetIntensityMatrix(float_p, matrix_size, 0, l);
      float_p += matrix_size;
    }
    if (matrix->HasIntensityVar()) {
      binary_size += sizeof(float) * matrix_size;
      matrix->SetIntensityVarMatrix(float_p, matrix_size, 0, l);
      float_p += matrix_size;
    }
    if (matrix->HasAltitude()) {
      binary_size += sizeof(float) * matrix_size;
      matrix->SetAltitudeMatrix(float_p, matrix_size, 0, l);
      float_p += matrix_size;
    }
    if (matrix->HasAltitudeVar()) {
      binary_size += sizeof(float) * matrix_size;
      matrix->SetAltitudeVarMatrix(float_p, matrix_size, 0, l);
      float_p += matrix_size;
    }
    if (matrix->HasGroundAltitude()) {
      binary_size += sizeof(float) * matrix_size;
      matrix->SetGroundAltitudeMatrix(float_p, matrix_size, 0, l);
      float_p += matrix_size;
    }

    uint_p = reinterpret_cast<const unsigned int*>(
        reinterpret_cast<const void*>(float_p));
    if (matrix->HasCount()) {
      binary_size += sizeof(unsigned int) * matrix_size;
      matrix->SetCountMatrix(uint_p, matrix_size, 0, l);
      uint_p += matrix_size;
    }
    if (matrix->HasGroundCount()) {
      binary_size += sizeof(unsigned int) * matrix_size;
      matrix->SetGroundCountMatrix(uint_p, matrix_size, 0, l);
      uint_p += matrix_size;
    }
    float_p =
        reinterpret_cast<const float*>(reinterpret_cast<const void*>(uint_p));
  }

  return binary_size;
}

size_t PyramidLosslessMapMatrixHandler::CreateBinary(
    const std::shared_ptr<BaseMapMatrix> base_matrix, unsigned char* buf,
    size_t buf_size) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  size_t target_size = GetBinarySize(matrix);
  if (buf_size >= target_size) {
    unsigned int resolution_num = matrix->GetResolutionNum();
    unsigned int ratio = matrix->GetResolutionRatio();
    unsigned int rows = matrix->GetRowsSafe();
    unsigned int cols = matrix->GetColsSafe();

    unsigned int* uint_p =
        reinterpret_cast<unsigned int*>(buf);  // resolution num
    *uint_p = resolution_num;
    ++uint_p;
    *uint_p = ratio;
    ++uint_p;
    *uint_p = rows;  // rows in first level
    ++uint_p;
    *uint_p = cols;  // cols in first level
    ++uint_p;
    // buf_size -= sizeof(unsigned int) * 4;

    // set has_*
    unsigned char* uc_p = reinterpret_cast<unsigned char*>(uint_p);
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    ++uc_p;
    *uc_p = 0;
    if (matrix->HasIntensity()) {
      *uc_p |= 1;
    }
    if (matrix->HasIntensityVar()) {
      *uc_p |= 2;
    }
    if (matrix->HasAltitude()) {
      *uc_p |= 4;
    }
    if (matrix->HasAltitudeVar()) {
      *uc_p |= 8;
    }
    if (matrix->HasGroundAltitude()) {
      *uc_p |= 16;
    }
    if (matrix->HasCount()) {
      *uc_p |= 32;
    }
    if (matrix->HasGroundCount()) {
      *uc_p |= 64;
    }
    ++uc_p;
    // buf_size -= sizeof(unsigned char) * 4;

    float* float_p = reinterpret_cast<float*>(reinterpret_cast<void*>(uc_p));
    for (unsigned int l = 0; l < resolution_num; ++l) {
      // unsigned int processed_size = 0;
      unsigned int matrix_size =
          matrix->GetRowsSafe(l) * matrix->GetColsSafe(l);
      if (matrix->HasIntensity()) {
        const FloatMatrix* intensity_matrix = matrix->GetIntensityMatrixSafe(l);
        memcpy(float_p, (*intensity_matrix)[0], matrix_size * sizeof(float));
        float_p += matrix_size;
        // processed_size += matrix_size * sizeof(float);
      }
      if (matrix->HasIntensityVar()) {
        const FloatMatrix* intensity_var_matrix =
            matrix->GetIntensityVarMatrixSafe(l);
        memcpy(float_p, (*intensity_var_matrix)[0],
               matrix_size * sizeof(float));
        float_p += matrix_size;
        // processed_size += matrix_size * sizeof(float);
      }
      if (matrix->HasAltitude()) {
        const FloatMatrix* altitude_matrix = matrix->GetAltitudeMatrixSafe(l);
        memcpy(float_p, (*altitude_matrix)[0], matrix_size * sizeof(float));
        float_p += matrix_size;
        // processed_size += matrix_size * sizeof(float);
      }
      if (matrix->HasAltitudeVar()) {
        const FloatMatrix* altitude_var_matrix =
            matrix->GetAltitudeVarMatrixSafe(l);
        memcpy(float_p, (*altitude_var_matrix)[0], matrix_size * sizeof(float));
        float_p += matrix_size;
        // processed_size += matrix_size * sizeof(float);
      }
      if (matrix->HasGroundAltitude()) {
        const FloatMatrix* ground_altitude_matrix =
            matrix->GetGroundAltitudeMatrixSafe(l);
        memcpy(float_p, (*ground_altitude_matrix)[0],
               matrix_size * sizeof(float));
        float_p += matrix_size;
        // processed_size += matrix_size * sizeof(float);
      }

      uint_p =
          reinterpret_cast<unsigned int*>(reinterpret_cast<void*>(float_p));
      if (matrix->HasCount()) {
        const UIntMatrix* count_matrix = matrix->GetCountMatrixSafe(l);
        memcpy(uint_p, (*count_matrix)[0], matrix_size * sizeof(unsigned int));
        uint_p += matrix_size;
        // processed_size += matrix_size * sizeof(unsigned int);
      }
      if (matrix->HasGroundCount()) {
        const UIntMatrix* ground_count_matrix =
            matrix->GetGroundCountMatrixSafe(l);
        memcpy(uint_p, (*ground_count_matrix)[0],
               matrix_size * sizeof(unsigned int));
        uint_p += matrix_size;
        // processed_size += matrix_size * sizeof(unsigned int);
      }
      float_p = reinterpret_cast<float*>(reinterpret_cast<void*>(uint_p));
      // assert(buf_size >= processed_size);
      // buf_size -= processed_size;
    }

    return target_size;
  }

  return 0;
}

size_t PyramidLosslessMapMatrixHandler::GetBinarySize(
    const std::shared_ptr<BaseMapMatrix> base_matrix) {
  const std::shared_ptr<PyramidMapMatrix> matrix =
      std::dynamic_pointer_cast<PyramidMapMatrix>(base_matrix);

  unsigned int resolution_num = matrix->GetResolutionNum();
  // assert(resolution_num > 0);
  if (resolution_num == 0) {
    throw "[PyramidLosslessMapMatrixHandler::get_binary_size]"
        "resolution_num == 0";
  }

  // resolution_num and ratio
  // rows and cols in first level
  // space for has_*
  size_t target_size = sizeof(unsigned int) * 2 + sizeof(unsigned int) * 2 +
                       sizeof(unsigned char) * 4;

  for (unsigned int l = 0; l < resolution_num; ++l) {
    unsigned int matrix_size = matrix->GetRowsSafe(l) * matrix->GetColsSafe(l);
    if (matrix->HasIntensity()) {
      target_size += sizeof(float) * matrix_size;
    }
    if (matrix->HasIntensityVar()) {
      target_size += sizeof(float) * matrix_size;
    }
    if (matrix->HasAltitude()) {
      target_size += sizeof(float) * matrix_size;
    }
    if (matrix->HasAltitudeVar()) {
      target_size += sizeof(float) * matrix_size;
    }
    if (matrix->HasGroundAltitude()) {
      target_size += sizeof(float) * matrix_size;
    }
    if (matrix->HasCount()) {
      target_size += sizeof(unsigned int) * matrix_size;
    }
    if (matrix->HasGroundCount()) {
      target_size += sizeof(unsigned int) * matrix_size;
    }
  }

  return target_size;
}

}  // namespace pyramid_map
}  // namespace msf
}  // namespace localization
}  // namespace apollo
