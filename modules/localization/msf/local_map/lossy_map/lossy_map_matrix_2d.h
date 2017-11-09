#ifndef BAIDU_ADU_LOSSY_MAP_MATRIX_FULL_ALT_2D_H
#define BAIDU_ADU_LOSSY_MAP_MATRIX_FULL_ALT_2D_H

#include <vector>
#include "modules/localization/msf/local_map/base_map/base_map_matrix.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"

namespace apollo {
namespace localization {
namespace msf {

struct LossyMapCell2D {
  /**@brief The default constructor. */
  LossyMapCell2D();
  /**@brief Reset to default value. */
  inline void reset();
  // /**@brief Load the map cell from a binary chunk.
  //  * @param <return> The size read (the real size of object).
  //  */
  // inline unsigned int load_binary(unsigned char * buf);
  // /**@brief Create the binary. Serialization of the object.
  //  * @param <buf, buf_size> The buffer and its size.
  //  * @param <return> The required or the used size of is returned.
  //  */
  // inline unsigned int create_binary(unsigned char * buf, unsigned int
  // buf_size) const;
  // /**@brief Get the binary size of the object. */
  // inline unsigned int get_binary_size() const;
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

  virtual void init(const BaseMapConfig* config);
  /**@brief Reset map cells data. */
  virtual void reset(const BaseMapConfig* config);

  void init(unsigned int rows, unsigned int cols);
  void reset(unsigned int rows, unsigned int cols);

  /**@brief Load the map cell from a binary chunk.
   * @param <return> The size read (the real size of object).
   */
  virtual unsigned int load_binary(unsigned char* buf);
  /**@brief Create the binary. Serialization of the object.
   * @param <buf, buf_size> The buffer and its size.
   * @param <return> The required or the used size of is returned.
   */
  virtual unsigned int create_binary(unsigned char* buf,
                                     unsigned int buf_size) const;
  /**@brief Get the binary size of the object. */
  virtual unsigned int get_binary_size() const;
  /**@brief get intensity image of node. */
  virtual void get_intensity_img(cv::Mat& intensity_img) const;

  inline LossyMapCell2D* operator[](int row) {
    return _map_cells + row * _cols;
  }
  inline const LossyMapCell2D* operator[](int row) const {
    return _map_cells + row * _cols;
  }

 protected:
  /**@brief The number of rows. */
  unsigned int _rows;
  /**@brief The number of columns. */
  unsigned int _cols;
  /**@brief The matrix data structure. */
  LossyMapCell2D* _map_cells;

 protected:
  inline unsigned char encode_intensity(const LossyMapCell2D& cell) const;
  inline void decode_intensity(unsigned char data, LossyMapCell2D& cell) const;
  inline unsigned short encode_var(const LossyMapCell2D& cell) const;
  inline void decode_var(unsigned short data, LossyMapCell2D& cell) const;
  inline unsigned short encode_altitude_ground(
      const LossyMapCell2D& cell) const;
  inline void decode_altitude_ground(unsigned short data,
                                     LossyMapCell2D& cell) const;
  inline unsigned short encode_altitude_avg(const LossyMapCell2D& cell) const;
  inline void decode_altitude_avg(unsigned short data,
                                  LossyMapCell2D& cell) const;
  inline unsigned char encode_count(const LossyMapCell2D& cell) const;
  inline void decode_count(unsigned char data, LossyMapCell2D& cell) const;
  const int _var_range = 1023;  // 65535;
  const int _var_ratio = 4;     // 256;
  // const unsigned int _alt_range = 1023;//65535;
  const float _alt_ground_interval = 0.04;
  const unsigned short _ground_void_flag = 0xffff;
  const float _alt_avg_interval = 0.04;
  const int _count_range = 2;  // 30;
  mutable float _alt_avg_min;
  mutable float _alt_avg_max;
  mutable float _alt_ground_min;
  mutable float _alt_ground_max;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOSSY_MAP_MATRIX_FULL_ALT_2D_H
