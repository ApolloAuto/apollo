#ifndef BAIDU_ADU_LOSSLESS_MAP_NODE_H
#define BAIDU_ADU_LOSSLESS_MAP_NODE_H

#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

class LosslessMapNode : public BaseMapNode {
 public:
  LosslessMapNode();
  ~LosslessMapNode();

  /**@brief Set the value of a pixel in the map node.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   */
  void set_value(const Eigen::Vector3d& coordinate, unsigned char intensity);
  /**@brief Set the value of a pixel in the map node if the pixel in the node.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   * @param <return> True, if pixel in the bound of the node, else False.
   * */
  bool set_value_if_in_bound(const Eigen::Vector3d& coordinate,
                             unsigned char intensity);
  /**@brief Set the vlaue of a pixel in a layer in the map node.
   * @param <coordinate> The 3D global coordinate. The z is used as the altitude
   * for the layer match.
   * @param <intensity> The reflectance intensity.
   */
  void set_value_layer(const Eigen::Vector3d& coordinate,
                       unsigned char intensity);
  /**@brief Given the 3D global coordinate, get the map cell average intensity
   * of each layer. */
  void get_value(const Eigen::Vector3d& coordinate,
                 std::vector<unsigned char>& values) const;
  /**@brief Given the 3D global coordinate, get the map cell variance of the
   * intensity of each layer. */
  void get_var(const Eigen::Vector3d& coordinate,
               std::vector<float>& vars) const;
  /**@brief Given the 3D global coordinate, get the map cell's average altitude
   * of each layer. */
  void get_alt(const Eigen::Vector3d& coordinate,
               std::vector<float>& alts) const;
  /**@brief Given the 3D global coordinate, get the map cell's variance of the
   * altitude of each layer. */
  void get_alt_var(const Eigen::Vector3d& coordinate,
                   std::vector<float>& alt_vars) const;
  /**@brief Given the 3D global coordinate, get the map cell's count of the
   * samples of each layer. */
  void get_count(const Eigen::Vector3d& coordinate,
                 std::vector<unsigned int>& counts) const;
  /**@brief Given the 3D global coordinate, get the map cell average intensity.
   */
  unsigned char get_value(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell variance of the
   * intensity. */
  float get_var(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's average altitude.
   */
  float get_alt(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's variance of the
   * altitude */
  float get_alt_var(const Eigen::Vector3d& coordinate) const;
  /**@brief Given the 3D global coordinate, get the map cell's count of the
   * samples. */
  unsigned int get_count(const Eigen::Vector3d& coordinate) const;
  /**@brief Get the map cell average intensity. */
  unsigned char get_value(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        .get_value();
  }
  /**@brief Get the map cell variance of the intensity. */
  float get_var(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        .get_var();
  }
  /**@brief Get the map cell's average altitude. */
  float get_alt(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        .get_alt();
  }
  /**@brief Get the map cell's variance of the altitude */
  float get_alt_var(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        .get_alt_var();
  }
  /**@brief Get the map cell's count of the samples. */
  unsigned int get_count(unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        .get_count();
  }
  /**@brief Get the constant map cell given the coordinate. */
  inline const LosslessMapSingleCell& get_first_map_cell(
      unsigned int row, unsigned int col) const {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        ._map_cells[0];
  }
  inline LosslessMapSingleCell& get_first_map_cell(unsigned int row,
                                                   unsigned int col) {
    return static_cast<LosslessMapMatrix*>(_map_matrix)
        ->get_map_cell(row, col)
        ._map_cells[0];
  }

  /**@brief Get the min altitude of point cloud in the node. */
  inline float get_min_altitude() const {
    return _min_altitude;
  }
  /**@brief Set the min altitude of point cloud in the node. */
  inline void set_min_altitude(float altitude) {
    _min_altitude = altitude;
  }

 protected:
  /**@brief The min altitude of point cloud in the node. */
  float _min_altitude;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOSSLESS_MAP_NODE_H
