#ifndef BAIDU_ADU_LOCALIZATION_LOSSLESS_MAP_H
#define BAIDU_ADU_LOCALIZATION_LOSSLESS_MAP_H

#include "modules/localization/msf/local_map/base_map/base_map.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

class LosslessMap : public BaseMap {
 public:
  explicit LosslessMap(LosslessMapConfig& config);
  ~LosslessMap();

 public:
  /**@brief Set the value of a pixel in the map, including all the resolution
   * levels.
   * @param <coordinate> The 3D global coordinate.
   * @param <intensity> The reflectance intensity.
   */
  void set_value(const Eigen::Vector3d& coordinate, int zone_id,
                 unsigned char intensity);
  /**@brief Set the vlaue of a pixel in a layer in the map node.
   * @param <coordinate> The 3D global coordinate. The z is used as the altitude
   * for the layer match.
   * @param <intensity> The reflectance intensity.
   */
  void set_value_layer(const Eigen::Vector3d& coordinate, int zone_id,
                       unsigned char intensity);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary.
   * This function returns the value of each layer in the map cell. */
  void get_value(const Eigen::Vector3d& coordinate, int zone_id,
                 unsigned int resolution_id,
                 std::vector<unsigned char>& values);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. This function returns the value of each layer in the map cell.
   */
  void get_value_safe(const Eigen::Vector3d& coordinate, int zone_id,
                      unsigned int resolution_id,
                      std::vector<unsigned char>& values);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary.
   * This function returns the value of each layer in the map cell. */
  void get_var(const Eigen::Vector3d& coordinate, int zone_id,
               unsigned int resolution_id, std::vector<float>& vars);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. This function returns the value of each layer in the map cell.
   */
  void get_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                    unsigned int resolution_id, std::vector<float>& vars);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary.
   * This function returns the value of each layer in the map cell. */
  void get_alt(const Eigen::Vector3d& coordinate, int zone_id,
               unsigned int resolution_id, std::vector<float>& alts);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. This function returns the value of each layer in the map cell.
   */
  void get_alt_safe(const Eigen::Vector3d& coordinate, int zone_id,
                    unsigned int resolution_id, std::vector<float>& alts);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary.
   * This function returns the value of each layer in the map cell. */
  void get_alt_var(const Eigen::Vector3d& coordinate, int zone_id,
                   unsigned int resolution_id, std::vector<float>& alt_vars);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. This function returns the value of each layer in the map cell.
   */
  void get_alt_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                        unsigned int resolution_id,
                        std::vector<float>& alt_vars);
  /**@brief Get the number of samples in the map cell. */
  void get_count(const Eigen::Vector3d& coordinate, int zone_id,
                 unsigned int resolution_id, std::vector<unsigned int>& counts);
  /**@brief Get the number of samples in the map cell thread safely. */
  void get_count_safe(const Eigen::Vector3d& coordinate, int zone_id,
                      unsigned int resolution_id,
                      std::vector<unsigned int>& counts);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary. */
  unsigned char get_value(const Eigen::Vector3d& coordinate, int zone_id,
                          unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. */
  unsigned char get_value_safe(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary. */
  float get_var(const Eigen::Vector3d& coordinate, int zone_id,
                unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. */
  float get_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                     unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary. */
  float get_alt(const Eigen::Vector3d& coordinate, int zone_id,
                unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. */
  float get_alt_safe(const Eigen::Vector3d& coordinate, int zone_id,
                     unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache and return the value, if necessary. */
  float get_alt_var(const Eigen::Vector3d& coordinate, int zone_id,
                    unsigned int resolution_id);
  /**@brief Given the 3D global coordinate, this function loads the
   * corresponding map node in the cache thread safely and return the value, if
   * necessary. */
  float get_alt_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                         unsigned int resolution_id);
  /**@brief Get the number of samples in the map cell. */
  unsigned int get_count(const Eigen::Vector3d& coordinate, int zone_id,
                         unsigned int resolution_id);
  /**@brief Get the number of samples in the map cell thread safely. */
  unsigned int get_count_safe(const Eigen::Vector3d& coordinate, int zone_id,
                              unsigned int resolution_id);
  /**@brief Set the ground height offset. */
  void set_ground_height_offset(double height_offset);
  /**@brief Preload map nodes for the next frame location calculation.
   * It will forecasts the nodes by the direction of the car moving.
   * Because the progress of loading will cost a long time (over 100ms),
   * it must do this for a period of time in advance.
   * After the index of nodes calculate finished, it will create loading tasks,
   * but will not wait for the loading finished, eigen version. */
  virtual void preload_map_area(const Eigen::Vector3d& location,
                                const Eigen::Vector3d& trans_diff,
                                unsigned int resolution_id,
                                unsigned int zone_id);
  /**@brief Load map nodes for the location calculate of this frame.
   * If the forecasts are correct in last frame, these nodes will be all in
   * cache, if not, then need to create loading tasks, and wait for the loading
   * finish, in order to the nodes which the following calculate needed are all
   * in the memory, eigen version. */
  virtual bool load_map_area(const Eigen::Vector3d& seed_pt3d,
                             unsigned int resolution_id, unsigned int zone_id,
                             int filter_size_x, int filter_size_y);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOCALIZATION_LOSSLESS_MAP_H
