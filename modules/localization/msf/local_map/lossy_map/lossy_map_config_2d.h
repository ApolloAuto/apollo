#ifndef BAIDU_ADU_LOSSY_MAP_CONFIG_2D_H
#define BAIDU_ADU_LOSSY_MAP_CONFIG_2D_H

#include "modules/localization/msf/local_map/base_map/base_map_config.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class LossyMapConfig2D : public BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit LossyMapConfig2D(std::string map_version = "lossy_map");
  ~LossyMapConfig2D() {}

  /**@brief The threshold to split more layers in the map node. */
  float _map_layer_alt_thres;
  /**@brief When load map nodes, the maximum number of map nodes will be kept in
   * memory. */
  unsigned int _map_cache_size;

  /**@brief During the visualization (for example, call the function get_image()
   * of map node layer), the maximum intensity value in the image. */
  float _max_intensity_value;
  /**@brief During the visualization (for example, call the function get_image()
   * of map node layer), the maximum intensity variance value in the image. */
  float _max_intensity_var_value;

 protected:
  /**@brief Create the XML structure. */
  virtual void create_xml(boost::property_tree::ptree& config) const;
  /**@brief Load the map options from a XML structure. */
  virtual void load_xml(boost::property_tree::ptree& config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOSSY_MAP_CONFIG_2D_H
