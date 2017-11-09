#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CONFIG_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CONFIG_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "modules/localization/msf/common/util/rect2d.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class BaseMapConfig {
 public:
  /**@brief The constructor gives the default map settings. */
  explicit BaseMapConfig(std::string map_version = "0.1");
  /**@brief Save the map option to a XML file. */
  bool save(const std::string file_path);
  /**@brief Load the map option from a XML file. */
  bool load(const std::string file_path);
  /**@brief Resize map range by range and resolutions. */
  void resize_map_range();
  /**@brief Set single resolutions. */
  void set_single_resolutions(float resolution = 0.125);
  /**@brief Set multi resolutions. */
  void set_multi_resolutions();

  /**@brief The version of map. */
  std::string _map_version;
  /**@brief The pixel resolutions in the map in meters. */
  std::vector<float> _map_resolutions;
  /**@brief The map node size in pixels. */
  unsigned int _map_node_size_x;
  /**@brief The map node size in pixels. */
  unsigned int _map_node_size_y;
  /**@brief The minimum and maximum UTM range in the map.
   *
   * The x direction is the easting in UTM coordinate.
   * The y direction is the northing in UTM coordinate.
   */
  Rect2D<double> _map_range;

  /**@brief Velodyne's height to the ground. Estimate the Velodyne's height
   * based on the ground height. */
  float _map_ground_height_offset;
  /**@brief Enable the compression. */
  bool _map_is_compression;

  /**@brief The map folder path. */
  std::string _map_folder_path;
  /**@brief The datasets that contributed to the map. */
  std::vector<std::string> _map_datasets;

 protected:
  /**@brief Create the XML structure. */
  virtual void create_xml(boost::property_tree::ptree& config) const;
  /**@brief Load the map options from a XML structure. */
  virtual void load_xml(boost::property_tree::ptree& config);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_CONFIG_H
