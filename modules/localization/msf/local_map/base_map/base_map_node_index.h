#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_INDEX_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_INDEX_H

#include <iostream>
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"

namespace apollo {
namespace localization {
namespace msf {

class MapNodeIndex;

std::ostream& operator<<(std::ostream& cout, const MapNodeIndex& index);

class MapNodeIndex {
 public:
  /**@brief The constructor. */
  MapNodeIndex();
  /**@brief Overload the less than operator. */
  bool operator<(const MapNodeIndex& index) const;
  /**@brief Overload the equal operator. */
  bool operator==(const MapNodeIndex& index) const;
  /**@brief Overload the unequal operator. */
  bool operator!=(const MapNodeIndex& index) const;
  std::string to_string() const;

  // /**@brief Construct a map node index, given a global coordinate. */
  // static MapNodeIndex get_map_node_index(const BaseMapConfig& option,
  //                                        const
  //                                        idl::car::core::numerical::Vector3D&
  //                                        coordinate, unsigned int
  //                                        resolution_id, int zone_id);
  // static MapNodeIndex get_map_node_index(const BaseMapConfig& option,
  //                                        const
  //                                        idl::car::core::numerical::Vector2D&
  //                                        coordinate, unsigned int
  //                                        resolution_id, int zone_id);

  /**@brief Construct a map node index, given a global coordinate, eigen
   * version. */
  static MapNodeIndex get_map_node_index(const BaseMapConfig& option,
                                         const Eigen::Vector3d& coordinate,
                                         unsigned int resolution_id,
                                         int zone_id);
  static MapNodeIndex get_map_node_index(const BaseMapConfig& option,
                                         const Eigen::Vector2d& coordinate,
                                         unsigned int resolution_id,
                                         int zone_id);

  /**@brief Get the index range (maximum possible index + 1) in the east
   * direction. */
  static unsigned int get_map_index_range_east(const BaseMapConfig& option,
                                               unsigned int resolution_id);
  /**@brief Get the index range (maximum possible index + 1) in the north
   * direction. */
  static unsigned int get_map_index_range_north(const BaseMapConfig& option,
                                                unsigned int resolution_id);

  friend std::ostream& operator<<(std::ostream& cout,
                                  const MapNodeIndex& index);

  /**@brief The ID of the resolution.
   * Should be less than BaseMapConfig::_map_resolutions.size(). */
  unsigned int _resolution_id;
  /**@brief The zone ID. 1 - 60 and -1 - -60.
   * The positive value is the zone at the north hemisphere. */
  int _zone_id;
  /**@brief The map node ID at the northing direction. */
  unsigned int _m;
  /**@brief The map node ID at the easting direction. */
  unsigned int _n;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_NODE_INDEX_H
