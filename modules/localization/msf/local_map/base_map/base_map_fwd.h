#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_FWD_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_FWD_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace apollo {
namespace localization {
namespace msf {

/**@brief The options of the reflectance map. */
class BaseMapConfig;

/**@brief The data structure of the base map. */
class BaseMap;

/**@brief The data structure of the map cells in a map node. */
class BaseMapMatrix;

/**@brief The data structure of a Node in the map. */
class BaseMapNode;

class MapNodeIndex;

/**@brief The memory pool for the data structure of BaseMapNode. */
class BaseMapNodePool;

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_FWD_H
