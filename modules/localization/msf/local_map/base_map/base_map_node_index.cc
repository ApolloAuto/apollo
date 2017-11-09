#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include <sstream>

namespace apollo {
namespace localization {
namespace msf {

MapNodeIndex::MapNodeIndex() {
  _resolution_id = 0;
  _zone_id = 50;
  _m = 0;
  _n = 0;
}

bool MapNodeIndex::operator<(const MapNodeIndex& index) const {
  if (_resolution_id < index._resolution_id) {
    return true;
  } else if (_resolution_id == index._resolution_id) {
    if (_zone_id < index._zone_id) {
      return true;
    } else if (_zone_id == index._zone_id) {
      if (_m < index._m) {
        return true;
      } else if (_m == index._m) {
        if (_n < index._n) {
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool MapNodeIndex::operator==(const MapNodeIndex& index) const {
  return _resolution_id == index._resolution_id && _zone_id == index._zone_id &&
         _m == index._m && _n == index._n;
}

bool MapNodeIndex::operator!=(const MapNodeIndex& index) const {
  return !(*this == index);
}

std::string MapNodeIndex::to_string() const {
  std::ostringstream ss;
  ss << "Map node (Resolution ID: " << _resolution_id
     << " Zone ID: " << _zone_id << " Easting: " << _n << " Northing: " << _m
     << ")";
  return ss.str();
}

// MapNodeIndex MapNodeIndex::get_map_node_index(const BaseMapConfig& option,
//         const idl::car::core::numerical::Vector3D& coordinate,
//         unsigned int resolution_id, int zone_id) {
//     idl::car::core::numerical::Vector2D coord2d;
//     coord2d.init(coordinate.get_data());
//     return get_map_node_index(option, coord2d, resolution_id, zone_id);
// }

// MapNodeIndex MapNodeIndex::get_map_node_index(const BaseMapConfig& option,
//         const idl::car::core::numerical::Vector2D& coordinate,
//         unsigned int resolution_id, int zone_id) {
//     assert(resolution_id < option._map_resolutions.size());
//     MapNodeIndex index;
//     index._resolution_id = resolution_id;
//     index._zone_id = zone_id;
//     int n = static_cast<int>((coordinate[0] - option._map_range.get_min_x())
//     /
//             (option._map_node_size_x *
//             option._map_resolutions[resolution_id]));
//     int m = static_cast<int>((coordinate[1] - option._map_range.get_min_y())
//     /
//             (option._map_node_size_y *
//             option._map_resolutions[resolution_id]));
//     if (n >= 0 && m >= 0 && n < get_map_index_range_east(option,
//     resolution_id) &&
//             m < get_map_index_range_north(option, resolution_id)) {
//         index._m = m;
//         index._n = n;
//     }
//     else {
//         assert(0 == 1); // should never reach here
//     }
//     return index;
// }

MapNodeIndex MapNodeIndex::get_map_node_index(const BaseMapConfig& option,
                                              const Eigen::Vector3d& coordinate,
                                              unsigned int resolution_id,
                                              int zone_id) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  return get_map_node_index(option, coord2d, resolution_id, zone_id);
}

MapNodeIndex MapNodeIndex::get_map_node_index(const BaseMapConfig& option,
                                              const Eigen::Vector2d& coordinate,
                                              unsigned int resolution_id,
                                              int zone_id) {
  assert(resolution_id < option._map_resolutions.size());
  MapNodeIndex index;
  index._resolution_id = resolution_id;
  index._zone_id = zone_id;
  int n = static_cast<int>(
      (coordinate[0] - option._map_range.GetMinX()) /
      (option._map_node_size_x * option._map_resolutions[resolution_id]));
  int m = static_cast<int>(
      (coordinate[1] - option._map_range.GetMinY()) /
      (option._map_node_size_y * option._map_resolutions[resolution_id]));
  if (n >= 0 && m >= 0 &&
      n < int(get_map_index_range_east(option, resolution_id)) &&
      m < int(get_map_index_range_north(option, resolution_id))) {
    index._m = m;
    index._n = n;
  } else {
    assert(0 == 1);  // should never reach here
  }
  return index;
}

unsigned int MapNodeIndex::get_map_index_range_east(
    const BaseMapConfig& option, unsigned int resolution_id) {
  return static_cast<unsigned int>(
      (option._map_range.GetMaxX() - option._map_range.GetMinX()) /
      (option._map_node_size_x * option._map_resolutions[resolution_id]));
}

unsigned int MapNodeIndex::get_map_index_range_north(
    const BaseMapConfig& option, unsigned int resolution_id) {
  return static_cast<unsigned int>(
      (option._map_range.GetMaxY() - option._map_range.GetMinY()) /
      (option._map_node_size_y * option._map_resolutions[resolution_id]));
}

std::ostream& operator<<(std::ostream& cerr, const MapNodeIndex& index) {
  cerr << index.to_string();
  return cerr;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
