#include "modules/localization/msf/local_map/lossy_map/lossy_map_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMap2D::LossyMap2D(LossyMapConfig2D& config) : BaseMap(config) {}

LossyMap2D::~LossyMap2D() {}

void LossyMap2D::preload_map_area(const Eigen::Vector3d& location,
                                  const Eigen::Vector3d& trans_diff,
                                  unsigned int resolution_id,
                                  unsigned int zone_id) {
  BaseMap::preload_map_area(location, trans_diff, resolution_id, zone_id);
  return;
}

bool LossyMap2D::load_map_area(const Eigen::Vector3d& seed_pt3d,
                               unsigned int resolution_id, unsigned int zone_id,
                               int filter_size_x, int filter_size_y) {
  BaseMap::load_map_area(seed_pt3d, resolution_id, zone_id, filter_size_x,
                         filter_size_y);
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
