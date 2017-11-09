#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_config.h"

namespace apollo {
namespace localization {
namespace msf {
LosslessMapNode::LosslessMapNode()
    : BaseMapNode(new LosslessMapMatrix(), new ZlibStrategy()) {}

LosslessMapNode::~LosslessMapNode() {}

void LosslessMapNode::set_value(const Eigen::Vector3d& coordinate,
                                unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.set_value(coordinate[2], intensity);
  if (_min_altitude > coordinate[2]) {
    _min_altitude = coordinate[2];
  }
  _is_changed = true;
}

bool LosslessMapNode::set_value_if_in_bound(const Eigen::Vector3d& coordinate,
                                            unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  if (is_success) {
    LosslessMapCell& map_cell =
        static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
    map_cell.set_value(coordinate[2], intensity);
    if (_min_altitude > coordinate[2]) {
      _min_altitude = coordinate[2];
    }
    _is_changed = true;
    return true;
  } else {
    return false;
  }
}

void LosslessMapNode::set_value_layer(const Eigen::Vector3d& coordinate,
                                      unsigned char intensity) {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.set_value_layer(
      coordinate[2], intensity,
      static_cast<const LosslessMapConfig*>(_map_config)->_map_layer_alt_thres);
  _is_changed = true;
}

void LosslessMapNode::get_value(const Eigen::Vector3d& coordinate,
                                std::vector<unsigned char>& values) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.get_value(values);
}

void LosslessMapNode::get_var(const Eigen::Vector3d& coordinate,
                              std::vector<float>& vars) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.get_var(vars);
}

void LosslessMapNode::get_alt(const Eigen::Vector3d& coordinate,
                              std::vector<float>& alts) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.get_alt(alts);
}

void LosslessMapNode::get_alt_var(const Eigen::Vector3d& coordinate,
                                  std::vector<float>& alt_vars) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.get_alt_var(alt_vars);
}

void LosslessMapNode::get_count(const Eigen::Vector3d& coordinate,
                                std::vector<unsigned int>& counts) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  map_cell.get_count(counts);
}

unsigned char LosslessMapNode::get_value(
    const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  return map_cell.get_value();
}

float LosslessMapNode::get_var(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  return map_cell.get_var();
}

float LosslessMapNode::get_alt(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  return map_cell.get_alt();
}

float LosslessMapNode::get_alt_var(const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  return map_cell.get_alt_var();
}

unsigned int LosslessMapNode::get_count(
    const Eigen::Vector3d& coordinate) const {
  Eigen::Vector2d coord2d(coordinate[0], coordinate[1]);
  unsigned int x = 0;
  unsigned int y = 0;
  bool is_success = get_coordinate(coord2d, x, y);
  assert(is_success);
  LosslessMapCell& map_cell =
      static_cast<LosslessMapMatrix*>(_map_matrix)->get_map_cell(y, x);
  return map_cell.get_count();
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
