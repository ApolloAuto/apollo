#include "modules/localization/msf/local_map/lossless_map/lossless_map.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"

namespace apollo {
namespace localization {
namespace msf {
LosslessMap::LosslessMap(LosslessMapConfig& config) : BaseMap(config) {}

LosslessMap::~LosslessMap() {}

void LosslessMap::set_value(const Eigen::Vector3d& coordinate, int zone_id,
                            unsigned char intensity) {
  for (size_t i = 0; i < _map_config._map_resolutions.size(); ++i) {
    MapNodeIndex index =
        MapNodeIndex::get_map_node_index(_map_config, coordinate, i, zone_id);
    LosslessMapNode* node =
        static_cast<LosslessMapNode*>(get_map_node_safe(index));
    node->set_value(coordinate, intensity);
  }
}

void LosslessMap::set_value_layer(const Eigen::Vector3d& coordinate,
                                  int zone_id, unsigned char intensity) {
  for (size_t i = 0; i < _map_config._map_resolutions.size(); ++i) {
    MapNodeIndex index =
        MapNodeIndex::get_map_node_index(_map_config, coordinate, i, zone_id);
    LosslessMapNode* node =
        static_cast<LosslessMapNode*>(get_map_node_safe(index));
    node->set_value_layer(coordinate, intensity);
  }
}

void LosslessMap::get_value(const Eigen::Vector3d& coordinate, int zone_id,
                            unsigned int resolution_id,
                            std::vector<unsigned char>& values) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  node->get_value(coordinate, values);
}

void LosslessMap::get_value_safe(const Eigen::Vector3d& coordinate, int zone_id,
                                 unsigned int resolution_id,
                                 std::vector<unsigned char>& values) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  ;
  node->get_value(coordinate, values);
}

void LosslessMap::get_var(const Eigen::Vector3d& coordinate, int zone_id,
                          unsigned int resolution_id,
                          std::vector<float>& vars) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  node->get_var(coordinate, vars);
}

void LosslessMap::get_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id,
                               std::vector<float>& vars) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  node->get_var(coordinate, vars);
}

void LosslessMap::get_alt(const Eigen::Vector3d& coordinate, int zone_id,
                          unsigned int resolution_id,
                          std::vector<float>& alts) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  node->get_alt(coordinate, alts);
}

void LosslessMap::get_alt_safe(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id,
                               std::vector<float>& alts) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  node->get_alt(coordinate, alts);
}

void LosslessMap::get_alt_var(const Eigen::Vector3d& coordinate, int zone_id,
                              unsigned int resolution_id,
                              std::vector<float>& alt_vars) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  node->get_alt_var(coordinate, alt_vars);
}

void LosslessMap::get_alt_var_safe(const Eigen::Vector3d& coordinate,
                                   int zone_id, unsigned int resolution_id,
                                   std::vector<float>& alt_vars) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  node->get_alt_var(coordinate, alt_vars);
}

void LosslessMap::get_count(const Eigen::Vector3d& coordinate, int zone_id,
                            unsigned int resolution_id,
                            std::vector<unsigned int>& counts) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  node->get_count(coordinate, counts);
}

void LosslessMap::get_count_safe(const Eigen::Vector3d& coordinate, int zone_id,
                                 unsigned int resolution_id,
                                 std::vector<unsigned int>& counts) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  node->get_count(coordinate, counts);
}

unsigned char LosslessMap::get_value(const Eigen::Vector3d& coordinate,
                                     int zone_id, unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  return node->get_value(coordinate);
}

unsigned char LosslessMap::get_value_safe(const Eigen::Vector3d& coordinate,
                                          int zone_id,
                                          unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  return node->get_value(coordinate);
}

float LosslessMap::get_var(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  return node->get_var(coordinate);
}

float LosslessMap::get_var_safe(const Eigen::Vector3d& coordinate, int zone_id,
                                unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  return node->get_var(coordinate);
}

float LosslessMap::get_alt(const Eigen::Vector3d& coordinate, int zone_id,
                           unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  return node->get_alt(coordinate);
}

float LosslessMap::get_alt_safe(const Eigen::Vector3d& coordinate, int zone_id,
                                unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  return node->get_alt(coordinate);
}

float LosslessMap::get_alt_var(const Eigen::Vector3d& coordinate, int zone_id,
                               unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  return node->get_alt_var(coordinate);
}

float LosslessMap::get_alt_var_safe(const Eigen::Vector3d& coordinate,
                                    int zone_id, unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  return node->get_alt_var(coordinate);
}

unsigned int LosslessMap::get_count(const Eigen::Vector3d& coordinate,
                                    int zone_id, unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node = static_cast<LosslessMapNode*>(get_map_node(index));
  return node->get_count(coordinate);
}

unsigned int LosslessMap::get_count_safe(const Eigen::Vector3d& coordinate,
                                         int zone_id,
                                         unsigned int resolution_id) {
  assert(resolution_id < _map_config._map_resolutions.size());
  MapNodeIndex index = MapNodeIndex::get_map_node_index(_map_config, coordinate,
                                                        resolution_id, zone_id);
  LosslessMapNode* node =
      static_cast<LosslessMapNode*>(get_map_node_safe(index));
  return node->get_count(coordinate);
}

void LosslessMap::preload_map_area(const Eigen::Vector3d& location,
                                   const Eigen::Vector3d& trans_diff,
                                   unsigned int resolution_id,
                                   unsigned int zone_id) {
  BaseMap::preload_map_area(location, trans_diff, resolution_id, zone_id);
  return;
}

bool LosslessMap::load_map_area(const Eigen::Vector3d& seed_pt3d,
                                unsigned int resolution_id,
                                unsigned int zone_id, int filter_size_x,
                                int filter_size_y) {
  BaseMap::load_map_area(seed_pt3d, resolution_id, zone_id, filter_size_x,
                         filter_size_y);
  return true;
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
