#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include <boost/foreach.hpp>

namespace apollo {
namespace localization {
namespace msf {

BaseMapConfig::BaseMapConfig(std::string map_version) {
    _map_resolutions.push_back(0.125);      // Resolution: 0.125m in level 2
    _map_node_size_x = 1024;    // in pixels
    _map_node_size_y = 1024;    // in pixels
    _map_range = Rect2D<double>(0, 0, 1000448.0, 10000384.0);    // in meters

    _map_version = map_version;
    _map_folder_path = ".";
}

bool BaseMapConfig::save(const std::string file_path) {
    boost::property_tree::ptree config;
    create_xml(config);
    boost::property_tree::write_xml(file_path, config);
    std::cerr << "Saved the map configuration to: " << file_path << "." << std::endl;
    return true;
}

bool BaseMapConfig::load(const std::string file_path) {
    boost::property_tree::ptree config;
    boost::property_tree::read_xml(file_path, config);

    std::string map_version = config.get<std::string>("map.map_config.version");
    if (_map_version == map_version) {
        load_xml(config);
        std::cerr << "Loaded the map configuration from: " << file_path << "." << std::endl;
        return true;
    }
    else {
        std::cerr << "[Fatal Error] Expect v" << _map_version
            << " map, but found v" << map_version << " map." << std::endl;
        return false;
    }
}

void BaseMapConfig::create_xml(boost::property_tree::ptree& config) const {
    config.put("map.map_config.version", _map_version);
    config.put("map.map_config.node_size.x", _map_node_size_x);
    config.put("map.map_config.node_size.y", _map_node_size_y);
    config.put("map.map_config.range.min_x", _map_range.get_min_x());
    config.put("map.map_config.range.min_y", _map_range.get_min_y());
    config.put("map.map_config.range.max_x", _map_range.get_max_x());
    config.put("map.map_config.range.max_y", _map_range.get_max_y());
    config.put("map.map_config.compression", _map_is_compression);
    config.put("map.map_runtime.map_ground_height_offset", _map_ground_height_offset);
    for (size_t i = 0; i < _map_resolutions.size(); ++i) {
        config.add("map.map_config.resolutions.resolution", _map_resolutions[i]);
    }

    for (size_t i = 0; i < _map_datasets.size(); ++i) {
        config.add("map.map_record.datasets.dataset", _map_datasets[i]);
    }
    return;
}

void BaseMapConfig::load_xml(boost::property_tree::ptree& config) {
    _map_resolutions.clear();
    _map_datasets.clear();
    _map_node_size_x = config.get<unsigned int>("map.map_config.node_size.x");
    _map_node_size_y = config.get<unsigned int>("map.map_config.node_size.y");
    double min_x = config.get<double>("map.map_config.range.min_x");
    double min_y = config.get<double>("map.map_config.range.min_y");
    double max_x = config.get<double>("map.map_config.range.max_x");
    double max_y = config.get<double>("map.map_config.range.max_y");
    _map_range = Rect2D<double>(min_x, min_y, max_x, max_y);
    _map_is_compression = config.get<bool>("map.map_config.compression");
    _map_ground_height_offset = config.get<float>("map.map_runtime.map_ground_height_offset");
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v,
            config.get_child("map.map_config.resolutions")) {
        _map_resolutions.push_back(atof(v.second.data().c_str()));
        std::cerr << "Resolution: " << v.second.data() << std::endl;
    }
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v,
            config.get_child("map.map_record.datasets")) {
        _map_datasets.push_back(v.second.data());
    }
    return;
}

void BaseMapConfig::resize_map_range() {
    double min_x = 0;
    double min_y = 0;
    double max_x = 0;
    double max_y = 0;
    
    double max_resolutions = 0.0;
    for (std::size_t i = 0; i < _map_resolutions.size(); ++i) {
        if (max_resolutions < _map_resolutions[i]) {
            max_resolutions = _map_resolutions[i];
        }
    }
    
    int n = 0;
    while (true) {
        if (min_x < _map_range.get_min_x()) {
            break;
        }
        ++n;
        min_x -= n * _map_node_size_x * max_resolutions;
    }
    n = 0;
    while (true) {
        if (min_y < _map_range.get_min_y()) {
            break;
        }
        ++n;
        min_y -= n * _map_node_size_y * max_resolutions;
    }
    n = 0;
    while (true) {
        if (max_x > _map_range.get_max_x()) {
            break;
        }
        ++n;
        max_x += n * _map_node_size_x * max_resolutions;
    }
    n = 0;
    while (true) {
        if (max_y > _map_range.get_max_y()) {
            break;
        }
        ++n;
        max_y += n * _map_node_size_y * max_resolutions;
    }
    _map_range = Rect2D<double>(min_x, min_y, max_x, max_y);
}

void BaseMapConfig::set_single_resolutions(float resolution) {
    _map_resolutions.clear();
    _map_resolutions.push_back(resolution);
}

void BaseMapConfig::set_multi_resolutions() {
    _map_resolutions.clear();
    _map_resolutions.push_back(0.03125); 
    _map_resolutions.push_back(0.0625);
    _map_resolutions.push_back(0.125);
    _map_resolutions.push_back(0.25);
    _map_resolutions.push_back(0.5);
    _map_resolutions.push_back(1);
    _map_resolutions.push_back(2);
    _map_resolutions.push_back(4);
    _map_resolutions.push_back(8);
    _map_resolutions.push_back(16);
}

} // namespace msf
} // namespace localization
} // namespace apollo
