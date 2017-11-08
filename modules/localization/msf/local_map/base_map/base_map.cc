#include "modules/localization/msf/local_map/base_map/base_map.h"
#include "modules/localization/msf/common/util/system_utility.h"

namespace apollo {
namespace localization {
namespace msf {

BaseMap::BaseMap(BaseMapConfig& config)
    : _map_config(config),
      _map_node_cache_lvl1(NULL),
      _map_node_cache_lvl2(NULL),
      _map_node_pool(NULL),
      _p_map_load_threads(NULL),
      _p_map_preload_threads(NULL) {
}

BaseMap::~BaseMap() {
    if (_p_map_load_threads) {
        delete _p_map_load_threads;
    }
    if (_p_map_preload_threads) {
        delete _p_map_preload_threads;
    }
    if (_map_node_cache_lvl1) {
        delete _map_node_cache_lvl1;
    }
    if (_map_node_cache_lvl2) {
        delete _map_node_cache_lvl2;
    }
}

void BaseMap::init_thread_pool(int load_thread_num, int preload_thread_num) {
    if (_p_map_load_threads) {
        delete _p_map_load_threads;
        _p_map_load_threads = NULL;
    }
    if (_p_map_preload_threads) {
        delete _p_map_preload_threads;
        _p_map_preload_threads = NULL;
    }
    _p_map_load_threads = new ThreadPool(load_thread_num);
    _p_map_preload_threads = new ThreadPool(preload_thread_num);
    return;
}

void BaseMap::init_map_node_caches(int cacheL1_size, int cahceL2_size) {
    assert(_map_node_cache_lvl1 == NULL);
    assert(_map_node_cache_lvl2 == NULL);
    _map_node_cache_lvl1 = new MapNodeCacheL1<MapNodeIndex, BaseMapNode>(cacheL1_size);
    _map_node_cache_lvl2 = new MapNodeCacheL2<MapNodeIndex, BaseMapNode>(cahceL2_size);
}

BaseMapNode* BaseMap::get_map_node(const MapNodeIndex& index) {
    BaseMapNode* node = NULL;
    _map_node_cache_lvl1->get(index, node);
    return node;
}

BaseMapNode* BaseMap::get_map_node_safe(const MapNodeIndex& index) {
    BaseMapNode* node = NULL;
    //try get from cacheL1
    if (_map_node_cache_lvl1->get(index, node)) {
        return node;
    }

    //try get from cacheL2
    boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
    if (_map_node_cache_lvl2->get(index, node)) {
        node->set_is_reserved(true);
        // if (_map_node_cache_lvl1->size() >= _map_node_cache_lvl1->capacity()) {
        //     _map_node_cache_lvl1->change_capacity(_map_node_cache_lvl1->size() + 1);
        // }
        _map_node_cache_lvl1->put(index, node);
        return node;
    }
    lock.unlock();

    //load from disk
    std::cerr << "get_map_node_safe: This node don't exist in cache! " << std::endl
        << "load this node from disk now!" << index << std::endl;
    load_map_node_thread_safety(index, true);
    boost::unique_lock<boost::recursive_mutex> lock2(_map_load_mutex); 
    _map_node_cache_lvl2->get(index, node);
    lock2.unlock();
    // if (_map_node_cache_lvl1->size() >= _map_node_cache_lvl1->capacity()) {
    //         _map_node_cache_lvl1->change_capacity(_map_node_cache_lvl1->size() + 1);
    // }
    _map_node_cache_lvl1->put(index, node);
    return node;
}

BaseMapNode* BaseMap::get_map_node_thread_safe(const MapNodeIndex& index) {
    BaseMapNode* node = NULL;
    _map_node_cache_lvl1->get_silent(index, node);
    return node;
}

/**@brief Return the map node, if it's not in the cache, safely load it. */
BaseMapNode* BaseMap::get_map_node_safe_thread_safe(
        const MapNodeIndex& index) {
    BaseMapNode* node = NULL;
    //try get from cacheL1
    if (_map_node_cache_lvl1->get_silent(index, node)) {
        //std::cout << "get_map_node_safe cacheL1" << std::endl;
        return node;
    }

    //try get from cacheL2
    boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
    if (_map_node_cache_lvl2->get(index, node)) {
        node->set_is_reserved(true);
        // if (_map_node_cache_lvl1->size() >= _map_node_cache_lvl1->capacity()) {
        //     _map_node_cache_lvl1->change_capacity(_map_node_cache_lvl1->size() + 1);
        // }
        _map_node_cache_lvl1->put(index, node);
        return node;
    }
    lock.unlock();

    //load from disk
    std::cerr << "get_map_node_safe_thread_safe: This node don't exist in cache! " << std::endl
        << "now load this node from disk!" << index << std::endl;
    load_map_node_thread_safety(index, true);
    boost::unique_lock<boost::recursive_mutex> lock2(_map_load_mutex); 
    _map_node_cache_lvl2->get(index, node);
    lock2.unlock();
    // if (_map_node_cache_lvl1->size() >= _map_node_cache_lvl1->capacity()) {
    //         _map_node_cache_lvl1->change_capacity(_map_node_cache_lvl1->size() + 1);
    // }
    _map_node_cache_lvl1->put(index, node);
    return node;
}

/**@brief Check if the map node in the cache. */
bool BaseMap::is_map_node_exist(const MapNodeIndex& index) const {
    return _map_node_cache_lvl1->is_exist(index);
}

bool BaseMap::set_map_folder_path(const std::string folder_path) {
    _map_config._map_folder_path = folder_path;

    // Try to load the config
    std::string config_path = _map_config._map_folder_path + "/config.xml";
    if (system::IsExists(config_path)) {
        _map_config.load(config_path);
        return true;
    } else {
        return false;
    }
}

void BaseMap::add_dataset(const std::string dataset_path) {
    _map_config._map_datasets.push_back(dataset_path);
    std::string config_path = _map_config._map_folder_path + "/config.xml";
    _map_config.save(config_path);
}

void BaseMap::load_map_nodes(std::set<MapNodeIndex> &map_ids) {
    assert(int(map_ids.size()) <= _map_node_cache_lvl1->capacity());
    // std::cout << "load_map_nodes size: " << map_ids.size() << std::endl;
    //check in cacheL1
    typename std::set<MapNodeIndex>::iterator itr = map_ids.begin();
    while (itr != map_ids.end()) {
        if (_map_node_cache_lvl1->is_exist(*itr)) {
            //std::cout << "load_map_nodes find in L1 cache" << std::endl;
            boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
            _map_node_cache_lvl2->is_exist(*itr); //fresh lru list
            lock.unlock();
            itr = map_ids.erase(itr);
        }
        else {
            ++itr;
        }
    }

    //check in cacheL2
    itr = map_ids.begin();
    BaseMapNode* node = NULL;
    boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
    while (itr != map_ids.end()) {
        if (_map_node_cache_lvl2->get(*itr, node)) {
            //std::cout << "load_map_nodes find in L2 cache" << std::endl;
            node->set_is_reserved(true);
            _map_node_cache_lvl1->put(*itr, node);
            itr = map_ids.erase(itr);
        }
        else {
            ++itr;
        }
    }
    lock.unlock();

    //load from disk sync
    itr = map_ids.begin();
    while (itr != map_ids.end()) {
        _p_map_load_threads->schedule(boost::bind(
                &BaseMap::load_map_node_thread_safety, this, *itr, true));
        ++itr;
    }

    std::cout << "before wait" << std::endl;
    _p_map_load_threads->wait();
    std::cout << "after wait" << std::endl;

    //check in cacheL2 again
    itr = map_ids.begin();
    node = NULL;
    boost::unique_lock<boost::recursive_mutex> lock2(_map_load_mutex); 
    while (itr != map_ids.end()) {
        if (_map_node_cache_lvl2->get(*itr, node)) {
            //std::cout << "load_map_nodes load from disk" << std::endl;
            std::cout << "load_map_nodes: preload missed, load this node in main thread.\n"
                    << *itr << std::endl;
            node->set_is_reserved(true);
            _map_node_cache_lvl1->put(*itr, node);
            itr = map_ids.erase(itr);
        }
        else {
            ++itr;
        }
    }
    lock2.unlock();

    assert(map_ids.size() == 0);
    return;
}

void BaseMap::preload_map_nodes(std::set<MapNodeIndex> &map_ids) {
    assert(int(map_ids.size()) <= _map_node_cache_lvl2->capacity());
    //check in cacheL2
    typename std::set<MapNodeIndex>::iterator itr = map_ids.begin();
    bool is_exist = false;
    while (itr != map_ids.end()) {
        boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
        is_exist = _map_node_cache_lvl2->is_exist(*itr);
        lock.unlock();
        if (is_exist) {
            itr = map_ids.erase(itr);
        }else {
            ++itr;
        }
    }
    
    //check whether in already preloading index set
    itr = map_ids.begin();
    auto preloading_itr = _map_preloading_task_index.end();
    while (itr != map_ids.end()) {
        boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex);
        preloading_itr = _map_preloading_task_index.find(*itr);
        lock.unlock();
        if (preloading_itr != _map_preloading_task_index.end()) { //already preloading
            itr = map_ids.erase(itr);
        }else {
            ++itr;
        }
    }

    //load form disk sync
    itr = map_ids.begin();
    while (itr != map_ids.end()) {
        std::cout << "Preload map node: " << *itr << std::endl;
        boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex);
        _map_preloading_task_index.insert(*itr);
        lock.unlock();
        _p_map_preload_threads->schedule(boost::bind(
                &BaseMap::load_map_node_thread_safety, this, *itr, false));
        ++itr;
    }

    return;
}

void BaseMap::attach_map_node_pool(BaseMapNodePool* map_node_pool) {
    _map_node_pool = map_node_pool;
}

void BaseMap::load_map_node_thread_safety
        (MapNodeIndex index, bool is_reserved) {
    BaseMapNode * map_node = NULL;
    while (map_node == NULL) {
        map_node = _map_node_pool->alloc_map_node();
        if (map_node == NULL) {
            boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex); 
            BaseMapNode* node_remove = _map_node_cache_lvl2->clear_one();
            if (node_remove) {
                _map_node_pool->free_map_node(node_remove);
            }
        }
    }
    //std::cout << "[successfull load node...]" << std::endl;
    map_node->init(&_map_config, index, false);
    if (!map_node->load()) {
        std::cerr << "Created map node: " << index << std::endl;
	}
    else {
	    std::cerr << " Loaded map node: " << index << std::endl;
	}
    map_node->set_is_reserved(is_reserved);
    
    boost::unique_lock<boost::recursive_mutex> lock(_map_load_mutex);
    BaseMapNode* node_remove = _map_node_cache_lvl2->put(index, map_node);
    //if the node already added into cacheL2, erase it from preloading set
    auto itr = _map_preloading_task_index.find(index);
    if (itr != _map_preloading_task_index.end()) {
        _map_preloading_task_index.erase(itr);
    }
    if (node_remove) {
        _map_node_pool->free_map_node(node_remove);
    }
    return;
}

void BaseMap::preload_map_area(const Eigen::Vector3d& location, const Eigen::Vector3d& trans_diff, 
                    unsigned int resolution_id, unsigned int zone_id) {
    assert(_p_map_preload_threads != NULL);
    assert(_map_node_pool != NULL);
    
    int x_direction = trans_diff[0] > 0 ? 1 : -1;
    int y_direction = trans_diff[1] > 0 ? 1 : -1;
    
    std::set<MapNodeIndex> map_ids;
	float map_pixel_resolution = this->_map_config._map_resolutions[resolution_id];
	///top left
	Eigen::Vector3d pt_top_left;
	pt_top_left[0] = location[0] - (this->_map_config._map_node_size_x*map_pixel_resolution/2.0);
	pt_top_left[1] = location[1] - (this->_map_config._map_node_size_y*map_pixel_resolution/2.0);
	pt_top_left[2] = 0;
	MapNodeIndex map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///top center
	Eigen::Vector3d pt_top_center;
	pt_top_center[0] = location[0];
	pt_top_center[1] = pt_top_left[1];
	pt_top_center[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_center, resolution_id, zone_id);
	map_ids.insert(map_id);

	///top right
	Eigen::Vector3d pt_top_right;
	pt_top_right[0] = location[0] + (this->_map_config._map_node_size_x*map_pixel_resolution/2.0);
	pt_top_right[1] = pt_top_left[1];
	pt_top_right[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_right, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle left
	Eigen::Vector3d pt_middle_left;
	pt_middle_left[0] = pt_top_left[0];
	pt_middle_left[1] = location[1];
	pt_middle_left[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_middle_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle center
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, location, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle right
	Eigen::Vector3d pt_middle_right;
	pt_middle_right[0] = pt_top_right[0];
	pt_middle_right[1] = pt_middle_left[1];
	pt_middle_right[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_middle_right, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom left
	Eigen::Vector3d pt_bottom_left;
	pt_bottom_left[0] = pt_top_left[0];
	pt_bottom_left[1] = location[1] + (this->_map_config._map_node_size_y*map_pixel_resolution/2.0);
	pt_bottom_left[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom center
	Eigen::Vector3d pt_bottom_center;
	pt_bottom_center[0] = pt_top_center[0];
	pt_bottom_center[1] = pt_bottom_left[1];
	pt_bottom_center[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_center, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom right
	Eigen::Vector3d pt_bottom_right;
	pt_bottom_right[0] = pt_top_right[0];
	pt_bottom_right[1] = pt_bottom_left[1];
	pt_bottom_right[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_right, resolution_id, zone_id);
	map_ids.insert(map_id);

    for (int i = -1; i < 2; ++i) {
        Eigen::Vector3d pt;
        pt[0] = location[0] + x_direction * 1.5 * this->_map_config._map_node_size_x * map_pixel_resolution;
        pt[1] = location[1] + static_cast<double>(i) * this->_map_config._map_node_size_y * map_pixel_resolution;
        pt[2] = 0;
        map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt, resolution_id, zone_id);
        map_ids.insert(map_id);
    }
    for (int i = -1; i < 2; ++i) {
        Eigen::Vector3d pt;
        pt[0] = location[0] + static_cast<double>(i) * this->_map_config._map_node_size_x * map_pixel_resolution;
        pt[1] = location[1] + y_direction * 1.5 * this->_map_config._map_node_size_y * map_pixel_resolution;
        pt[2] = 0;
        map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt, resolution_id, zone_id);
        map_ids.insert(map_id);
    }
    {
        Eigen::Vector3d pt;
        pt[0] = location[0] + x_direction * 1.5 * this->_map_config._map_node_size_x 
            * map_pixel_resolution;
        pt[1] = location[1] + y_direction * 1.5 * this->_map_config._map_node_size_y 
            * map_pixel_resolution;
        pt[2] = 0;
        map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt, resolution_id, zone_id);
        map_ids.insert(map_id);
    }

    this->preload_map_nodes(map_ids);
    return;
}

bool BaseMap::load_map_area(const Eigen::Vector3d& seed_pt3d, unsigned int resolution_id, 
        unsigned int zone_id, int filter_size_x, int filter_size_y) {
    assert(_p_map_load_threads != NULL);
    assert(_map_node_pool != NULL);
    std::set<MapNodeIndex> map_ids;
	float map_pixel_resolution = this->_map_config._map_resolutions[resolution_id];
	///top left
	Eigen::Vector3d pt_top_left;
	pt_top_left[0] = seed_pt3d[0] - (this->_map_config._map_node_size_x*map_pixel_resolution/2.0)
								  - static_cast<int>(filter_size_x/2) * map_pixel_resolution;
	pt_top_left[1] = seed_pt3d[1] - (this->_map_config._map_node_size_y*map_pixel_resolution/2.0)
								  - static_cast<int>(filter_size_y/2) * map_pixel_resolution;
	pt_top_left[2] = 0;
	MapNodeIndex map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///top center
	Eigen::Vector3d pt_top_center;
	pt_top_center[0] = seed_pt3d[0];
	pt_top_center[1] = pt_top_left[1];
	pt_top_center[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_center, resolution_id, zone_id);
	map_ids.insert(map_id);

	///top right
	Eigen::Vector3d pt_top_right;
	pt_top_right[0] = seed_pt3d[0] + (this->_map_config._map_node_size_x*map_pixel_resolution/2.0)
								  + static_cast<int>(filter_size_x/2) * map_pixel_resolution;
	pt_top_right[1] = pt_top_left[1];
	pt_top_left[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_top_right, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle left
	Eigen::Vector3d pt_middle_left;
	pt_middle_left[0] = pt_top_left[0];
	pt_middle_left[1] = seed_pt3d[1];
	pt_middle_left[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_middle_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle center
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, seed_pt3d, resolution_id, zone_id);
	map_ids.insert(map_id);

	///middle right
	Eigen::Vector3d pt_middle_right;
	pt_middle_right[0] = pt_top_right[0];
	pt_middle_right[1] = seed_pt3d[1];
	pt_middle_right[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_middle_right, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom left
	Eigen::Vector3d pt_bottom_left;
	pt_bottom_left[0] = pt_top_left[0];
	pt_bottom_left[1] = seed_pt3d[1] + (this->_map_config._map_node_size_y*map_pixel_resolution/2.0)
								     + static_cast<int>(filter_size_y/2) * map_pixel_resolution;
	pt_bottom_left[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_left, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom center
	Eigen::Vector3d pt_bottom_center;
	pt_bottom_center[0] = seed_pt3d[0];
	pt_bottom_center[1] = pt_bottom_left[1];
	pt_bottom_center[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_center, resolution_id, zone_id);
	map_ids.insert(map_id);

	///bottom right
	Eigen::Vector3d pt_bottom_right;
	pt_bottom_right[0] = pt_top_right[0];
	pt_bottom_right[1] = pt_bottom_left[1];
	pt_bottom_right[2] = 0;
	map_id = MapNodeIndex::get_map_node_index(this->_map_config, pt_bottom_right, resolution_id, zone_id);
	map_ids.insert(map_id);

    this->load_map_nodes(map_ids);
    return true;
}

} // namespace msf
} // namespace localization
} // namespace apollo
