#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_H

#include <map>
#include <list>
#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"
#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/base_map/base_map_cache.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_pool.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The data structure of the base map. */
class BaseMap {
public:
    /**@brief The constructor. */
    BaseMap(BaseMapConfig& config);
    /**@brief The destructor. */
    virtual ~BaseMap();

    /**@brief Init load threadpool and preload threadpool. */
    void init_thread_pool(int load_thread_num, int preload_thread_num);
    //@brief Init level 1 and level 2 map node caches. */ 
    virtual void init_map_node_caches(int cacheL1_size, int cahceL2_size);

    /**@brief Get the map node, if it's not in the cache, return false. */
    BaseMapNode* get_map_node(const MapNodeIndex& index);
    /**@brief Return the map node, if it's not in the cache, safely load it. */
    BaseMapNode* get_map_node_safe(const MapNodeIndex& index);
    /**@brief Get the map node thread safe, if it's not in the cache, return false. */
    BaseMapNode* get_map_node_thread_safe(const MapNodeIndex& index);
    /**@brief Return the map node, if it's not in the cache, safely load it. */
    BaseMapNode* get_map_node_safe_thread_safe(const MapNodeIndex& index);
    /**@brief Check if the map node in the cache. */
    bool is_map_node_exist(const MapNodeIndex& index) const;

    /**@brief Set the directory of the map. */
    bool set_map_folder_path(const std::string folder_path);
    /**@brief Add a dataset path to the map config. */
    void add_dataset(const std::string dataset_path);

    /**@brief Preload map nodes for the next frame location calculation. 
     * It will forecasts the nodes by the direction of the car moving. 
     * Because the progress of loading will cost a long time (over 100ms), 
     * it must do this for a period of time in advance. 
     * After the index of nodes calculate finished, it will create loading tasks, 
     * but will not wait for the loading finished, eigen version. */
    virtual void preload_map_area(const Eigen::Vector3d& location, const Eigen::Vector3d& trans_diff, 
                    unsigned int resolution_id, unsigned int zone_id);
    /**@brief Load map nodes for the location calculate of this frame. 
     * If the forecasts are correct in last frame, these nodes will be all in cache, if not, 
     * then need to create loading tasks, and wait for the loading finish, 
     * in order to the nodes which the following calculate needed are all in the memory, eigen version. */
    virtual bool load_map_area(const Eigen::Vector3d& seed_pt3d, unsigned int resolution_id, 
            unsigned int zone_id, int filter_size_x, int filter_size_y);

    /**@brief Attach map node pointer. */
    void attach_map_node_pool(BaseMapNodePool* p_map_node_pool);

    /**@brief Write all the map nodes to a single binary file stream. It's for binary streaming or packing.
     * @param <buf, buf_size> The buffer and its size.
     * @param <return> The required or the used size of is returned.
     */
    void write_binary(FILE * file);
    /**@brief Load all the map nodes from a single binary file stream. It's for binary streaming or packing.
     * @param <map_folder_path> The new map folder path to save the map.
     * @param <return> The size read (the real size of body).
     */
    void load_binary(FILE * file, std::string map_folder_path = "");

    /**@brief Get the map config. */
    inline const BaseMapConfig& get_config() const {
        return _map_config;
    }
    /**@brief Get the map config. */
    inline BaseMapConfig& get_config() {
        return _map_config;
    }

protected:
    /**@brief Load map node by index.*/
    void load_map_nodes(std::set<MapNodeIndex> &map_ids);
    /**@brief Load map node by index.*/
    void preload_map_nodes(std::set<MapNodeIndex> &map_ids);
    /**@brief Load map node by index, thread_safety. */
    void load_map_node_thread_safety(MapNodeIndex index, bool is_reserved = false);

    /**@brief The map settings. */
    BaseMapConfig& _map_config;
    /**@brief All the map nodes in the Map (in the disk). */
    std::list<MapNodeIndex> _map_nodes_disk;
    
    /**@brief The cache for map node preload. */
    MapNodeCacheL1<MapNodeIndex, BaseMapNode>* _map_node_cache_lvl1;
    /**brief The dynamic map node preloading thread pool pointer. */
    MapNodeCacheL2<MapNodeIndex, BaseMapNode>* _map_node_cache_lvl2;
    /**@brief The map node memory pool pointer. */
    BaseMapNodePool* _map_node_pool;
    /**@brief The dynamic map node loading thread pool pointer. */
    boost::threadpool::pool* _p_map_load_threads;
    /**@brief The dynamic map node preloading thread pool pointer. */
    boost::threadpool::pool* _p_map_preload_threads;
    /**@bried Keep the index of preloading nodes. */
    std::set<MapNodeIndex> _map_preloading_task_index;
    /**@brief The mutex for preload map node. **/
    boost::recursive_mutex _map_load_mutex;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_H
