#ifndef MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H
#define MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H

#include "modules/localization/msf/local_map/base_map/base_map_fwd.h"
#include <list>
#include <boost/thread.hpp>
#include "modules/localization/msf/common/threadpool/threadpool.hpp"  

namespace apollo {
namespace localization {
namespace msf {

/**@brief The memory pool for the data structure of BaseMapNode. */
class BaseMapNodePool {
public:
    /**@brief Constructor 
     * @param <pool_size> The memory pool size.
     * @param <thread_size> The thread pool size.
     */
    BaseMapNodePool(unsigned int pool_size, unsigned int thread_size);
    /**@brief Destructor */
    virtual ~BaseMapNodePool();
    /**@brief Initialize the pool. 
     * @param <map_config> The map option.
     * @param <is_fixed_size> The flag of pool auto expand.
     * */
    void initial(const BaseMapConfig* map_config, bool is_fixed_size = true);
    /**@brief Release the pool. */
    void release();
    /**@brief Get a MapNode object from memory pool.
     * @param <return> The MapNode object.
     * */
    BaseMapNode* alloc_map_node();
    /**@brief Release MapNode object to memory pool.
     * @param <map_node> The released MapNode object.
     * */
    void free_map_node(BaseMapNode* map_node);
    /**@brief Get the size of pool. */
    unsigned int get_pool_size() {
        return _pool_size;
    }
private:
    /**@brief The task function of the thread pool for release node.
     * @param <map_node> The released MapNode object.
     * */
    void free_map_node_task(BaseMapNode* map_node);
    /**@brief new a map node. */
    virtual BaseMapNode* alloc_new_map_node() = 0;
    /**@brief init a map node. */
    virtual void init_new_map_node(BaseMapNode* node);
    /**@brief finalize a map node, before reset or delloc the map node. */
    virtual void finalize_map_node(BaseMapNode* node);
    /**@brief delloc a map node. */
    virtual void delloc_map_node(BaseMapNode* node);
    /**@brief reset a map node. */
    virtual void reset_map_node(BaseMapNode* node);
protected:
    /**@brief The flag of pool auto expand. */
    bool _is_fixed_size;
    /**@brief The list for free node. */
    std::list<BaseMapNode*> _free_list;
    /**@brief The set for used node. */
    std::set<BaseMapNode*> _busy_nodes;
    /**@brief The size of memory pool. */
    unsigned int _pool_size;
    /**@brief The thread pool for release node. */
    boost::threadpool::pool _node_reset_workers;
    /**@brief The mutex for release thread.*/
    boost::mutex _mutex;
    /**@brief The mutex for release thread.*/
    const BaseMapConfig* _map_config;
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif // MODULES_LOCALIZATION_MSF_LOCAL_MAP_BASE_MAP_BASE_MAP_POOL_H