#ifndef BAIDU_ADU_LOSSLESS_MAP_POOL_H
#define BAIDU_ADU_LOSSLESS_MAP_POOL_H

#include "modules/localization/msf/local_map/base_map/base_map_pool.h"
#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_node.h"
#include "modules/localization/msf/local_map/lossless_map/lossless_map_matrix.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The memory pool for the data structure of BaseMapNode. */
class LosslessMapNodePool: public BaseMapNodePool {
public:
    /**@brief Constructor 
     * @param <pool_size> The memory pool size.
     * @param <thread_size> The thread pool size.
     */
    LosslessMapNodePool(unsigned int pool_size, unsigned int thread_size);
    /**@brief Destructor */
    virtual ~LosslessMapNodePool() {}
private:
    virtual BaseMapNode* alloc_new_map_node();
};

} // namespace msf
} // namespace localization
} // namespace apollo

#endif  //BAIDU_ADU_LOSSLESS_MAP_POOL_H