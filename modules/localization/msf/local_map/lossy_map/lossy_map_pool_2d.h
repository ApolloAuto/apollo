#ifndef BAIDU_ADU_LOSSY_MAP_POOL_2D_H
#define BAIDU_ADU_LOSSY_MAP_POOL_2D_H

#include "modules/localization/msf/local_map/base_map/base_map_config.h"
#include "modules/localization/msf/local_map/base_map/base_map_node_index.h"
#include "modules/localization/msf/local_map/base_map/base_map_pool.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_node_2d.h"

namespace apollo {
namespace localization {
namespace msf {

/**@brief The memory pool for the data structure of BaseMapNode. */
class LossyMapNodePool2D : public BaseMapNodePool {
 public:
  /**@brief Constructor
   * @param <pool_size> The memory pool size.
   * @param <thread_size> The thread pool size.
   */
  LossyMapNodePool2D(unsigned int pool_size, unsigned int thread_size);
  /**@brief Destructor */
  virtual ~LossyMapNodePool2D() {}

 private:
  virtual BaseMapNode* alloc_new_map_node();
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOSSY_MAP_POOL_2D_H
