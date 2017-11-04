#include "modules/localization/msf/local_map/lossless_map/lossless_map_pool.h"

namespace apollo {
namespace localization {
namespace msf {

LosslessMapNodePool::LosslessMapNodePool(unsigned int pool_size, unsigned int thread_size):
        BaseMapNodePool(pool_size, thread_size) {
}

BaseMapNode* LosslessMapNodePool::alloc_new_map_node() {
    return new LosslessMapNode();
}

} // namespace msf
} // namespace localization
} // namespace apollo
