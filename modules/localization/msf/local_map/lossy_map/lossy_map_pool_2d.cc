#include "modules/localization/msf/local_map/lossy_map/lossy_map_pool_2d.h"

namespace apollo {
namespace localization {
namespace msf {

LossyMapNodePool2D::LossyMapNodePool2D(unsigned int pool_size,
                                       unsigned int thread_size)
    : BaseMapNodePool(pool_size, thread_size) {}

BaseMapNode* LossyMapNodePool2D::alloc_new_map_node() {
  return new LossyMapNode2D();
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
