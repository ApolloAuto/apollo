#ifndef BAIDU_ADU_LOSSY_MAP_NODE_2D_H
#define BAIDU_ADU_LOSSY_MAP_NODE_2D_H

#include "modules/localization/msf/local_map/base_map/base_map_node.h"
#include "modules/localization/msf/local_map/lossy_map/lossy_map_matrix_2d.h"

namespace apollo {
namespace localization {
namespace msf {

class LossyMapNode2D : public BaseMapNode {
 public:
  LossyMapNode2D() : BaseMapNode(new LossyMapMatrix2D(), new ZlibStrategy()) {}
  ~LossyMapNode2D() {}
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // BAIDU_ADU_LOSSY_MAP_NODE_H
