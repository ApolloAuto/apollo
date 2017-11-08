
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_DETECTION_NMS_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_DETECTION_NMS_H

/*
 *  author: liming22@baidu.com
 */
#include <traffic_light/base/common_type.hpp>
//template <typename Dtype>
//bool nms(const std::vector< BBox<Dtype> >& bbxes,  const int img_height, const int img_width, const float overlappingThreshold, std::vector< BBox<Dtype> >& bbxes_nms);
namespace adu {
namespace perception {
namespace traffic_light {
bool
nms(const std::vector<float> &col, const std::vector<float> &row, const std::vector<float> &size_w,
    const std::vector<float> &size_h, const std::vector<float> &score, const int height,
    const int width,
    const float overlappingThreshold, std::vector<float> &col_nms, std::vector<float> &row_nms,
    std::vector<float> &size_w_nms, std::vector<float> &size_h_nms, std::vector<float> &score_nms);

//template <typename Dtype>
bool nms(const std::vector<BoundBox_t> &bbxes, const int img_height, const int img_width,
         const float overlappingThreshold, std::vector<BoundBox_t> &bbxes_nms);
}
}
}
#endif
