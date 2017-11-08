//
// Created by gaohan02 on 16-8-1.
//

#include "verify_hdmap.h"
#include "module/perception/traffic_light/base/utils.h"

namespace adu {
namespace perception {
namespace traffic_light {
void HDmapVerify::verify(cv::Mat &ros_image,
                         std::vector<LightPtr> *lights) {
  int rows = ros_image.rows;
  int cols = ros_image.cols;

  for (int h = 0; h < lights->size(); h++) {
    LightPtr light = (*lights)[h];
    if (!box_is_valid(light->region.projection_roi, ros_image.size())) {
      clear_box(light->region.projection_roi);
      continue;
    }

  }
}

void HDmapVerify::init(float hd_scale) {
  _hd_scale = hd_scale;
}
HDmapVerify::HDmapVerify(float hd_scale) {
  init(hd_scale);
}
}
}
}