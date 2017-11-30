//
// Created by caros on 11/27/17.
//
#include "tl_shared_data.h"
#include "modules/perception/traffic_light/base/image.h"
namespace apollo {
namespace perception {
namespace traffic_light {

const int kCountCameraId(static_cast<int>(CAMERA_ID_COUNT));
const int kLongFocusIdx(static_cast<int>(LONG_FOCUS));
const int kShortFocusIdx(static_cast<int>(SHORT_FOCUS));
std::vector<int> image_border_size(kCountCameraId, 100);
std::map<TLColor, std::string> kColorStr = {
    {UNKNOWN_COLOR, "unknown"},
    {RED, "red"},
    {GREEN, "green"},
    {YELLOW, "yellow"},
    {BLACK, "black"}
};
}
}
}