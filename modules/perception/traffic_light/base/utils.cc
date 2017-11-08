//
// Created by gaohan02 on 16-8-1.
//

#include "utils.h"

#define _USE_MATH_DEFINES

#include <cmath>

namespace apollo {
namespace perception {
namespace traffic_light {

uint64_t ts_double_2_int64(double ts) {
  uint64_t result = (uint64_t) (ts * 1e6);
  return result * 1000;
}

void clear_box(cv::Rect &rect) {
  rect.x = 0;
  rect.y = 0;
  rect.width = 0;
  rect.height = 0;
}
bool box_is_valid(const cv::Rect &box, const cv::Size &size) {
  if (box.width <= 0 || box.height <= 0
      || box.x >= size.width || box.y >= size.height
      || box.x + box.width - 1 < 0 || box.y + box.height - 1 < 0) {
    return false;
  }
  return true;
}
cv::Rect refined_box(cv::Rect box, const cv::Size &size) {
  int cols = size.width;
  int rows = size.height;
  if (box.x < 0) {
    box.width += box.x;
    box.x = 0;
  }
  if (box.y < 0) {
    box.height += box.y;
    box.y = 0;
  }
  if (box.x >= cols) {
    box.x = box.width = 0;
  }
  if (box.y >= rows) {
    box.y = box.height = 0;
  }
  box.width = (box.width + box.x < cols) ? box.width : cols - box.x - 1;
  box.height = (box.height + box.y < rows) ? box.height : rows - box.y - 1;

  if (box.width < 0) {
    box.width = 0;
  }
  if (box.height < 0) {
    box.height = 0;
  }
  return box;
}

cv::Point2f get_center(cv::Rect box) {
  return cv::Point2f(box.x + box.width / 2, box.y + box.height / 2);
}
float get_distance(cv::Point2f p1, cv::Point2f p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void get_hour_minute(double ts, int &hour, int &minute) {
  int sec = ((int) ts) % (24 * 3600);
  hour = sec / 3600;
  minute = (sec % 3600) / 60;
}
int hour_minute_to_second(int hour, int minute) {
  return hour * 3600 + minute * 60;
}

float get_2d_gaussian_score(cv::Point2f p1, cv::Point2f p2, float sigma1, float sigma2) {
  return std::exp(-0.5 * (((p1.x - p2.x) * (p1.x - p2.x)) / (sigma1 * sigma1) +
      ((p1.y - p2.y) * (p1.y - p2.y)) / (sigma2 * sigma2)));
  // / (2.0 * M_PI * sigma1 * sigma2);
}

float get_1d_gaussian_score(float x1, float x2, float sigma) {
  return std::exp(-0.5 * (x1 - x2) * (x1 - x2) / (sigma * sigma));
  // / (2.0 * M_PI * sigma * sigma);
}

}
}
}
