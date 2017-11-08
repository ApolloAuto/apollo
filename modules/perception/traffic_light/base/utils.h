//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_UTILS_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_UTILS_H

#include <opencv2/opencv.hpp>

namespace apollo {
namespace perception {
namespace traffic_light {

uint64_t ts_double_2_int64(double ts);

void get_hour_minute(double ts, int &hour, int &minute);

int hour_minute_to_second(int hour, int minute);

void clear_box(cv::Rect &rect);

bool box_is_valid(const cv::Rect &box, const cv::Size &size);
cv::Rect refined_box(cv::Rect box, const cv::Size &size);

cv::Point2f get_center(cv::Rect box);

float get_distance(cv::Point2f, cv::Point2f);

float get_2d_gaussian_score(cv::Point2f p1, cv::Point2f p2, float sigma1, float sigma2);

float get_1d_gaussian_score(float x1, float x2, float sigma);

}
}
}

#endif //GREEN_UTILS_H
