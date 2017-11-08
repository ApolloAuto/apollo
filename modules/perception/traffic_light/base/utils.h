//
// Created by gaohan02 on 16-8-1.
//

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_UTILS_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_UTILS_H

#include <opencv2/opencv.hpp>

namespace apollo {
namespace perception {
namespace traffic_light {

uint64_t TimestampDouble2Int64(double ts);

void ClearBox(cv::Rect &rect);

bool BoxIsValid(const cv::Rect &box, const cv::Size &size);
cv::Rect RefinedBox(cv::Rect box, const cv::Size &size);

cv::Point2f GetCenter(cv::Rect box);

float GetDistance(cv::Point2f, cv::Point2f);

float Get2dGaussianScore(cv::Point2f p1, cv::Point2f p2, float sigma1, float sigma2);

float Get1dGaussianScore(float x1, float x2, float sigma);

}
}
}

#endif //GREEN_UTILS_H
