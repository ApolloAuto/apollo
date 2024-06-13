
#pragma once

#include <vanjeelidar/common/wj_common.hpp>

#include <cmath>

namespace vanjee
{
namespace lidar
{
        /// @brief 计算(0, 360)角度范围内的`sin`和`cos`值粒度是`0.001`度，所以保存它们的两个数组大小为
        class Trigon
        {
        private:
            float *sins_;
            float *coss_;
            float *tans_;
            float* o_sins_;
            float* o_coss_;
            float* o_tans_;

        public:
            constexpr static int64_t ANGLE_MIN = -180000;
            constexpr static int64_t ANGLE_MAX = 360000;

            Trigon()
            {
                int64_t range = ANGLE_MAX - ANGLE_MIN;
                o_sins_ = (float *)malloc(range * sizeof(float));
                o_coss_ = (float *)malloc(range * sizeof(float));
                o_tans_ = (float *)malloc(range * sizeof(float));
                for (int64_t i = ANGLE_MIN, j = 0; i < ANGLE_MAX; i++, j++)
                {
                    double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.001);
                    o_sins_[j] = (float)std::sin(rad);
                    o_coss_[j] = (float)std::cos(rad);
                    o_tans_[j] = (float)std::tan(rad);
                }
                sins_ = o_sins_ - ANGLE_MIN;
                coss_ = o_coss_ - ANGLE_MIN;
                tans_ = o_tans_ - ANGLE_MIN;
            }
            ~Trigon()
            {
                free(o_sins_);
                free(o_coss_);
                free(o_tans_);
            }

            float sin(int64_t angle)
            {
                if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
                {
                    angle = 0;
                }
                return sins_[angle];
            }
            float cos(int64_t angle)
            {
                if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
                {
                    angle = 0;
                }
                return coss_[angle];
            }
            float tan(int64_t angle)
            {
                if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
                {
                    angle = 0;
                }
                return tans_[angle];
            }
            void print()
            {
                for (int64_t i = -1800; i < -1790; i++)
                {
                    std::cout << sins_[i] << "\t" << coss_[i] << std::endl;
                }

            }
        };

} 

} 
