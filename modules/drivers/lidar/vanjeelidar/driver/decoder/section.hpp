
#pragma once
#include <cmath>
#include <stdint.h>
namespace vanjee
{
namespace lidar
{
        /// @brief 点云的有效性校验，检查水平角是否在有效范围内
        class AzimuthSection
        {
        public:
            AzimuthSection(int64_t start, int64_t end)
            {
                full_round_ = (start == 0) && (end == 360000);

                start_ = start % 360000;
                end_ = end % 360000;
                cross_zero_ = (start_ > end_);
            }
            /// @brief 检查指定的角度`angle`是否在有效范围内
            bool in(int64_t angle)
            {
                if (full_round_)
                    return true;

                if (cross_zero_)
                {
                    return (angle >= start_) || (angle < end_);
                }
                else
                {
                    return (angle >= start_) && (angle < end_);
                }
            }

        private:
            bool full_round_;
            int64_t start_; 
            int64_t end_;   
            bool cross_zero_;
        };
        /// @brief 检查指定的`distance`是否在有效范围内
        class DistanceSection
        {
        public:
            DistanceSection(float min, float max, float usr_min, float usr_max)
                : min_(min), max_(max)
            {
                if (usr_min < 0)
                    usr_min = 0;
                if (usr_max < 0)
                    usr_max = 0;

                if ((usr_min != 0) || (usr_max != 0))
                {
                    min_ = usr_min;
                    max_ = usr_max;
                }
            }

            bool in(float distance)
            {
                return ((min_ <= distance) && (distance <= max_));
            }
        private:
            float min_; 
            float max_; 
        };
        class Projection
        {
        public:
            Projection()
            {
                
            }
            
            double dot(double A[3], double B[3])
            {
                return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
            }
            
            double norm(double V[3])
            {
                return sqrtf(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
            }
            double normAB(double V[3], double U[3])
            {
                return sqrtf((V[0] * V[0] + V[1] * V[1] + V[2] * V[2]) * (U[0] * U[0] + U[1] * U[1] + U[2] * U[2]));
            }

        };

        

} 

} 
