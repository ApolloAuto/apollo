#pragma once

#include <memory>
#include <vector>
#include <math.h>
#include <iostream>

#include "vanjeelidar/common/super_header.hpp"
#include <vanjeelidar/driver/decoder/wlr720_32/protocol/imu/complementary_filter.hpp>

#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)

namespace vanjee
{
  namespace lidar
  {
    class ImuParamGet720_32
    {
      public:
        double x_ang_k_;
        double x_ang_b_;
        double y_ang_k_;
        double y_ang_b_;
        double z_ang_k_;
        double z_ang_b_;

        double x_add_k_;
        double x_add_b_;
        double y_add_k_;
        double y_add_b_;
        double z_add_k_;
        double z_add_b_;

        double x_zero_param_;
        double y_zero_param_;
        double z_zero_param_;

        // std::vector<double> imuXda;//emplace_back
        // std::vector<double> imuYda;
        // std::vector<double> imuZda;

        double rotate_b_[2][2];

        int32_t me_zero_param_time_;
        int32_t aim_me_zero_param_time_;

        double dere_time_pre_;

        struct imuGetResultPa
        {
            double q0;
            double q1;
            double q2;
            double q3;

            double x_angle;
            double y_angle;
            double z_angle;

            double x_add;
            double y_add;
            double z_add;

            void init()
            {
                q0 = q1 = q2 = q3 = 0;
                x_angle = y_angle = z_angle = 0;
                x_add = y_add = z_add = 0;
            }
        };

        imuGetResultPa imu_result_stu_;

        //std::shared_ptr<imu_tools_32::ComplementaryFilter32> m_filter_;
        imu_tools_32::ComplementaryFilter32 m_filter_;

        ImuParamGet720_32(int axisoffset)
        {
            x_ang_k_ = y_ang_k_ = z_ang_k_ = 1;
            x_ang_b_ = y_ang_b_ = z_ang_b_ = 0;

            x_add_k_ = y_add_k_ = z_add_k_ = 1;
            x_add_b_ = y_add_b_ = z_add_b_ = 0;

            x_zero_param_ = y_zero_param_ = z_zero_param_ = 0;

            rotate_b_[0][0] = cos(HDL_Grabber_toRadians(axisoffset));
            rotate_b_[0][1] = -sin(HDL_Grabber_toRadians(axisoffset));
            rotate_b_[1][0] = sin(HDL_Grabber_toRadians(axisoffset));
            rotate_b_[1][1] = cos(HDL_Grabber_toRadians(axisoffset));

            me_zero_param_time_ = 0;
            aim_me_zero_param_time_ = 1000;

            dere_time_pre_ = 0;

            imu_result_stu_.init();
        }

        ~ImuParamGet720_32()
        {

        }

        inline void setImuTempCalibrationParams(double x_k,double x_b,double y_k,double y_b,double z_k,double z_b)
        {
          x_ang_k_ = x_k;
          x_ang_b_ = x_b;
          y_ang_k_ = y_k;
          y_ang_b_ = y_b;
          z_ang_k_ = z_k;
          z_ang_b_ = z_b;
        }

        inline void setImuAcceCalibrationParams(double x_k,double x_b,double y_k,double y_b,double z_k,double z_b)
        {
          x_add_k_ = x_k;
          x_add_b_ = x_b;
          y_add_k_ = y_k;
          y_add_b_ = y_b;
          z_add_k_ = z_k;
          z_add_b_ = z_b;
        }

        inline void imuAngCorrbyTemp(double &x_ang, double &y_ang, double &z_ang , double imuTempreture)
        {
            x_ang -= x_ang_k_ * (imuTempreture-40) + x_ang_b_;
            y_ang -= y_ang_k_ * (imuTempreture-40) + y_ang_b_;
            z_ang -= z_ang_k_ * (imuTempreture-40) + z_ang_b_;
        }

        inline void imuAddCorrbyKB(double &x_add, double &y_add, double &z_add)
        {
            x_add = x_add_k_ * x_add - x_add_b_;
            y_add = y_add_k_ * y_add - y_add_b_;
            z_add = z_add_k_ * z_add - z_add_b_;
        }
        
        inline void rotateimu(double &x_ang, double &y_ang, double &z_ang , double &x_add, double &y_add, double &z_add, double imuTempreture)
        {
            double rotate_Ag[2] = {x_add, y_add};
            double rotate_Aa[2] = {x_ang, y_ang};

            x_add = rotate_Ag[0] * rotate_b_[0][0] + rotate_Ag[1] * rotate_b_[1][0];
            y_add = rotate_Ag[0] * rotate_b_[0][1] + rotate_Ag[1] * rotate_b_[1][1];

            x_ang = rotate_Aa[0] * rotate_b_[0][0] + rotate_Aa[1] * rotate_b_[1][0];
            y_ang = rotate_Aa[0] * rotate_b_[0][1] + rotate_Aa[1] * rotate_b_[1][1];
        }
        

        int32_t imuGetZeroPa(double x_ang, double y_ang, double z_ang)
        {
            x_zero_param_ += x_ang / aim_me_zero_param_time_;
            y_zero_param_ += y_ang / aim_me_zero_param_time_;
            z_zero_param_ += z_ang / aim_me_zero_param_time_;

            return me_zero_param_time_++;
        }

        bool imuGet(double x_ang, double y_ang, double z_ang, double x_add, double y_add, double z_add , double temperature , double time)
        {
            imuAngCorrbyTemp(x_ang, y_ang, z_ang , temperature);
            
            x_add = (x_add / 1000 * 0.061 * 9.81);
            y_add = (y_add / 1000 * 0.061 * 9.81);
            z_add = (z_add / 1000 * 0.061 * 9.81);

            x_ang = x_ang / 1000 * 8.75 * 0.0174533;
            y_ang = y_ang / 1000 * 8.75 * 0.0174533;
            z_ang = z_ang / 1000 * 8.75 * 0.0174533;

            imuAddCorrbyKB(x_add, y_add, z_add);

            double l_dertatime;
            if (dere_time_pre_ <= time)
            {
              l_dertatime = time - dere_time_pre_;
            }
            else
            {
              l_dertatime = 100000000 - dere_time_pre_ + time;
            }

            l_dertatime = l_dertatime / 100000000;
            dere_time_pre_ = time;

            if (me_zero_param_time_ < aim_me_zero_param_time_)
            {
                imuGetZeroPa(x_ang, y_ang, z_ang);
                dere_time_pre_ = time;
                return false;
            }
            else if (me_zero_param_time_ == aim_me_zero_param_time_)
            {
                std::cout << "Begin publish Imu" << std::endl;
                me_zero_param_time_ ++;
                dere_time_pre_ = time;
            }

            x_ang = x_ang - x_zero_param_;
            y_ang = y_ang - y_zero_param_;
            z_ang = z_ang - z_zero_param_;

            imu_result_stu_.x_angle = x_ang;
            imu_result_stu_.y_angle = y_ang;
            imu_result_stu_.z_angle = z_ang;

            imu_result_stu_.x_add = x_add;
            imu_result_stu_.y_add = y_add;
            imu_result_stu_.z_add = z_add;

            static double q0 = 0;
            static double q1 = 0;
            static double q2 = 0;
            static double q3 = 0;

            m_filter_.update(x_add, y_add, z_add, x_ang, y_ang, z_ang, l_dertatime, q0, q1, q2, q3);
		        m_filter_.getOrientation(q0, q1, q2, q3);

            imu_result_stu_.q0 = q0;
            imu_result_stu_.q1 = q1;
            imu_result_stu_.q2 = q2;
            imu_result_stu_.q3 = q3;

            return true;
        }
    };

  }
}