#pragma once

#include <cstdio>
#include <cmath>
#include <iostream>

namespace imu_tools_32
{
    double kGravity = 9.81;
    const double gamma_ = 0.01;
    // Bias estimation steady state thresholds
    const double kAngularVelocityThreshold = 0.2;
    const double kAccelerationThreshold = 0.1;
    const double kDeltaAngularVelocityThreshold = 0.01;

    class ComplementaryFilter32
    {
    private:
        // static double kGravity;
        // static const double gamma_;
        // static const double kAngularVelocityThreshold;
        // static const double kAccelerationThreshold;
        // static const double kDeltaAngularVelocityThreshold;

        

        // Gain parameter for the complementary filter, belongs in [0, 1].
        double gain_acc_;
        double gain_mag_;

        // Bias estimation gain parameter, belongs in [0, 1].
        double bias_alpha_;

        // Parameter whether to do bias estimation or not.
        bool do_bias_estimation_;

        // Parameter whether to do adaptive gain or not.
        bool do_adaptive_gain_;

        bool initialized_;
        bool steady_state_;

        // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
        // the orientation of the fixed frame wrt the body frame.
        double q0_, q1_, q2_, q3_;

        // Bias in angular velocities;
        double wx_prev_, wy_prev_, wz_prev_;

        // Bias in angular velocities;
        double wx_bias_, wy_bias_, wz_bias_;

    public:

        void updateBiases(double ax, double ay, double az,
                          double wx, double wy, double wz)
        {
            steady_state_ = checkState(ax, ay, az, wx, wy, wz);

            if (steady_state_)
            {
                wx_bias_ += bias_alpha_ * (wx - wx_bias_);
                wy_bias_ += bias_alpha_ * (wy - wy_bias_);
                wz_bias_ += bias_alpha_ * (wz - wz_bias_);
            }
            // else
            // {
            //     wx_bias_ = 0;
            //     wy_bias_ = 0;
            //     wz_bias_ = 0;
            // }

            wx_prev_ = wx;
            wy_prev_ = wy;
            wz_prev_ = wz;
        }

        bool checkState(double ax, double ay, double az, double wx,
                        double wy, double wz) const
        {
            double acc_magnitude = sqrt(ax * ax + ay * ay + az * az);
            if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
            {

                return false;
            }

            if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
                fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
                fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
            {
                // fprintf(stderr,"wx:%f  wx_prev_:%f  wy:%f  wy_prev_:%f  wz:%f  wz_prev_:%f\n",wx,wx_prev_, wy,wy_prev_, wz, wz_prev_);
                return false;
            }

            if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
                fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
                fabs(wz - wz_bias_) > kAngularVelocityThreshold)
            {
                // fprintf(stderr,"wx:%f  wx_bias_:%f  wy:%f  wy_bias_:%f  wz:%f  wz_bias_:%f\n",wx,wx_bias_, wy,wy_bias_, wz, wz_bias_);
                return false;
            }

            if (fabs(wx - wx_bias_) < 0.005 && fabs(wy - wy_bias_) < 0.007 && fabs(wz - wz_bias_) < 0.002)
            {
                return true;
            }
            else
            {
                return false;
            }

            //return true;
        }

        void getPrediction(double wx, double wy, double wz,
                           double dt, double &q0_pred,
                           double &q1_pred, double &q2_pred,
                           double &q3_pred) 
        {
            double wx_unb = wx - wx_bias_;
            double wy_unb = wy - wy_bias_;
            double wz_unb = wz - wz_bias_;

            q0_pred = q0_ + 0.5 * dt * (wx_unb * q1_ + wy_unb * q2_ + wz_unb * q3_);
            q1_pred = q1_ + 0.5 * dt * (-wx_unb * q0_ - wy_unb * q3_ + wz_unb * q2_);
            q2_pred = q2_ + 0.5 * dt * (wx_unb * q3_ - wy_unb * q0_ - wz_unb * q1_);
            q3_pred = q3_ + 0.5 * dt * (-wx_unb * q2_ + wy_unb * q1_ - wz_unb * q0_);

            normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred);
        }

        void getMeasurement(double ax, double ay, double az,
                            double &q0_meas, double &q1_meas,
                            double &q2_meas, double &q3_meas)
        {
            // q_acc is the quaternion obtained from the acceleration vector
            // representing the orientation of the Global frame wrt the Local frame with
            // arbitrary yaw (intermediary frame). q3_acc is defined as 0.

            // Normalize acceleration vector.
            normalizeVector(ax, ay, az);                

            if (az >= 0)
            {
                q0_meas = sqrt((az + 1) * 0.5);
                q1_meas = -ay / (2.0 * q0_meas);
                q2_meas = ax / (2.0 * q0_meas);
                q3_meas = 0;
            }
            else
            {
                double X = sqrt((1 - az) * 0.5);
                q0_meas = -ay / (2.0 * X);
                q1_meas = X;
                q2_meas = 0;
                q3_meas = ax / (2.0 * X);
            }
        }

        void getMeasurement(double ax, double ay, double az,
                            double mx, double my, double mz,
                            double &q0_meas, double &q1_meas,
                            double &q2_meas, double &q3_meas)
        {
            // q_acc is the quaternion obtained from the acceleration vector
            // representing the orientation of the Global frame wrt the Local frame with
            // arbitrary yaw (intermediary frame). q3_acc is defined as 0.
            double q0_acc, q1_acc, q2_acc, q3_acc;

            // Normalize acceleration vector.
            normalizeVector(ax, ay, az);
            if (az >= 0)
            {
                q0_acc = sqrt((az + 1) * 0.5);
                q1_acc = -ay / (2.0 * q0_acc);
                q2_acc = ax / (2.0 * q0_acc);
                q3_acc = 0;
            }
            else
            {
                double X = sqrt((1 - az) * 0.5);
                q0_acc = -ay / (2.0 * X);
                q1_acc = X;
                q2_acc = 0;
                q3_acc = ax / (2.0 * X);
            }

            // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
            // frame by the inverse of q_acc.
            // l = R(q_acc)^-1 m
            double lx = (q0_acc * q0_acc + q1_acc * q1_acc - q2_acc * q2_acc) * mx +
                        2.0 * (q1_acc * q2_acc) * my - 2.0 * (q0_acc * q2_acc) * mz;
            double ly = 2.0 * (q1_acc * q2_acc) * mx +
                        (q0_acc * q0_acc - q1_acc * q1_acc + q2_acc * q2_acc) * my +
                        2.0 * (q0_acc * q1_acc) * mz;

            // q_mag is the quaternion that rotates the Global frame (North West Up)
            // into the intermediary frame. q1_mag and q2_mag are defined as 0.
            double gamma = lx * lx + ly * ly;
            double beta = sqrt(gamma + lx * sqrt(gamma));
            double q0_mag = beta / (sqrt(2.0 * gamma));
            double q3_mag = ly / (sqrt(2.0) * beta);

            // The quaternion multiplication between q_acc and q_mag represents the
            // quaternion, orientation of the Global frame wrt the local frame.
            // q = q_acc times q_mag
            quaternionMultiplication(q0_acc, q1_acc, q2_acc, q3_acc, q0_mag, 0, 0,
                                     q3_mag, q0_meas, q1_meas, q2_meas, q3_meas);
            // q0_meas = q0_acc*q0_mag;
            // q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
            // q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
            // q3_meas = q0_acc*q3_mag;
        }

        void getAccCorrection(double ax, double ay, double az,
                              double p0, double p1, double p2,
                              double p3, double &dq0, double &dq1,
                              double &dq2, double &dq3)
        {
            // Normalize acceleration vector.
            normalizeVector(ax, ay, az);

            // Acceleration reading rotated into the world frame by the inverse
            // predicted quaternion (predicted gravity):
            double gx, gy, gz;
            rotateVectorByQuaternion(ax, ay, az, p0, -p1, -p2, -p3, gx, gy, gz);

            // Delta quaternion that rotates the predicted gravity into the real
            // gravity:
            dq0 = sqrt((gz + 1) * 0.5);
            dq1 = -gy / (2.0 * dq0);
            dq2 = gx / (2.0 * dq0);
            dq3 = 0.0;
        }

        void getMagCorrection(double mx, double my, double mz,
                              double p0, double p1, double p2,
                              double p3, double &dq0, double &dq1,
                              double &dq2, double &dq3)
        {
            // Magnetic reading rotated into the world frame by the inverse predicted
            // quaternion:
            double lx, ly, lz;
            rotateVectorByQuaternion(mx, my, mz, p0, -p1, -p2, -p3, lx, ly, lz);

            // Delta quaternion that rotates the l so that it lies in the xz-plane
            // (points north):
            double gamma = lx * lx + ly * ly;
            double beta = sqrt(gamma + lx * sqrt(gamma));
            dq0 = beta / (sqrt(2.0 * gamma));
            dq1 = 0.0;
            dq2 = 0.0;
            dq3 = ly / (sqrt(2.0) * beta);
        }

        double getAdaptiveGain(double alpha, double ax, double ay,
                               double az)
        {
            double a_mag = sqrt(ax * ax + ay * ay + az * az);
            double error = fabs(a_mag - kGravity) / kGravity;
            double factor;
            double error1 = 0.1;
            double error2 = 0.2;
            double m = 1.0 / (error1 - error2);
            double b = 1.0 - m * error1;
            if (error < error1)
                factor = 1.0;
            else if (error < error2)
                factor = m * error + b;
            else
                factor = 0.0;
            // printf("FACTOR: %f \n", factor);
            return factor * alpha;
        }

    public:
        ComplementaryFilter32()
            : gain_acc_(0.01),
              gain_mag_(0.01),
              bias_alpha_(0.01),
              do_bias_estimation_(true),
              do_adaptive_gain_(true),
              initialized_(false),
              steady_state_(false),
              q0_(1),
              q1_(0),
              q2_(0),
              q3_(0),
              wx_prev_(0),
              wy_prev_(0),
              wz_prev_(0),
              wx_bias_(0),
              wy_bias_(0),
              wz_bias_(0)
        {
            //std::cout << "ComplementaryFilter shushihua" <<std::endl; 
        }

        ~ComplementaryFilter32()
        {
        }

        bool setGainAcc(double gain)
        {
            if (gain >= 0 && gain <= 1.0)
            {
                gain_acc_ = gain;
                return true;
            }
            else
                return false;
        }

        bool setGainMag(double gain)
        {
            if (gain >= 0 && gain <= 1.0)
            {
                gain_mag_ = gain;
                return true;
            }
            else
                return false;
        }

        bool setBiasAlpha(double bias_alpha)
        {
            if (bias_alpha >= 0 && bias_alpha <= 1.0)
            {
                bias_alpha_ = bias_alpha;
                return true;
            }
            else
                return false;
        }

        double getBiasAlpha() const
        {
            return bias_alpha_;
        }

        bool getSteadyState() const
        {
            return steady_state_;
        }

        void setDoBiasEstimation(bool do_bias_estimation)
        {
            do_bias_estimation_ = do_bias_estimation;
        }

        bool getDoBiasEstimation() const
        {
            return do_bias_estimation_;
        }

        void setDoAdaptiveGain(bool do_adaptive_gain)
        {
            do_adaptive_gain_ = do_adaptive_gain;
        }

        bool getDoAdaptiveGain() const
        {
            return do_adaptive_gain_;
        }

        double getAngularVelocityBiasX() const
        {
            return wx_bias_;
        }

        double getAngularVelocityBiasY() const
        {
            return wy_bias_;
        }

        double getAngularVelocityBiasZ() const
        {
            return wz_bias_;
        }

        void setOrientation(double q0, double q1, double q2,
                            double q3)
        {
            // Set the state to inverse (state is fixed wrt body).
            invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
        }

        double getGainAcc() const
        {
            return gain_acc_;
        }

        double getGainMag() const
        {
            return gain_mag_;
        }

        void getOrientation(double &q0, double &q1, double &q2,
                            double &q3) 
        {
            // Return the inverse of the state (state is fixed wrt body).
            invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
        }

        void update(double ax, double ay, double az, double wx,
                    double wy, double wz, double dt,
                    double &q_0, double &q_1, double &q_2, double &q_3)
        {
            if (!initialized_)
            {
                // kGravity = az;
                // First time - ignore prediction:
                getMeasurement(ax, ay, az, q0_, q1_, q2_, q3_);
                q_0 = q0_;
                q_1 = q1_;
                q_2 = q2_;
                q_3 = q3_;
                initialized_ = true;
                return;
            }

            // Bias estimation.
            if (do_bias_estimation_)
                updateBiases(ax, ay, az, wx, wy, wz);

            // Prediction.
            double q0_pred, q1_pred, q2_pred, q3_pred;
            getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);

            // Correction (from acc):
            // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
            // where qI = identity quaternion
            double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
            getAccCorrection(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc,
                             dq1_acc, dq2_acc, dq3_acc);

            double gain;
            if (do_adaptive_gain_)
            {
                gain = getAdaptiveGain(gain_acc_, ax, ay, az);
            }
            else
            {
                gain = gain_acc_;
            }

            scaleQuaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

            quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc,
                                     dq1_acc, dq2_acc, dq3_acc, q0_, q1_, q2_, q3_);

            normalizeQuaternion(q0_, q1_, q2_, q3_);
            q_0 = q0_;
            q_1 = q1_;
            q_2 = q2_;
            q_3 = q3_;
        }

        void update(double ax, double ay, double az, double wx,
                    double wy, double wz, double mx, double my,
                    double mz, double dt)
        {
            if (!initialized_)
            {
                // First time - ignore prediction:
                getMeasurement(ax, ay, az, mx, my, mz, q0_, q1_, q2_, q3_);
                initialized_ = true;
                return;
            }

            // Bias estimation.
            if (do_bias_estimation_)
                updateBiases(ax, ay, az, wx, wy, wz);

            // Prediction.
            double q0_pred, q1_pred, q2_pred, q3_pred;
            getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);

            // Correction (from acc):
            // q_temp = q_pred * [(1-gain) * qI + gain * dq_acc]
            // where qI = identity quaternion
            double dq0_acc, dq1_acc, dq2_acc, dq3_acc;
            getAccCorrection(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc,
                             dq1_acc, dq2_acc, dq3_acc);
            double alpha = gain_acc_;
            if (do_adaptive_gain_)
                alpha = getAdaptiveGain(gain_acc_, ax, ay, az);
            scaleQuaternion(alpha, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

            double q0_temp, q1_temp, q2_temp, q3_temp;
            quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc,
                                     dq1_acc, dq2_acc, dq3_acc, q0_temp, q1_temp,
                                     q2_temp, q3_temp);

            // Correction (from mag):
            // q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
            // where qI = identity quaternion
            double dq0_mag, dq1_mag, dq2_mag, dq3_mag;
            getMagCorrection(mx, my, mz, q0_temp, q1_temp, q2_temp, q3_temp, dq0_mag,
                             dq1_mag, dq2_mag, dq3_mag);

            scaleQuaternion(gain_mag_, dq0_mag, dq1_mag, dq2_mag, dq3_mag);

            quaternionMultiplication(q0_temp, q1_temp, q2_temp, q3_temp, dq0_mag,
                                     dq1_mag, dq2_mag, dq3_mag, q0_, q1_, q2_, q3_);

            normalizeQuaternion(q0_, q1_, q2_, q3_);
        }

        void normalizeVector(double &x, double &y, double &z)
        {
            double norm = sqrt(x * x + y * y + z * z);

            x /= norm;
            y /= norm;
            z /= norm;
        }

        void normalizeQuaternion(double &q0, double &q1, double &q2, double &q3)
        {
            double norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
            q0 /= norm;
            q1 /= norm;
            q2 /= norm;
            q3 /= norm;
        }

        void invertQuaternion(double q0, double q1, double q2, double q3,
                              double &q0_inv, double &q1_inv, double &q2_inv,
                              double &q3_inv)
        {
            // Assumes quaternion is normalized.
            q0_inv = q0;
            q1_inv = -q1;
            q2_inv = -q2;
            q3_inv = -q3;
        }

        void scaleQuaternion(double gain, double &dq0, double &dq1, double &dq2,
                             double &dq3)
        {
            if (dq0 < 0.0) // 0.9
            {
                // Slerp (Spherical linear interpolation):
                double angle = acos(dq0);
                double A = sin(angle * (1.0 - gain)) / sin(angle);
                double B = sin(angle * gain) / sin(angle);
                dq0 = A + B * dq0;
                dq1 = B * dq1;
                dq2 = B * dq2;
                dq3 = B * dq3;
            }
            else
            {
                // Lerp (Linear interpolation):
                dq0 = (1.0 - gain) + gain * dq0;
                dq1 = gain * dq1;
                dq2 = gain * dq2;
                dq3 = gain * dq3;
            }

            normalizeQuaternion(dq0, dq1, dq2, dq3);
        }

        void quaternionMultiplication(double p0, double p1, double p2, double p3,
                                      double q0, double q1, double q2, double q3,
                                      double &r0, double &r1, double &r2, double &r3)
        {
            // r = p q
            r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3;
            r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2;
            r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1;
            r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0;
        }

        void rotateVectorByQuaternion(double x, double y, double z, double q0,
                                      double q1, double q2, double q3, double &vx,
                                      double &vy, double &vz)
        {
            vx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * x +
                 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z;
            vy = 2 * (q1 * q2 + q0 * q3) * x +
                 (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y +
                 2 * (q2 * q3 - q0 * q1) * z;
            vz = 2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y +
                 (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z;
        }
    };

}

