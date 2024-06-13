
#pragma once
#include <vanjeelidar/msg/packet.hpp>
#include <vanjeelidar/msg/imu_packet.hpp>
#include <vanjeelidar/msg/scan_data_msg.hpp>

#include <vanjeelidar/common/error_code.hpp>
#include <vanjeelidar/driver/driver_param.hpp>
#include <vanjeelidar/driver/decoder/member_checker.hpp>
#include <vanjeelidar/driver/decoder/basic_attr.hpp>
#include <vanjeelidar/driver/decoder/chan_angles.hpp>
#include <vanjeelidar/driver/decoder/imu_calibration_param.hpp>
#include <vanjeelidar/driver/decoder/section.hpp>
#include <vanjeelidar/driver/decoder/trigon.hpp>
#include <vanjeelidar/driver/decoder/split_strategy.hpp>
#include <vanjeelidar/driver/difop/protocol_base.hpp>
#include <vanjeelidar/driver/difop/difop_base.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 
#endif

#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <memory>
#include <iomanip>

namespace vanjee
{
namespace lidar
{
#pragma pack(push, 1)
        typedef struct 
        {
            uint16_t start_angle;
            uint16_t end_angle;
        }WJFOV;
        typedef struct 
        {
            uint8_t sync_mode;
            uint8_t sync_sts;
            WJTimestampUTC timestamp;
        }WJTimeInfo;

#pragma pack(pop)

        enum WJEchoMode
        {
            ECHO_SINGLE = 0,
            ECHO_DUAL
        };

        struct WJDecoderConstParam
        {
            uint16_t MSOP_LEN;

            uint16_t LASER_NUM;          
            uint16_t BLOCKS_PER_PKT;     
            uint16_t CHANNELS_PER_BLOCK; 

            double DISTANCE_MIN; 
            double DISTANCE_MAX;
            double DISTANCE_RES;
            double TEMPERATURE_RES;
        };
#define INIT_ONLY_ONCE()                                                                                               \
  static bool init_flag = false;                                                                                       \
  if (init_flag)                                                                                                       \
    return param;                                                                                                      \
  init_flag = true;


        /// @brief 解析雷达MSOP/DIFOP Packet，得到点云。
        template <typename T_PointCloud>
        class Decoder
        {
        public:
#ifdef ENABLE_TRANSFORM
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
            virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size) = 0;
            virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol){WJ_INFO << "Default processDifop."<<WJ_REND;};
            virtual ~Decoder() = default;
            bool processMsopPkt(const uint8_t *pkt, size_t size);

            explicit Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param);
            /// @brief 计算数据块的时间间隔
            double getPacketDuration();
            void enableWritePktTs(bool value);
            double prevPktTs();
            void transformPoint(float &x, float &y, float &z);

            void regCallback(const std::function<void(const Error &)> &cb_excep,
                             const std::function<void(uint16_t, double)> &cb_split_frame,
                             const std::function<void(void)> & cb_imu_pkt = nullptr,
                             const std::function<void(double)> & cb_scan_data = nullptr);

            void regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16,GetDifoCtrlClass>> get_difo_ctrl_map_ptr);

            std::shared_ptr<T_PointCloud> point_cloud_; 
            std::shared_ptr<ImuPacket> imu_packet_;
            std::shared_ptr<ScanData> scan_data_;

        protected:
            double cloudTs();

            WJDecoderConstParam const_param_;                      
            WJDecoderParam param_;  
            Imu_Calibration_Param imu_calibration_param_;                               
            std::function<void(uint16_t, double)> cb_split_frame_; 
            std::function<void(void)> cb_imu_pkt_;
            std::function<void(double)> cb_scan_data_;
            std::function<void(const Error &)> cb_excep_;
            bool write_pkt_ts_;
            std::shared_ptr<std::map<uint16,GetDifoCtrlClass>> get_difo_ctrl_map_ptr_;

#ifdef ENABLE_TRANSFORM
            Eigen::Matrix4d trans_;
#endif

            Trigon trigon_; 
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

            double packet_duration_;           
            DistanceSection distance_section_; 
            Projection projection;

            WJEchoMode echo_mode_; 
  
            bool angles_ready_;
            double prev_pkt_ts_;    
            double first_point_ts_; 
            double last_point_ts_; 
        };

        template <typename T_PointCloud>
        void Decoder<T_PointCloud>::regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16,GetDifoCtrlClass>> 
                                                                get_difo_ctrl_map_ptr)
        {
          get_difo_ctrl_map_ptr_ = get_difo_ctrl_map_ptr;
        }
        
        template <typename T_PointCloud>
        inline void Decoder<T_PointCloud>::regCallback(const std::function<void(const Error &)> &cb_excep,
                                                       const std::function<void(uint16_t, double)> &cb_split_frame,
                                                       const std::function<void(void)> &cb_imu_pkt,
                                                       const std::function<void(double)> &cb_scan_data)
        {
            cb_excep_ = cb_excep;
            cb_split_frame_ = cb_split_frame;
            cb_imu_pkt_ = cb_imu_pkt;
            cb_scan_data_ = cb_scan_data;
        }

        template <typename T_PointCloud>
        inline Decoder<T_PointCloud>::Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param)
            : const_param_(const_param)
            , param_(param)
            , write_pkt_ts_(false)
            , packet_duration_(0)
            , distance_section_(const_param.DISTANCE_MIN
            , const_param.DISTANCE_MAX
            , param.min_distance, param.max_distance)
            , echo_mode_(ECHO_SINGLE)
            , angles_ready_(false)
            , prev_pkt_ts_(0.0)
            , first_point_ts_(0.0)
        {
#ifdef ENABLE_TRANSFORM
            Eigen::AngleAxisd current_rotation_x(DEGREE_TO_RADIAN(param_.transform_param.roll), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd current_rotation_y(DEGREE_TO_RADIAN(param_.transform_param.pitch), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd current_rotation_z(DEGREE_TO_RADIAN(param_.transform_param.yaw), Eigen::Vector3d::UnitZ());
            Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                                     param_.transform_param.z);
            trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
        }

        template <typename T_PointCloud>
        inline void Decoder<T_PointCloud>::enableWritePktTs(bool value)
        {
            write_pkt_ts_ = value;
        }
        template <typename T_PointCloud>
        inline double Decoder<T_PointCloud>::getPacketDuration()
        {
            return packet_duration_;
        }

        template <typename T_PointCloud>
        inline double Decoder<T_PointCloud>::prevPktTs()
        {
            return prev_pkt_ts_;
        }

        template <typename T_PointCloud>
        inline double Decoder<T_PointCloud>::cloudTs()
        {
            double ret_point_ts = param_.ts_first_point ? first_point_ts_ : last_point_ts_;
            return (ret_point_ts < 0 ? 0 : ret_point_ts);
        }
        /// @brief 对点做坐标变换。它基于第三方开源库Eigen
        template <typename T_PointCloud>
        inline void Decoder<T_PointCloud>::transformPoint(float &x, float &y, float &z)
        {
#ifdef ENABLE_TRANSFORM
            Eigen::Vector4d target_ori(x, y, z, 1);
            Eigen::Vector4d target_rotate = trans_ * target_ori;
            x = target_rotate(0);
            y = target_rotate(1);
            z = target_rotate(2);
#endif
        }
        /// @brief 处理DIFOP Packet
        /// @brief 校验Packet的长度是否匹配。
        /// @brief 校验Packet的标志字节是否匹配。
        /// @brief 如果校验无误，调用decodeDifopPkt()。这是一个纯虚拟函数，由各雷达的派生类提供自己的实现
        
        /// @brief 处理MSOP Packet
        template <typename T_PointCloud>
        inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t *pkt, size_t size)
        {
            constexpr static int CLOUD_POINT_MAX = 1000000;
            if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX))
            {
                LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
            }
            /// @brief 检查当前配置信息是否已经就绪(`angles_ready_`)
            if (param_.wait_for_difop && !angles_ready_)
            {
                DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
                return false;
            }
            /// @brief Packet的长度是否匹配
            // if (size != this->const_param_.MSOP_LEN)
            // {
            //     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
            //     return false;
            // }
            // /// @brief Packet的标志字节是否匹配。
            return decodeMsopPkt(pkt, size);
        }
} 
} // namespace vanjee