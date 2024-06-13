#pragma once

#include <vanjeelidar/driver/lidar_driver_impl.hpp>
#include <vanjeelidar/msg/packet.hpp>
#include <vanjeelidar/msg/imu_packet.hpp>
#include <vanjeelidar/msg/scan_data_msg.hpp>

namespace vanjee
{
namespace lidar
{
        std::stringstream getDriverVersion();

        /**
         * @brief 激光雷达驱动接口类
         */
        template <typename T_PointCloud>
        class LidarDriver
        {
        private:
            std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;

        public:
            /**
             * @brief 构造函数，初始化驱动程序指针
             */
            LidarDriver()
                : driver_ptr_(std::make_shared<LidarDriverImpl<T_PointCloud>>())
            {
            }
            /**
             * @brief 向驱动注册激光雷达点云回调函数。点云就绪后，将调用此函数
             * @param callback 回调函数
             */
            inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)> &cb_get_cloud,
                                              const std::function<void(std::shared_ptr<T_PointCloud>)> &cb_put_cloud)
            {
                driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
            }
            /**
             * @brief 向驱动注册激光雷达数据包回调函数。数据包就绪后，将调用此函数
             * @param callback 回调函数
             */
            inline void regPacketCallback(const std::function<void(const Packet &)> &cb_put_pkt)
            {
                driver_ptr_->regPacketCallback(cb_put_pkt);
            }

            inline void regImuPacketCallback(const std::function<std::shared_ptr<ImuPacket>(void)> &cb_get_imu_pkt,
                                             const std::function<void(std::shared_ptr<ImuPacket>)> &cb_put_imu_pkt)
            {
              driver_ptr_->regImuPacketCallback(cb_get_imu_pkt,cb_put_imu_pkt);
            }

            /**
             * @brief 向驱动程序注册异常消息回调函数。发生错误时，将调用此函数
             * @param callback 回调函数
             */
            inline void regExceptionCallback(const std::function<void(const Error &)> &cb_excep)
            {
                driver_ptr_->regExceptionCallback(cb_excep);
            }
            /**
             * @brief 初始化函数，用于设置参数和实例对象，用于从在线激光雷达或pcap获取数据包
             * @param param 配置参数
             * @return 成功 返回 true; 否则 返回 false
             */
            inline bool init(const WJDriverParam &param)
            {
                return driver_ptr_->init(param);
            }
            /**
             * @brief 开始接收数据包的线程
             * @return 成功 返回 true; 失败 返回 false
             */
            inline bool start()
            {
                return driver_ptr_->start();
            }
            /**
             * @brief 停止所有线程
             */
            inline void stop()
            {
                driver_ptr_->stop();
            }
        };

} 

} 
