
#pragma once

#include <vanjeelidar/driver/decoder/decoder_mech.hpp>
#include <vanjeelidar/driver/decoder/imu_calibration_param.hpp>
#include <vanjeelidar/driver/difop/protocol_base.hpp>
#include <vanjeelidar/driver/difop/cmd_class.hpp>
#include <vanjeelidar/driver/difop/protocol_abstract.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/frames/cmd_repository_720.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/frames/protocol_ldangle_get.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/frames/protocol_imuaddpa_get.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/frames/protocol_imulinepa_get.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/frames/protocol_imu_temp_get.hpp>
#include <vanjeelidar/driver/decoder/wlr720/protocol/imu/imuParamGet.hpp>

namespace vanjee
{
    namespace lidar
    {
        #pragma pack(push, 1)
        typedef struct _Vanjee720Channel
        {
            uint16_t distance;      ///< 距离 缩小4倍
            uint8_t intensity;      ///< 强度值
            uint8_t reflectivity;   ///< 反射
        } Vanjee720Channel;

        typedef struct _Vanjee720Block
        {
            uint16_t azimuth;               ///< 方位角
            Vanjee720Channel channel[16];   ///< 通道数据
        } Vanjee720Block;

        typedef struct _Vanjee720Block2
        {
            uint16_t azimuth;               //< 方位角
            Vanjee720Channel channel[19];   ///< 通道数据
        } Vanjee720Block2;

        typedef struct _Vanjee720BlockFFEE
        {
            uint8_t header[2];
            uint16_t azimuth;               ///< 方位角
            Vanjee720Channel channel[19];   ///< 通道数据
        } Vanjee720BlockFFEE;

        typedef struct _Vanjee720Difop
        {
            uint8_t mac_id[2];      
            uint16_t circle_id;     
            uint8_t data_time[6];   
            uint8_t time_stamp[4];  
            int16_t palstance_x;   
            int16_t palstance_y;
            int16_t palstance_z;
            int16_t linear_acceleration_x;
            int16_t linear_acceleration_y;
            int16_t linear_acceleration_z;
            uint8_t info[34];
        } Vanjee720Difop;

        typedef struct _Vanjee720MsopPkt16
        {
            uint8_t head[2];
            uint8_t channel_num;
            uint8_t return_wave_num;
            uint8_t block_num;
            Vanjee720Block blocks[18]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt16;

        typedef struct _Vanjee720MsopPkt19
        {
            Vanjee720Block2 blocks[15]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt19;

        typedef struct _Vanjee720MsopPkt19Double
        {
            uint8_t head[2];
            uint8_t channel_num;
            uint8_t return_wave_num;
            uint8_t block_num;
            Vanjee720Block2 blocks[12]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt19Double;

        typedef struct _Vanjee720MsopPkt19FFEE
        {
            Vanjee720BlockFFEE blocks[15]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt19FFEE;

        #pragma pack(pop)
        template <typename T_PointCloud>
        class DecoderVanjee720 : public DecoderMech<T_PointCloud>
        {
        private:
            std::vector<std::vector<double>> all_points_luminous_moment_720c_;  // 缓存16通道一圈点云时间差
            std::vector<std::vector<double>> all_points_luminous_moment_720f_;  // 缓存19通道一圈点云时间差
            const double luminous_period_of_ld_ = 0.00005555;                   // 相邻水平角度下时间间隔
            const double luminous_period_of_adjacent_ld_ = 0.00000234;          // 组内相邻垂直角度下时间间隔
            
            int32_t azimuth_cur_ = -1.0; // 当前角度

            int32_t pre_frame_id_ = -1;
            uint8_t publish_mode_ = 0;

            std::vector<float> point_num_offset_ffdd_ = std::vector<float>
            {
                0, 90, 0, 50
            };

            std::vector<float> point_num_offset_ffee_ = std::vector<float>
            {
                0, 90, 0, 45
            };
        
            std::shared_ptr<SplitStrategy> split_strategy_; 
            static WJDecoderMechConstParam &getConstParam(uint8_t mode);
            static WJEchoMode getEchoMode(uint8_t mode);
            void initLdLuminousMoment(void);

        public:
            constexpr static double FRAME_DURATION = 0.1;
            constexpr static uint32_t SINGLE_PKT_NUM = 100;

            virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
            virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
            virtual ~DecoderVanjee720() = default;
            explicit DecoderVanjee720(const WJDecoderParam &param);

            bool decodeMsopPkt720F_FFEE(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt720C(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt720F_FFDD(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt720F_DoubleEcho(const uint8_t *pkt, size_t size);

            void SendImuData(Vanjee720Difop difop , double temperature , int azimuth,double timestamp);

            uint16_t chanToline(uint16_t chan);

        public:
            std::shared_ptr<ImuParamGet720> m_imu_params_get_;
            double imu_temperature_;
        };
    
        template<typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::initLdLuminousMoment()
        {
            double offset = 0;
            all_points_luminous_moment_720c_.resize(4);
            all_points_luminous_moment_720c_[0].resize(57600);
            all_points_luminous_moment_720c_[1].resize(28800);
            all_points_luminous_moment_720c_[2].resize(19200);
            all_points_luminous_moment_720c_[3].resize(14400);
            all_points_luminous_moment_720f_.resize(4);
            all_points_luminous_moment_720f_[0].resize(68400);
            all_points_luminous_moment_720f_[1].resize(34200);
            all_points_luminous_moment_720f_[2].resize(22800);
            all_points_luminous_moment_720f_[3].resize(17100);
            for(uint16_t col = 0;col < 3600;col++)
            {
                for(uint8_t row = 0;row < 16;row++)
                {
                    if(row < 4)
                        offset = (row+1)*luminous_period_of_adjacent_ld_;
                    else if(row < 7)
                        offset = (row+1-4)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ / 4;
                    else if (row < 12)
                        offset =(row - 7)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 2 / 4;
                    else
                        offset = (row+1-12)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 3 / 4;
                    
                    if(col < 900)
                    {
                        for(int i = 0; i < 4; i++)
                            all_points_luminous_moment_720c_[i][col*16+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 900 && col < 1200)
                    {
                        for(int i = 0; i < 3; i++)
                            all_points_luminous_moment_720c_[i][col*16+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 1200 && col < 1800)
                    {
                        for(int i = 0; i < 2; i++)
                            all_points_luminous_moment_720c_[i][col*16+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else
                    {
                        all_points_luminous_moment_720c_[0][col*16+row] = col*luminous_period_of_ld_ + offset;
                    }
                }
            }

            for(uint16_t col = 0;col < 3600;col++)
            {
                for(uint8_t row = 0;row < 19;row++)
                {
                    if(row < 5)
                        offset = row*luminous_period_of_adjacent_ld_;
                    else if(row < 9)
                        offset = (row-5)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ / 4;
                    else if (row < 14)
                        offset =(row-9)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 2 / 4;
                    else
                        offset = (row-14)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 3 / 4;

                    if(col < 900)
                    {
                        for(int i = 0; i < 4; i++)
                            all_points_luminous_moment_720f_[i][col*19+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 900 && col < 1200)
                    {
                        for(int i = 0; i < 3; i++)
                            all_points_luminous_moment_720f_[i][col*19+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 1200 && col < 1800)
                    {
                        for(int i = 0; i < 2; i++)
                            all_points_luminous_moment_720f_[i][col*19+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else
                    {
                        all_points_luminous_moment_720f_[0][col*19+row] = col*luminous_period_of_ld_ + offset;
                    }
                }
            }
        }

        template <typename T_PointCloud>
        inline WJDecoderMechConstParam &DecoderVanjee720<T_PointCloud>::getConstParam(uint8_t mode)
        {
            //WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
            uint16_t msop_len = 1253;
            uint16_t laser_num = 16;
            uint16_t block_num = 18;
            uint16_t chan_num = 16;
            float distance_min = 0.3f;
            float distance_max = 120.0f;
            float distance_resolution = 0.004f;
            float init_temperature = 80.0f;

            // if(lidar_type_id == 1 && mode == 1)
            // {
            //     msop_len = 1260;
            //     laser_num = 16;
            //     block_num = 15;
            //     chan_num = 19;
            // }
            // else if(lidar_type_id == 1 && mode == 1)
            // {
            //     msop_len = 1253;
            //     laser_num = 16;
            //     block_num = 18;
            //     chan_num= 16;
            // }
            // else if(lidar_type_id == 2 && mode == 1)
            // {
            //     msop_len = 1235;
            //     laser_num = 16;
            //     block_num = 15;
            //     chan_num = 16;
            // }
            // else if(lidar_type_id == 2 && mode == 2)
            // {
            //     msop_len = 1001;
            //     laser_num = 16;
            //     block_num = 12;
            //     chan_num = 19;
            // }

            static WJDecoderMechConstParam param =
                {
                    msop_len /// msop len
                    ,
                    laser_num /// laser number
                    ,
                    block_num /// blocks per packet
                    ,
                    chan_num /// channels per block
                    ,
                    distance_min /// distance min
                    ,
                    distance_max /// distance max
                    ,
                    distance_resolution /// distance resolution
                    ,
                    init_temperature /// initial value of temperature
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
        }
        
        template <typename T_PointCloud>
        inline WJEchoMode DecoderVanjee720<T_PointCloud>::getEchoMode(uint8_t mode)
        {
            switch (mode)
            {
            case 0x10: ///
                return WJEchoMode::ECHO_DUAL;
            case 0x20:
            case 0x30:

            default:
                return WJEchoMode::ECHO_SINGLE;
            }
        }
        
        template <typename T_PointCloud>
        inline DecoderVanjee720<T_PointCloud>::DecoderVanjee720(const WJDecoderParam &param)
            : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
        {
            if(param.max_distance < param.min_distance)
                WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

            publish_mode_ = param.publish_mode;
            this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
            split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

            m_imu_params_get_ = std::make_shared<ImuParamGet720>(0);
            imu_temperature_ = 25.0;
            
            if(this->param_.config_from_file)
            {
              int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
              int ret_imu = this->imu_calibration_param_.loadFromFile(param.imu_param_path);  
              
              m_imu_params_get_->setImuTempCalibrationParams(
                                    this->imu_calibration_param_.x_axis_temp_k,this->imu_calibration_param_.x_axis_temp_b,
                                    this->imu_calibration_param_.y_axis_temp_k,this->imu_calibration_param_.y_axis_temp_b,
                                    this->imu_calibration_param_.z_axis_temp_k,this->imu_calibration_param_.z_axis_temp_b);
                                   
              m_imu_params_get_->setImuAcceCalibrationParams(
                                    this->imu_calibration_param_.x_axis_acc_k,this->imu_calibration_param_.x_axis_acc_b,
                                    this->imu_calibration_param_.y_axis_acc_k,this->imu_calibration_param_.y_axis_acc_b,
                                    this->imu_calibration_param_.z_axis_acc_k,this->imu_calibration_param_.z_axis_acc_b);
            }
            initLdLuminousMoment();
        }

        template <typename T_PointCloud>
        inline bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
        {
            bool ret = false;
            switch (size)
            {
            case 1260:
            {
                ret = decodeMsopPkt720F_FFEE(pkt, size);
            }
            break;

            case 1253:
            {
                ret = decodeMsopPkt720C(pkt, size);
            }
            break;

            case 1235:
            {
                ret = decodeMsopPkt720F_FFDD(pkt, size);
            }
            break;

            case 1001:
            {
                ret = decodeMsopPkt720F_DoubleEcho(pkt, size);
            }
            break;
            }
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_FFEE(const uint8_t *pkt, size_t size)
        {
            const Vanjee720MsopPkt19FFEE &packet = *(Vanjee720MsopPkt19FFEE *)pkt;

            bool ret = false;
            double pkt_ts = 0;

            uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
            uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
            if(loss_packets_num > 1 && pre_frame_id_ >= 0)
                WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
            pre_frame_id_ = frame_id;

            if(!this->param_.use_lidar_clock)
                pkt_ts = getTimeHost() * 1e-6;
            else
            {
                std::tm stm;
                memset(&stm, 0, sizeof(stm));
                stm.tm_year = packet.difop.data_time[5] + 100;
                stm.tm_mon = packet.difop.data_time[4] - 1;
                stm.tm_mday = packet.difop.data_time[3];
                stm.tm_hour = packet.difop.data_time[2];
                stm.tm_min = packet.difop.data_time[1];
                stm.tm_sec = packet.difop.data_time[0];
                double nsec = (packet.difop.time_stamp[0] + (packet.difop.time_stamp[1] << 8) + (packet.difop.time_stamp[2] << 16) + ((packet.difop.time_stamp[3] & 0x0F) << 24)) / 100000000.0;
                pkt_ts = std::mktime(&stm) + nsec;
            }

            int32_t resolution = 10;
            uint8_t resolution_index = 0;
            double last_point_to_first_point_time = 0;
            if(packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0)
            {
                resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }
            else
            {
                resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }

            if(resolution == 10)
            {
                resolution_index = 0;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720f_[resolution_index][68399];
            }
            else if(resolution == 20)
            {
                resolution_index = 1;
                last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720f_[resolution_index][34199];
            }
            else if(resolution == 30)
            {
                resolution_index = 2;
                last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720f_[resolution_index][22799];
            }
            else if(resolution == 40)
            {
                resolution_index = 3;
                last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720f_[resolution_index][17099];
            }
            else
            {
                return ret;
            }
            
            azimuth_cur_ = packet.blocks[0].azimuth % 36000;
            
            SendImuData(packet.difop , imu_temperature_ , azimuth_cur_, pkt_ts);

            for (uint16_t blk = 0; blk < 15; blk++)
            {
                const Vanjee720BlockFFEE &block = packet.blocks[blk];
                int32_t azimuth = block.azimuth % 36000;
                int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

                if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0)
                {
                    int32_t time_reload_num = (azimuth / resolution) / 180;
                    if(((azimuth / resolution) % 180) > 0)
                        time_reload_num += 1;
                    int32_t point_gap_num = (time_reload_num * 180 - point_num_offset_ffee_[resolution_index]) * 19 ;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
                    this->first_point_ts_ =  this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];

                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
                if ((block.header[0] == 255) && (block.header[1] == 238))
                {
                    double timestamp_point;
                    for (uint16_t chan = 0; chan < 19; chan++)
                    {
                        float x, y, z, xy;

                        uint32_t point_id = azimuth / resolution * 19 + chan;
                        if(this->param_.ts_first_point == true)
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
                        }
                        else
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                        }

                        const Vanjee720Channel &channel = block.channel[chan];
                        float distance = channel.distance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final = 0;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }

                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.reflectivity);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            
                            setIntensity(point, 0);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                }
                if (azimuth_trans == 0)
                {
                    this->last_point_ts_ = pkt_ts;
                    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720C(const uint8_t *pkt, size_t size)
        {
            const Vanjee720MsopPkt16 &packet = *(Vanjee720MsopPkt16 *)pkt;
            
            bool ret = false;
            double pkt_ts = 0;

            uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
            uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
            if(loss_packets_num > 1 && pre_frame_id_ >= 0)
                WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
            pre_frame_id_ = frame_id;

            if(!this->param_.use_lidar_clock)
                pkt_ts = getTimeHost() * 1e-6;
            else
            {
                std::tm stm;
                memset(&stm, 0, sizeof(stm));
                stm.tm_year = packet.difop.data_time[5] + 100;
                stm.tm_mon = packet.difop.data_time[4] - 1;
                stm.tm_mday = packet.difop.data_time[3];
                stm.tm_hour = packet.difop.data_time[2];
                stm.tm_min = packet.difop.data_time[1];
                stm.tm_sec = packet.difop.data_time[0];
                double nsec = (packet.difop.time_stamp[0] + (packet.difop.time_stamp[1] << 8) + (packet.difop.time_stamp[2] << 16) + ((packet.difop.time_stamp[3] & 0x0F) << 24)) / 100000000.0;
                pkt_ts = std::mktime(&stm) + nsec;
            }

            int32_t resolution = 10;
            uint8_t resolution_index = 0;
            double last_point_to_first_point_time = 0;
            if(packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0)
            {
                resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }
            else
            {
                resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }

            if(resolution == 10)
            {
                resolution_index = 0;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][57599];
            }
            else if(resolution == 20)
            {
                resolution_index = 1;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][28799];
            }
            else if(resolution == 30)
            {
                resolution_index = 2;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][19199];
            }
            else if(resolution == 40)
            {
                resolution_index = 3;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][14399];
            }
            else
            {
                return ret;
            }

            azimuth_cur_ = packet.blocks[0].azimuth % 36000;
                        
            SendImuData(packet.difop , imu_temperature_ , azimuth_cur_, pkt_ts);

            int blacknum = packet.return_wave_num * packet.block_num;
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                if(packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1))
                {
                    continue;
                }
                else if(packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0))
                {
                    continue;
                }
                const Vanjee720Block &block = packet.blocks[blk];
                int32_t azimuth = block.azimuth % 36000;
                int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

                if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0)
                {
                    int32_t time_reload_num = (azimuth / resolution) / 180;
                    if(((azimuth / resolution) % 180) > 0)
                        time_reload_num += 1;
                    int32_t point_gap_num = (time_reload_num * 180 - point_num_offset_ffdd_[resolution_index]) * 16;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720c_[resolution_index][point_gap_num] - last_point_to_first_point_time;
                    this->first_point_ts_ =  this->last_point_ts_ - all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size()-1];

                    this->cb_split_frame_(16, this->cloudTs());
                    ret = true;
                }
                {
                    double timestamp_point;
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        uint32_t point_id = azimuth / resolution * pkt[2] + chan;
                        if(this->param_.ts_first_point == true)
                        {
                          timestamp_point = all_points_luminous_moment_720c_[resolution_index][point_id];
                        }
                        else
                        {
                          timestamp_point = all_points_luminous_moment_720c_[resolution_index][point_id] - all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size()-1];
                        }

                        const Vanjee720Channel &channel = block.channel[chan];
                        float distance = channel.distance * this->const_param_.DISTANCE_RES;
                        int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                        int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth * 10) % 360000;
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }


                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.reflectivity);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                            
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            setIntensity(point, 0);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                }
                if (azimuth_trans == 0)
                {
                    this->last_point_ts_ = pkt_ts;
                    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_720c_[resolution_index][all_points_luminous_moment_720c_[resolution_index].size()-1];
                    this->cb_split_frame_(16, this->cloudTs());
                    ret = true;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_FFDD(const uint8_t *pkt, size_t size)
        {
            auto &packet = *(Vanjee720MsopPkt19 *)&pkt[5];

            bool ret = false;
            double pkt_ts = 0;

            uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
            uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
            if(loss_packets_num > 1 && pre_frame_id_ >= 0)
                WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
            pre_frame_id_ = frame_id;

            if(!this->param_.use_lidar_clock)
                pkt_ts = getTimeHost() * 1e-6;
            else
            {
                std::tm stm;
                memset(&stm, 0, sizeof(stm));
                stm.tm_year = packet.difop.data_time[5] + 100;
                stm.tm_mon = packet.difop.data_time[4] - 1;
                stm.tm_mday = packet.difop.data_time[3];
                stm.tm_hour = packet.difop.data_time[2];
                stm.tm_min = packet.difop.data_time[1];
                stm.tm_sec = packet.difop.data_time[0];
                double nsec = (packet.difop.time_stamp[0] + (packet.difop.time_stamp[1] << 8) + (packet.difop.time_stamp[2] << 16) + ((packet.difop.time_stamp[3] & 0x0F) << 24)) / 100000000.0;
                pkt_ts = std::mktime(&stm) + nsec;
            }

            int32_t resolution = 10;
            uint8_t resolution_index = 0;
            double last_point_to_first_point_time = 0;
            if(packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0)
            {
                resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }
            else
            {
                resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }

            if(resolution == 10)
            {
                resolution_index = 0;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720f_[resolution_index][68399];
            }
            else if(resolution == 20)
            {
                resolution_index = 1;
                last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720f_[resolution_index][34199];
            }
            else if(resolution == 30)
            {
                resolution_index = 2;
                last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720f_[resolution_index][22799];
            }
            else if(resolution == 40)
            {
                resolution_index = 3;
                last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720f_[resolution_index][17099];
            }
            else
            {
                return ret;
            }

            azimuth_cur_ = packet.blocks[0].azimuth % 36000;

            SendImuData(packet.difop , imu_temperature_ , azimuth_cur_, pkt_ts);

            int blacknum = pkt[3] * pkt[4];
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                const Vanjee720Block2 &block = packet.blocks[blk];
                int32_t azimuth = block.azimuth % 36000;
                int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

                if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0)
                {
                    int32_t time_reload_num = (azimuth / resolution) / 180;
                    if(((azimuth / resolution) % 180) > 0)
                        time_reload_num += 1;
                    int32_t point_gap_num = (time_reload_num * 180 - point_num_offset_ffdd_[resolution_index]) * 19;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
                    this->first_point_ts_ =  this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];

                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
                
                {
                    double timestamp_point;
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        uint32_t point_id = azimuth / resolution * pkt[2] + chan;
                        if(this->param_.ts_first_point == true)
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
                        }
                        else
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                        }

                        const Vanjee720Channel &channel = block.channel[chan];
                        float distance = channel.distance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final = 0;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }

                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.reflectivity);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            setIntensity(point, 0);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                }
                if (azimuth_trans == 0)
                {
                    this->last_point_ts_ = pkt_ts;
                    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt720F_DoubleEcho(const uint8_t *pkt, size_t size)
        {
            auto &packet = *(Vanjee720MsopPkt19Double *)pkt;

            bool ret = false;
            double pkt_ts = 0;

            uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
            uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
            if(loss_packets_num > 1 && pre_frame_id_ >= 0)
                WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
            pre_frame_id_ = frame_id;

            if(!this->param_.use_lidar_clock)
                pkt_ts = getTimeHost() * 1e-6;
            else
            {
                std::tm stm;
                memset(&stm, 0, sizeof(stm));
                stm.tm_year = packet.difop.data_time[5] + 100;
                stm.tm_mon = packet.difop.data_time[4] - 1;
                stm.tm_mday = packet.difop.data_time[3];
                stm.tm_hour = packet.difop.data_time[2];
                stm.tm_min = packet.difop.data_time[1];
                stm.tm_sec = packet.difop.data_time[0];
                double nsec = (packet.difop.time_stamp[0] + (packet.difop.time_stamp[1] << 8) + (packet.difop.time_stamp[2] << 16) + ((packet.difop.time_stamp[3] & 0x0F) << 24)) / 100000000.0;
                pkt_ts = std::mktime(&stm) + nsec;
            }

            int32_t resolution = 10;
            uint8_t resolution_index = 0;
            double last_point_to_first_point_time = 0;
            if(packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0)
            {
                resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }
            else
            {
                resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }

            if(resolution == 10)
            {
                resolution_index = 0;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720c_[resolution_index][68399];
            }
            else if(resolution == 20)
            {
                resolution_index = 1;
                last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720c_[resolution_index][34199];
            }
            else if(resolution == 30)
            {
                resolution_index = 2;
                last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720c_[resolution_index][22799];
            }
            else if(resolution == 40)
            {
                resolution_index = 3;
                last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720c_[resolution_index][17099];
            }
            else
            {
                return ret;
            }
            
            azimuth_cur_ = packet.blocks[0].azimuth % 36000;
            
            SendImuData(packet.difop , imu_temperature_ , azimuth_cur_, pkt_ts);

            int blacknum = packet.return_wave_num * packet.block_num;
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                if(packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1))
                {
                    continue;
                }
                else if(packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0))
                {
                    continue;
                }
                const Vanjee720Block2 &block = packet.blocks[blk];
                int32_t azimuth = block.azimuth % 36000;
                int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

                if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0)
                {
                    int32_t time_reload_num = (azimuth / resolution) / 180;
                    if(((azimuth / resolution) % 180) > 0)
                        time_reload_num += 1;
                    int32_t point_gap_num = (time_reload_num * 180 - point_num_offset_ffdd_[resolution_index]) * 19;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720f_[resolution_index][point_gap_num] - last_point_to_first_point_time;
                    this->first_point_ts_ =  this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];

                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
                
                {
                    double timestamp_point;
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        uint32_t point_id = azimuth / resolution * pkt[2] + chan;
                        if(this->param_.ts_first_point == true)
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id];
                        }
                        else
                        {
                          timestamp_point = all_points_luminous_moment_720f_[resolution_index][point_id] - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                        }

                        const Vanjee720Channel &channel = block.channel[chan];
                        float distance = channel.distance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final = 0;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + resolution * 10 / 4 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }

                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.reflectivity);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            
                            setIntensity(point, 0);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                }
                if (azimuth_trans == 0)
                {
                    this->last_point_ts_ = pkt_ts;
                    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_720f_[resolution_index][all_points_luminous_moment_720f_[resolution_index].size()-1];
                    this->cb_split_frame_(19, this->cloudTs());
                    ret = true;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        inline uint16_t DecoderVanjee720<T_PointCloud>::chanToline(uint16_t chan)
        {
            uint16_t line;
            switch (chan)
            {
            case 1:
                line = 8;
                break;

            case 2:
                line = 1;
                break;

            case 3:
                line = 2;
                break;

            case 4:
                line = 3;
                break;

            case 5:
                line = 4;
                break;

            case 6:
                line = 8;
                break;

            case 7:
                line = 5;
                break;

            case 8:
                line = 6;
                break;

            case 9:
                line = 7;
                break;

            case 10:
                line = 8;
                break;

            case 11:
                line = 9;
                break;

            case 12:
                line = 10;
                break;

            case 13:
                line = 11;
                break;

            case 14:
                line = 12;
                break;

            case 15:
                line = 8;
                break;

            case 16:
                line = 13;
                break;

            case 17:
                line = 14;
                break;

            case 18:
                line = 15;
                break;

            case 19:
                line = 16;
                break;
            }

            return line-1;
        }


        template <typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::SendImuData(Vanjee720Difop difop , double temperature , int azimuth , double timestamp)
        {
            int32 time = (difop.time_stamp[0] + (difop.time_stamp[1] << 8) + (difop.time_stamp[2] << 16) + ((difop.time_stamp[3] & 0x0F) << 24));
        
            bool l_get_flag = m_imu_params_get_->imuGet(difop.palstance_x , difop.palstance_y, difop.palstance_z, difop.linear_acceleration_x, difop.linear_acceleration_y, difop.linear_acceleration_z, temperature, time);
            if (l_get_flag)
            {
                this->imu_packet_->timestamp = timestamp;
                this->imu_packet_->angular_voc[0] = m_imu_params_get_->imu_result_stu_.x_angle;
                this->imu_packet_->angular_voc[1] = m_imu_params_get_->imu_result_stu_.y_angle;
                this->imu_packet_->angular_voc[2] = m_imu_params_get_->imu_result_stu_.z_angle;

                this->imu_packet_->linear_acce[0] = m_imu_params_get_->imu_result_stu_.x_add;
                this->imu_packet_->linear_acce[1] = m_imu_params_get_->imu_result_stu_.y_add;
                this->imu_packet_->linear_acce[2] = m_imu_params_get_->imu_result_stu_.z_add;

                this->imu_packet_->orientation[0] = m_imu_params_get_->imu_result_stu_.q0;
                this->imu_packet_->orientation[1] = m_imu_params_get_->imu_result_stu_.q1;
                this->imu_packet_->orientation[2] = m_imu_params_get_->imu_result_stu_.q2;
                this->imu_packet_->orientation[3] = m_imu_params_get_->imu_result_stu_.q3;

                this->cb_imu_pkt_();
            }
        }

        template<typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
        {
          std::shared_ptr<ProtocolAbstract720> p;
          std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

          if(*sp_cmd == *(CmdRepository720::CreateInstance()->sp_ld_angle_get_))
          {
            p = std::make_shared<Protocol_LDAngleGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->sp_imu_line_param_get_))
          {
            p = std::make_shared<Protocol_ImuLineGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->sp_imu_add_param_get_))
          {
            p = std::make_shared<Protocol_ImuAddGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->sp_temperature_param_get_))
          {
            p = std::make_shared<Protocol_ImuTempGet>();
          }
          else
          {
            return;
          }
          p->Load(*protocol);

          std::shared_ptr<ParamsAbstract> params = p->Params;
          if(typeid(*params) == typeid(Params_LDAngle720))
          {
            if(!this->param_.wait_for_difop)
            {
                if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
                {
                    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                }
                return;
            }

            std::shared_ptr<Params_LDAngle720> param = std::dynamic_pointer_cast<Params_LDAngle720>(params);

            std::vector<double> vert_angles;
            std::vector<double> horiz_angles;

            for (int line_id = 0; line_id < param->lines_num_; line_id++)
            {
              vert_angles.push_back((double)(param->ver_angle_[line_id] / 1000.0));
              horiz_angles.push_back((double)(0));
            }

            this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->lines_num_ , vert_angles , horiz_angles);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
              Decoder<T_PointCloud>::angles_ready_ = true;
            }
          }
          else if(typeid(*params) == typeid(Params_IMULine720))
          {
            if(!this->param_.wait_for_difop)
            {
                if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
                {
                    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                }
                return;
            }

            std::shared_ptr<Params_IMULine720> param = std::dynamic_pointer_cast<Params_IMULine720>(params);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<IMU> angular_vel data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            }

            this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path,this->imu_calibration_param_.TEMP_PARAM,
                                                        param->x_k_, param->x_b_, param->y_k_, param->y_b_, param->z_k_, param->z_b_);

            WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ << WJ_REND;

            m_imu_params_get_->setImuTempCalibrationParams(param->x_k_ , param->x_b_ , param->y_k_ , param->y_b_ , param->z_k_ , param->z_b_);

          }
          else if(typeid(*params) == typeid(Params_IMUAdd720))
          {
            if(!this->param_.wait_for_difop)
            {
                if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
                {
                    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                }
                return;
            }
            
            std::shared_ptr<Params_IMUAdd720> param = std::dynamic_pointer_cast<Params_IMUAdd720>(params);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<IMU> linear_acc data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            }

            this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path,this->imu_calibration_param_.ACC_PARAM,
                                                            param->x_k_,param->x_b_,param->y_k_ , param->y_b_,param->z_k_ , param->z_b_ );

            WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ <<WJ_REND;

            m_imu_params_get_->setImuAcceCalibrationParams(param->x_k_ , param->x_b_ , param->y_k_ , param->y_b_ , param->z_k_ , param->z_b_);
          }
          else if(typeid(*params) == typeid(Params_LiDARRunStatus))
          {
            std::shared_ptr<Params_LiDARRunStatus> param = std::dynamic_pointer_cast<Params_LiDARRunStatus>(params);

            WJ_INFOL << "imu_temp:" << param->imu_temp_ << WJ_REND;

            imu_temperature_ = param->imu_temp_ / 100.0f;            
          }
          else
          {
            WJ_WARNING << "Unknown Params Type..." << WJ_REND;
          }

        }

    } // namespace lidar

} // namespace vanjee
