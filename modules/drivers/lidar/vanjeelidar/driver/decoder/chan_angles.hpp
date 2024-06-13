
#pragma once

#include <vanjeelidar/common/wj_common.hpp>
#include <vanjeelidar/common/wj_log.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

namespace vanjee
{
namespace lidar
{
#pragma pack(push, 1)
        typedef struct
        {
            uint8_t sign;
            uint16_t value;
        }WJSCalibrationAngle;
#pragma pack(pop)
        class ChanAngles
        {
        private:
            uint16_t chan_num_;
            std::vector<int32_t> vert_angles_;
            std::vector<int32_t> horiz_angles_;
            std::vector<vector<int32_t>> horiz_vert_angles_; // 水平垂直补偿角度
            std::vector<uint16_t> user_chans_;

        public:
            ChanAngles(uint16_t chan_num)
                : chan_num_(chan_num)
            {
                vert_angles_.resize(chan_num_);
                horiz_angles_.resize(chan_num_);
                horiz_vert_angles_.resize(chan_num_ * 3); // 144 线 每线 1200个补偿角度
                for(int i = 0; i< horiz_vert_angles_.size();i++)
                {
                    horiz_vert_angles_[i].resize(1200);
                }
                user_chans_.resize(chan_num_);
            }

            /// <summary>
            /// 字符串拆分
            /// </summary>
            /// <param name="strSur">需要进行拆分的字符串</param>
            /// <param name="cConChar">拆分字符</param>
            /// <returns>返回拆分后的各个子字符串</returns>
            std::vector<std::string> vStrSplit(std::string strSur, char cConChar)
            {
                std::vector<std::string> vStrVec;//向量容器
                std::string::size_type pos1, pos2;//记录字符串出现位置
                pos1 = 0;
                pos2 = strSur.find(cConChar, 0);//从0位置开始查找出现字符串cConChar的位置
                //查找到字符串
                while (std::string::npos != pos2)
                {
                    //将从pos1位置开始，长pos2 - pos1 的 字符串 插入vStrVec
                    vStrVec.push_back(strSur.substr(pos1, pos2 - pos1));
                    pos1 = pos2 + 1;
                    pos2 = strSur.find(cConChar, pos1);
                }
                // 将从某位置开始的剩余字符串 插入 vStrVec
                vStrVec.push_back(strSur.substr(pos1));
                return vStrVec;
            }

            int loadFromFile(const std::string &angle_path)
            {
                std::vector<int32_t> vert_angles;
                std::vector<int32_t> horiz_angles;
                int ret = loadFromFile(angle_path, chan_num_, vert_angles, horiz_angles);
                if (ret < 0)
                    return ret;

                if (vert_angles.size() != chan_num_)
                {
                    return -1;
                }

                vert_angles_.swap(vert_angles);
                horiz_angles_.swap(horiz_angles);
                genUserChan(vert_angles_, user_chans_);
                return 0;
            }

            int loadFromFile(const std::string &vangle_path, const std::string &hangle_path)
            {
                std::vector<int32_t> vert_angles;
                std::vector<::vector<int32_t>>horiz_vert_angles;
                int ret = loadFromFile(vangle_path, hangle_path, chan_num_, vert_angles, horiz_vert_angles);
                if (ret < 0)
                    return ret;

                if (vert_angles.size() != chan_num_)
                {
                    return -1;
                }

                vert_angles_.swap(vert_angles);
                horiz_vert_angles_.swap(horiz_vert_angles);
                genUserChan(vert_angles_, user_chans_);
                return 0;
            }

            int loadFromLiDAR(const std::string &angle_path,int num,std::vector<double> vert_angles,
                                    std::vector<double> horiz_angles)
            {
                for(size_t i =0; i< num; i++)
                {
                    vert_angles_[i] = (static_cast<int32_t>(vert_angles[i] * 1000));
                    horiz_angles_[i] = (static_cast<int32_t>(horiz_angles[i] * 1000));
                }

                std::ofstream fd(angle_path.c_str(), std::ios::out);
                if (!fd.is_open())
                {
                    WJ_WARNING << "fail to open angle file:" << angle_path << WJ_REND;
                    return -1;
                }

                std::string line;
                for (size_t i = 0; i < num; i++)
                {
                    if (horiz_angles[0] != 0)
                    {
                        fd << std::to_string(vert_angles[i])<<","<<std::to_string(horiz_angles[i])<<"\n";
                    }
                    else
                    {
                        fd << std::to_string(vert_angles[i])<<","<<std::to_string(horiz_angles_[i])<<"\n";
                    }
                    
                }

                fd.close();
                return 0;
            }

            /// @brief 从给出的雷达内部通道编号，得到用户通道编号
            uint16_t toUserChan(uint16_t chan)
            {
                return user_chans_[chan];
            }
            /// @brief 对参数给出的水平角作修正
            int32_t horizAdjust(uint16_t chan, int32_t horiz)
            {
                return (horiz + horiz_angles_[chan]);
            }
            int32_t vertAdjust(uint16_t chan)
            {
                return vert_angles_[chan];
            }
            int32_t horiz_vertAdjust(uint16_t chan, int32_t horiz)
            {
                return (horiz_vert_angles_[chan][horiz]);
            }
        private:
            /// @brief 根据成员变量`vert_angles_[]`中的角度值，计算升序排列的用户通道编号数组
            void genUserChan(const std::vector<int32_t> &vert_angles, std::vector<uint16_t> &user_chans)
            {
                user_chans.resize(vert_angles.size());

                for (size_t i = 0; i < vert_angles.size(); i++)
                {
                    int32_t angle = vert_angles[i];
                    uint16_t chan = 0;

                    for (size_t j = 0; j < vert_angles.size(); j++)
                    {
                        if (vert_angles[j] < angle)
                        {
                            chan++;
                        }
                    }

                    user_chans[i] = chan;
                }
            }
            int loadFromFile(const std::string &angle_path, size_t size, std::vector<int32_t> &vert_angles, std::vector<int32_t> &horiz_angles)
            {
                vert_angles.clear();
                horiz_angles.clear();

                std::ifstream fd(angle_path.c_str(), std::ios::in);
                if (!fd.is_open())
                {
                    WJ_WARNING << "fail to open vangle file:" << angle_path << WJ_REND;
                    return -1;
                }

                std::string line;
                for (size_t i = 0; i < size; i++)
                {
                    try
                    {
                        if (!std::getline(fd, line))
                            return -1;

                        float vert = std::stof(line);

                        float horiz = 0;
                        size_t pos_comma = line.find_first_of(',');
                        if (pos_comma != std::string::npos)
                        {
                            horiz = std::stof(line.substr(pos_comma + 1));
                        }

                        vert_angles.emplace_back(static_cast<int32_t>(vert * 1000));
                        horiz_angles.emplace_back(static_cast<int32_t>(horiz * 1000));
                    }
                    catch(...)
                    {
                        WJ_ERROR << "The format of angle config file " << angle_path
                        << " is wrong. Please check (e.g. indentation)." << WJ_REND;
                    }
                
                }
                fd.close();
                return 0;
            }

            int loadFromFile(const std::string &vangle_path,const std::string &hangle_path, size_t size, std::vector<int32_t> &vert_angles,
                                    std::vector<std::vector<int32_t>> &horiz_vert_angles)
            {
                vert_angles.clear();
                horiz_vert_angles.clear();

                horiz_vert_angles.resize(size * 3); // 144 线 每线 1200个补偿角度
                for(int i = 0; i < horiz_vert_angles.size(); i++)
                {
                    horiz_vert_angles[i].resize(1200);
                }

                std::ifstream fd_v(vangle_path.c_str(), std::ios::in);
                std::ifstream fd_h(hangle_path.c_str(), std::ios::in);
                
                if (!fd_v.is_open())
                {
                    WJ_WARNING << "fail to open vangle file:" << vangle_path << WJ_REND;
                    return -1;
                }
                if (!fd_h.is_open())
                {
                    WJ_WARNING << "fail to open hangle file:" << hangle_path << WJ_REND;
                    return -1;
                }

                std::string line;
                for (size_t i = 0; i < size; i++)
                {
                    if (!std::getline(fd_v, line))
                        return -1;

                    float vert = std::stof(line);
                    vert_angles.emplace_back(static_cast<int32_t>(vert * 1000));
                }
                
                int rowidx = 0; // 行索引
                std::string h_line;
                //std::vector<int32_t> h_p_pointNum;
                while (std::getline(fd_h, h_line))
                {
                    std::stringstream sinFile(h_line);
                    std::vector<std::string> LineData = ChanAngles::vStrSplit(sinFile.str(),',');
                    float horiz = 0.f;
                    int ch = ((rowidx + 1) % 3) == 0 ? (rowidx + 1) / 3 : (rowidx + 1) / 3 + 1; // 线号转通道号
                    int i = 0;
                    int hvidx = 1199;
                    
                    for(int i = 0; i < 1200;i++)
                    {
                        double v = 0;
                        double VAngle = 0;
                        switch((rowidx + 1) % 3)
                        {
                            case 0:
                            v = std::stod(LineData[hvidx]) * 1000;
                            VAngle = vert_angles[ch - 1] + v;
                            horiz_vert_angles[rowidx][i] = VAngle;
                            break;
                            
                            case 1:
                            v = std::stod(LineData[hvidx]) * 1000;
                            VAngle = vert_angles[ch - 1] + v;
                            horiz_vert_angles[rowidx+1][i] = VAngle;
                            break;

                            case 2:
                            v = std::stod(LineData[hvidx]) * 1000;
                            VAngle = vert_angles[ch - 1] + v;
                            horiz_vert_angles[rowidx-1][i] = VAngle;
                            break;;
                        }
                        hvidx--;
                    }

                    rowidx++;
                }
                fd_v.close();
                fd_h.close();
                return 0;
            }
            
            bool angleCheck(int32_t v)
            {
                return ((-9000 <= v) && (v < 9000));
            }
        };

} 
} 
