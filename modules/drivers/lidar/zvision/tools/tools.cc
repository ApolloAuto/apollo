/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "tcp_client.h"
#include "tools.h"
#include <set>
#include <math.h>
#include <cstring>
#include <iostream>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>

#include "modules/drivers/lidar/zvision/tools/tcp_client.h"
#include "modules/drivers/lidar/proto/zvision.pb.h"

namespace apollo
{
    namespace drivers
    {
        namespace zvision
        {

            bool LidarTools::CheckDeviceRet(std::string ret)
            {
                return (0x00 == ret[2]) && (0x00 == ret[3]);
            }

            int LidarTools::GetOnlineCalibrationData(std::string ip, CalibrationData &cal)
            {
                // std::string ec;
                // const int ppf = 256000; // points per frame, 256000 reserved
                const int ppk = 128; // points per cal udp packet
                // std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation);
                // int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
                // std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
                const int send_len = 4;
                char cal_cmd[send_len] = {(char)0xBA, (char)0x07, (char)0x00, (char)0x00};
                std::string cmd(cal_cmd, send_len);

                const int recv_len = 4;
                std::string recv(recv_len, 'x');

                cal.model = UNKNOWN;
                TcpClient client(1000, 1000, 1000);
                int ret = client.Connect(ip);
                if (ret)
                {
                    AERROR << "Connect error: " << client.GetSysErrorCode();
                    return -1;
                }

                if (client.SyncSend(cmd, send_len))
                {
                    AERROR << "SyncSend error: " << client.GetSysErrorCode();
                    client.Close();
                    return -1;
                }

                if (client.SyncRecv(recv, recv_len))
                {
                    AERROR << "SyncRecv error: " << client.GetSysErrorCode();
                    client.Close();
                    return -1;
                }

                if (!CheckDeviceRet(recv))
                {
                    AERROR << "CheckDeviceRet error: " << client.GetSysErrorCode();
                    client.Close();
                    return -1;
                }

                const int cal_pkt_len = 1040;
                std::string cal_recv(cal_pkt_len, 'x');

                // receive first packet to identify the device type
                if (client.SyncRecv(cal_recv, cal_pkt_len))
                {
                    client.Close();
                    return -1;
                }

                int total_packet = 0;

                std::string dev_code((char *)cal_recv.c_str() + 3, 6);
                uint32_t data_size = 0;
                if (0 == dev_code.compare("30_B1 "))
                {
                    total_packet = 235;
                    // data_size = (10000 * 3 * 2);
                    data_size = ppk * total_packet * 2; // packet must give a full
                    cal.model = ML30B1;
                }
                else if (0 == dev_code.compare("30S_A1"))
                {
                    total_packet = 400;
                    data_size = (6400 * 8 * 2);
                    cal.model = ML30SA1;
                }
                else
                {
                    AERROR << "Calibration packet identify error";
                    client.Close();
                    return -1;
                }

                // check the data
                unsigned char *check_data = (unsigned char *)cal_recv.c_str();
                unsigned char check_all_00 = 0x00;
                unsigned char check_all_ff = 0xFF;
                for (int i = 0; i < 1040 - 16; i++)
                {
                    check_all_00 |= check_data[i];
                    check_all_ff &= check_data[i];
                }
                if (0x00 == check_all_00)
                {
                    AERROR << ("Check calibration data error, data is all 0x00.\n");
                    client.Close();
                    return -1;
                }
                if (0xFF == check_all_ff)
                {
                    AERROR << ("Check calibration data error, data is all 0xFF.\n");
                    client.Close();
                    return -1;
                }
                // printf("Check data ok.");
                std::vector<float> &data = cal.data;
                {
                    int network_data = 0;
                    int host_data = 0;
                    float *pfloat_data = reinterpret_cast<float *>(&host_data);
                    for (int i = 0; i < 128 * 2; i++)
                    {
                        memcpy(&network_data, check_data + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                        host_data = ntohl(network_data);
                        data.push_back(*pfloat_data);
                    }
                }

                for (int i = 0; i < total_packet - 1; i++)
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(110));
                    int ret = client.SyncRecv(cal_recv, cal_pkt_len);
                    if (ret)
                    {
                        AERROR << ("Receive calibration data error\n");
                        client.Close();
                        return -1;
                    }
                    check_data = (unsigned char *)cal_recv.c_str();
                    {
                        int network_data = 0;
                        int host_data = 0;
                        float *pfloat_data = reinterpret_cast<float *>(&host_data);
                        for (int i = 0; i < 128 * 2; i++)
                        {
                            memcpy(&network_data, check_data + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                            host_data = ntohl(network_data);
                            data.push_back(*pfloat_data);
                        }
                    }
                }

                if (client.SyncRecv(recv, recv_len))
                {
                    printf("Recv ack error.\n");
                    client.Close();
                    return -1;
                }
                // printf("recv ack ok.\n");
                if (!CheckDeviceRet(recv))
                {
                    printf("Check ack error.\n");
                    client.Close();
                    return -1;
                }

                if (data_size != data.size())
                {
                    printf("Calbration data size [%6lu] is not valid, [%6u] wanted.\n", data.size(), data_size);
                    return -1;
                }
                // printf("check ack ok, cal type is %d, first data is %.3f %.3f.\n", cal.model, data[0], cal.data[1]);
                AINFO << ("Get calibration data ok.\n");
                return 0;
            }

            int LidarTools::ReadCalibrationFile(std::string filename, CalibrationData &cal)
            {
                std::ifstream file;
                file.open(filename, std::ios::in);
                std::string line;
                int ret = 0;
                cal.model = UNKNOWN;
                if (file.is_open())
                {
                    std::vector<std::vector<std::string>> lines;
                    while (std::getline(file, line))
                    {
                        if (line.size() > 0)
                        {
                            std::istringstream iss(line);
                            std::vector<std::string> datas;
                            std::string data;
                            while (iss >> data)
                            {
                                datas.push_back(data);
                            }
                            lines.push_back(datas);
                        }
                    }
                    file.close();
                    if (10000 == lines.size())
                    {
                        cal.data.resize(60000);
                        for (int i = 0; i < 10000; i++)
                        {
                            const int column = 7;

                            std::vector<std::string> &datas = lines[i];
                            if (datas.size() != column)
                            {
                                ret = -1;
                                AERROR << "Resolve calibration file data error.";
                                break;
                            }
                            for (int j = 1; j < column; j++)
                            {
                                int fov = (j - 1) % 3;
                                if (0 == ((j - 1) / 3)) // azimuth
                                {
                                    cal.data[i * 6 + fov * 2] = static_cast<float>(std::atof(datas[j].c_str()));
                                }
                                else // elevation
                                {
                                    cal.data[i * 6 + fov * 2 + 1] = static_cast<float>(std::atof(datas[j].c_str()));
                                }
                            }
                        }
                        cal.model = ML30B1;
                    }
                    else if (6400 == lines.size())
                    {
                        cal.data.resize(6400 * 8 * 2);
                        for (int i = 0; i < 6400; i++)
                        {
                            const int column = 17;

                            std::vector<std::string> &datas = lines[i];
                            if (datas.size() != column)
                            {
                                ret = -1;
                                AERROR << "Resolve calibration file data error.";
                                break;
                            }
                            for (int j = 1; j < column; j++)
                            {
                                cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                            }
                        }
                        cal.model = ML30SA1;
                    }
                    else if (32000 == lines.size())
                    {
                        cal.data.resize(32000 * 3 * 2);
                        for (int i = 0; i < 32000; i++)
                        {
                            const int column = 7;

                            std::vector<std::string> &datas = lines[i];
                            if (datas.size() != column)
                            {
                                ret = -1;
                                AERROR << "Resolve calibration file data error.";
                                break;
                            }
                            for (int j = 1; j < column; j++)
                            {
                                cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                            }
                        }
                        cal.model = MLX;
                    }
                    else
                    {
                        cal.model = UNKNOWN;
                        ret = -1;
                        AERROR << "Invalid calibration file length.";
                    }

                    return ret;
                }
                else
                {
                    AERROR << "Open calibration file error.";
                    return -1;
                }
            }

            void LidarTools::ComputeCalibrationData(CalibrationData &cal, PointCalibrationTable &cal_lut)
            {
                cal_lut.data.resize(cal.data.size() / 2);
                cal_lut.model = cal.model;
                if (ML30B1 == cal.model)
                {
                    for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
                    {
                        float azi = static_cast<float>(cal.data[i * 2] / 180.0 * 3.1416);
                        float ele = static_cast<float>(cal.data[i * 2 + 1] / 180.0 * 3.1416);

                        PointCalibrationData &point_cal = cal_lut.data[i];
                        point_cal.ele = ele;
                        point_cal.azi = azi;
                        point_cal.cos_ele = std::cos(ele);
                        point_cal.sin_ele = std::sin(ele);
                        point_cal.cos_azi = std::cos(azi);
                        point_cal.sin_azi = std::sin(azi);
                    }
                }
                else if (ML30SA1 == cal.model)
                {
                    const int start = 8;
                    int fov_index[start] = {0, 6, 1, 7, 2, 4, 3, 5};
                    for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
                    {
                        int start_number = i % start;
                        int group_number = i / start;
                        int point_numer = group_number * start + fov_index[start_number];
                        float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                        float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                        PointCalibrationData &point_cal = cal_lut.data[i];
                        point_cal.ele = ele;
                        point_cal.azi = azi;
                        point_cal.cos_ele = std::cos(ele);
                        point_cal.sin_ele = std::sin(ele);
                        point_cal.cos_azi = std::cos(azi);
                        point_cal.sin_azi = std::sin(azi);
                    }
                }
                else if (MLX == cal.model)
                {
                    const int start = 3;
                    int fov_index[start] = {2, 1, 0};
                    for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
                    {
                        int start_number = i % start;
                        int group_number = i / start;
                        int point_numer = group_number * start + fov_index[start_number];
                        float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                        float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                        PointCalibrationData &point_cal = cal_lut.data[i];
                        point_cal.ele = ele;
                        point_cal.azi = azi;
                        point_cal.cos_ele = std::cos(ele);
                        point_cal.sin_ele = std::sin(ele);
                        point_cal.cos_azi = std::cos(azi);
                        point_cal.sin_azi = std::sin(azi);
                    }
                }
                else if (EZ6 == cal.model)
                {
                    const std::vector<float> azi_comp_96{
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44};

                    const std::vector<float> ele_comp_96{
                        9.525285573, 9.317917481, 9.110984094, 8.904475209, 8.698380899, 8.492691473, 8.287397425,
                        8.082489408, 7.877958198, 7.673794669, 7.469989773, 7.266534515, 7.063419901, 6.860637087,
                        6.658177115, 6.456031074, 6.254190047, 6.052645105, 5.851387299, 5.650407658, 5.449697184,
                        5.249246846, 5.049047585, 4.849090303, 4.649365896, 4.449865141, 4.250578862, 4.051497817,
                        3.85261273, 3.653914287, 3.455393142, 3.257039913, 3.058845188, 2.860799521, 2.66289344,
                        2.465117442, 2.267461986, 2.069917548, 1.872474537, 1.675123357, 1.477854393, 1.280658013,
                        1.083524569, 0.886444402, 0.689407839, 0.4924052, 0.295426797, 0.098462937, -0.098496077,
                        -0.295459941, -0.492438351, -0.689441, -0.886477575, -1.083557758, -1.28069122, -1.477887622,
                        -1.67515661, -1.872507817, -2.069950858, -2.26749533, -2.465150793, -2.662926827, -2.860832948,
                        -3.058878657, -3.257073428, -3.455426705, -3.653947902, -3.852646399, -4.051531544, -4.250612649,
                        -4.449898992, -4.649399814, -4.849124347, -5.049081704, -5.249281044, -5.449731463, -5.650442022,
                        -5.851421751, -6.052679649, -6.254224686, -6.456065811, -6.658211954, -6.860672031, -7.063454954,
                        -7.266569596, -7.470024968, -7.673829982, -7.877993633, -8.082524969, -8.287433117, -8.4927273,
                        -8.698416867, -8.904511321, -9.111020356, -9.317953899, -9.525322152};

                    const std::vector<float> azi_comp_192{
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                        -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                        0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44};

                    const std::vector<float> ele_comp_192{
                        9.577127596, 9.47344355, 9.369759504, 9.266184135, 9.162717441, 9.059356873, 8.95610243, 8.852951631, 8.749904477, 8.646958543, 8.544113829, 8.441367961,
                        8.338720937, 8.236170421, 8.133716412, 8.031356605, 7.929091, 7.826917316, 7.724835552, 7.622843445, 7.520940997, 7.419125958, 7.31739833, 7.215755862,
                        7.114198554, 7.012724197, 6.91133279, 6.810022094, 6.708792108, 6.607640605, 6.506567584, 6.405570817, 6.304650304, 6.203803812, 6.10303134, 6.002330653,
                        5.90170175, 5.801142389, 5.700652568, 5.600230039, 5.499874802, 5.399584599, 5.299359431, 5.199197031, 5.0990974, 4.999058264, 4.899079624, 4.799159201,
                        4.699296998, 4.599490708, 4.49974033, 4.400043572, 4.300400432, 4.200808601, 4.101268078, 4.001776545, 3.902334002, 3.802938119, 3.703588898, 3.604284001,
                        3.505023428, 3.405804835, 3.30662822, 3.207491232, 3.108393869, 3.009333771, 2.910310938, 2.811323001, 2.71236996, 2.613449441, 2.514561442, 2.415703578,
                        2.31687585, 2.218075877, 2.119303657, 2.020556795, 1.921835289, 1.823136742, 1.724461152, 1.625806116, 1.527171634, 1.428555298, 1.329957108, 1.231374652,
                        1.13280793, 1.034254528, 0.935714444, 0.837185261, 0.73866698, 0.64015718, 0.54165586, 0.443160599, 0.344671398, 0.246185832, 0.147703902, 0.049223184,
                        -0.049256323, -0.147737043, -0.246218975, -0.344704543, -0.443193748, -0.541689013, -0.640190338, -0.738700144, -0.837218432, -0.935747621, -1.034287713,
                        -1.132841124, -1.231407855, -1.329990321, -1.428588521, -1.527204869, -1.625839363, -1.724494411, -1.823170015, -1.921868577, -2.020590098, -2.119336976,
                        -2.218109212, -2.316909196, -2.415736927, -2.514594802, -2.613482819, -2.712403358, -2.811356418, -2.910344375, -3.00936723, -3.10842735, -3.207524735,
                        -3.306661747, -3.405838386, -3.505057004, -3.604317603, -3.703622526, -3.802971775, -3.902367685, -4.001810258, -4.10130182, -4.200842373, -4.300434235,
                        -4.400077406, -4.499774197, -4.599524608, -4.699330947, -4.799193213, -4.899113686, -4.999092365, -5.099131539, -5.199231209, -5.299393648, -5.399618858,
                        -5.499909102, -5.600264382, -5.700686954, -5.801176819, -5.901736225, -6.002365174, -6.103065908, -6.203838427, -6.304684967, -6.40560553, -6.506602347,
                        -6.607675418, -6.708826973, -6.810057012, -6.911367762, -7.012759224, -7.114233615, -7.215790935, -7.317433439, -7.419161125, -7.520976221, -7.622878729,
                        -7.724870895, -7.82695272, -7.929126467, -8.031392135, -8.133752006, -8.23620608, -8.338756663, -8.441403754, -8.544149692, -8.646994475, -8.74994048,
                        -8.852987707, -8.95613858, -9.059393098, -9.162753742, -9.266220513, -9.369795962, -9.473480089, -9.577164215};
                }
                else
                {
                }
            }

        }
    }
}
