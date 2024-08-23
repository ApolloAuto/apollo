/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/zvision/parser/parser.h"

const float SPEED_US = 0.299792458 / 2;

namespace apollo
{
    namespace drivers
    {
        namespace zvision
        {

            // 定义静态常量向量数据
            const std::vector<float> EZ6Parser::azi_comp_96_init{
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44
            };

            const std::vector<float> EZ6Parser::ele_comp_96_init{
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
                -8.698416867, -8.904511321, -9.111020356, -9.317953899, -9.525322152
            };

            const std::vector<float> EZ6Parser::azi_comp_192_init{
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44,
                -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44, -0.44,
                0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44
            };

            const std::vector<float> EZ6Parser::ele_comp_192_init{
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
                -8.852987707, -8.95613858, -9.059393098, -9.162753742, -9.266220513, -9.369795962, -9.473480089, -9.577164215
            };

            void EZ6Parser::GeneratePointcloud(
                const std::shared_ptr<ZvisionScan> &scan_msg,
                std::shared_ptr<PointCloud> out_msg)
            {
                // allocate a point cloud with same time and frame ID as raw data
                out_msg->mutable_header()->set_frame_id(scan_msg->header().frame_id());
                out_msg->set_height(1);
                out_msg->mutable_header()->set_sequence_num(
                    scan_msg->header().sequence_num());

                size_t packets_size = scan_msg->firing_pkts_size();
                for (size_t i = 0; i < packets_size; ++i)
                {
                    Unpack(scan_msg->firing_pkts(static_cast<int>(i)), out_msg);
                }

                if (out_msg->point().empty())
                {
                    // we discard this pointcloud if empty
                    // AERROR << "All points is NAN!Please check zvision:" << config_.model();
                }

                // set default width
                out_msg->set_width(out_msg->point_size());
            }

            /** @brief convert raw packet to point cloud
             *
             *  @param pkt raw packet to Unpack
             *  @param pc shared pointer to point cloud (points are appended)
             */
            void EZ6Parser::Unpack(const ZvisionPacket &pkt,
                                   std::shared_ptr<PointCloud> pc)
            {
                // const RawPacket* raw = (const RawPacket*)&pkt.data[0];
                // unsigned char *pdata = const_cast<unsigned char *>((unsigned char *)pkt.c_str());
                const unsigned char *pdata = (const unsigned char *)pkt.data().c_str();
                if (pkt.data().size() != 1058)
                {
                    // printf("Su pkt.data().size() != 1058, size = %d \n", pkt.data().size());

                    uint16_t packageCnt = ntohs(*((uint16_t *)(pdata + 10))) & 0xFFFF;
                    uint16_t DataRow = ntohs(*((uint16_t *)(pdata + 20))) & 0xFFFF; // 行 192
                    uint16_t DataCol = ntohs(*((uint16_t *)(pdata + 22))) & 0xFFFF; // 列 600
                    uint16_t frameCnt = ntohs(*((uint16_t *)(pdata + 4))) & 0xFFFF;

                    int BlockCnt = 4;
                    int pointCnt = 96;
                    int offset_v = 3 * 8;

                    float fov_v = 0;
                    std::map<int, float> angle_H_Map;

                    for (int i = 0; i < BlockCnt; i++)
                    {
                        int offset1 = 24 + (i * 320);
                        int AzimuthCnt = 4;
                        float angle_H = 0;

                        int blockFlag = 0;
                        int pointFlag = i / 2;
                        int colFlag = i % 2;

                        if (colFlag == 0)
                        {
                            blockFlag = 0;
                            angle_H_Map.clear();

                            for (int j = 0; j < AzimuthCnt; j++) // the first four
                            {
                                int offset2 = j * 3;
                                uint8_t pn = ((*(uint8_t *)(pdata + offset1 + offset2 + 0)) & 0x80) >> 7;
                                uint8_t zData = *(uint8_t *)(pdata + offset1 + offset2 + 0) & 0x7F;
                                uint8_t xData = *(uint8_t *)(pdata + offset1 + offset2 + 1) & 0xFF;

                                float xData_ = xData / 256.0;

                                float fov_h0 = (float)(zData + xData_);
                                if (pn == 1)
                                {
                                    fov_h0 = -fov_h0;
                                }
                                angle_H_Map.insert(std::pair<int, float>(j, fov_h0));
                            }

                            for (int j = 0; j < AzimuthCnt; j++)
                            {
                                int offset2 = j * 3;

                                uint8_t pn = ((*(uint8_t *)(pdata + offset1 + offset2 + 320 + (3 * 4) + 0)) & 0x80) >> 7;
                                uint8_t zData = *(uint8_t *)(pdata + offset1 + offset2 + 320 + (3 * 4) + 0) & 0x7F;
                                uint8_t xData = *(uint8_t *)(pdata + offset1 + offset2 + 320 + (3 * 4) + 1) & 0xFF;

                                float xData_ = xData / 256.0;

                                float fov_h0 = (float)(zData + xData_);
                                if (pn == 1)
                                {
                                    fov_h0 = -fov_h0;
                                }
                                angle_H_Map.insert(std::pair<int, float>(j + 4, fov_h0));
                            }

                            uint8_t elepn = ((*(uint8_t *)(pdata + offset1 + offset_v + 0)) & 0x80) >> 7;
                            uint8_t elezData = *(uint8_t *)(pdata + offset1 + offset_v + 0) & 0x7F;
                            uint8_t elexData = *(uint8_t *)(pdata + offset1 + offset_v + 1) & 0xFF;
                            float elexData_ = elexData / 256.0;
                            fov_v = (float)(elezData + elexData_);
                            if (elepn == 1)
                            {
                                fov_v = -fov_v;
                            }
                        }
                        else
                        {
                            blockFlag = 1;
                        }

                        int offset3 = 24 + 28 + (i * 320);
                        for (int n = 0; n < pointCnt; n++)
                        {
                            int offset4 = n * 3;

                            uint16_t distance0 = ntohs(*((uint16_t *)(pdata + offset3 + offset4 + 0))) & 0x7FFF;
                            uint8_t reflectivity = *((uint8_t *)(pdata + offset3 + offset4 + 2)) & 0xFF;
                            uint16_t retro_flag = ntohs(*((uint16_t *)(pdata + offset3 + offset4 + 0))) >> 15;

                            float distance = (float)distance0 / 16.0;

                            distance = distance * SPEED_US;

                            int key = n / 24;
                            // float getAngle_H = 0;
                            float getAngle_H = angle_H_Map[key];

                            float angledata = fov_v;
                            if (fov_v < -900) // 过滤不显示的镜面数据
                            {
                                continue;
                            }

                            if ((azi_comp_192_.size() != 0) && (ele_comp_192_.size() != 0))
                            {
                                getAngle_H += azi_comp_192_.at(96 * blockFlag + n);
                                angledata += ele_comp_192_.at(96 * blockFlag + n);
                            }

                            float azi = (float)getAngle_H / 180.0 * 3.1416;
                            float ele = (float)angledata / 180.0 * 3.1416;
                            int col = packageCnt * 2 + pointFlag;
                            int row = 96 * blockFlag + n;

                            PointXYZIT *point = pc->add_point();

                            float x = distance * cos(ele) * sin(azi);
                            float y = distance * cos(ele) * cos(azi);
                            float z = distance * sin(ele);
                            Eigen::Vector3d tp(x, y, z);
                            if (need_transform_)
                            {
                                tp = transform_ * tp;
                            }

                            point->set_x(static_cast<float>(tp.x()));
                            point->set_y(static_cast<float>(tp.y()));
                            point->set_z(static_cast<float>(tp.z()));
                            
                            // point->set_x(x);
                            // point->set_y(y);
                            // point->set_z(z);

                            point->set_intensity(reflectivity);
                            point->set_timestamp(GetExcitonTimestampNSSampleA(pdata));

                            // printf("Su size != 1058 ------>  x = %f , y = %f , z = %f , reflectivity = %d , timestamp = %d \n", x, y, z, reflectivity, GetExcitonTimestampNSSampleB(pdata));

                        }
                    }
                }
                else
                {
                    // printf("Su pkt.data().size() == 1058, size = %d \n", pkt.data().size());

                    int head_len = 18;

                    uint8_t sync_type = *((uint8_t *)(pdata + head_len + 0));
                    uint8_t sync_status = *((uint8_t *)(pdata + head_len + 1));

                    uint8_t block_num = *((uint8_t *)(pdata + head_len + 2));
                    uint8_t echo_type = *((uint16_t *)(pdata + head_len + 3));

                    uint16_t row = ntohs(*((uint16_t *)(pdata + head_len + 4)));
                    uint16_t column = ntohs(*((uint16_t *)(pdata + head_len + 6)));

                    

                    for (int i = 0; i < block_num; i++)
                    {
                        int blockhead_len = head_len + 8;

                        uint16_t slot_id = ntohs(*((uint16_t *)(pdata + blockhead_len + 0)));
                        uint16_t start_id = ntohs(*((uint16_t *)(pdata + blockhead_len + 2)));
                        uint8_t point_cnt = *((uint8_t *)(pdata + blockhead_len + 4));
                        uint8_t echo_num = *((uint8_t *)(pdata + blockhead_len + 5));

                        float fov_h_group[8] = {0};
                        for (int h = 0; h < 8; h++)
                        {
                            uint8_t sign = ((*(uint8_t *)(pdata + blockhead_len + 16 + h * 2)) & 0x80) >> 7;
                            uint8_t data_int = *(uint8_t *)(pdata + blockhead_len + 16 + h * 2) & 0x7F;
                            uint8_t data_dec = *(uint8_t *)(pdata + blockhead_len + 17 + h * 2) & 0xFF;

                            float data_dec_f = data_dec / 256.0;
                            float fov_h = data_int + data_dec_f;
                            if (sign)
                                fov_h = -fov_h;

                            fov_h_group[h] = fov_h;
                            //
                        }

                        uint8_t sign = ((*(uint8_t *)(pdata + blockhead_len + 48)) & 0x80) >> 7;
                        uint8_t data_int = *(uint8_t *)(pdata + blockhead_len + 48) & 0x7F;
                        uint8_t data_dec = *(uint8_t *)(pdata + blockhead_len + 49) & 0xFF;

                        float data_dec_f = data_dec / 256.0;
                        float fov_v = data_int + data_dec_f;
                        if (sign)
                            fov_v = -fov_v;

                        uint8_t side_flag = *((uint8_t *)(pdata + blockhead_len + 50));

                        for (int pt = 0; pt < point_cnt; pt++)
                        {
                            uint16_t distance = ntohs(*((uint16_t *)(pdata + blockhead_len + 64 + pt * 5)));
                            uint8_t reflectivity = *((uint8_t *)(pdata + blockhead_len + 66 + pt * 5));
                            uint8_t confidence = *((uint8_t *)(pdata + blockhead_len + 67 + pt * 5));
                            uint8_t Flag = *((uint8_t *)(pdata + blockhead_len + 68 + pt * 5));

                            int retro_flag = Flag & 0x1;
                            int dirty_flag = (Flag >> 1) & 0x1;
                            int dirty_grade = (Flag >> 2) & 0x3;

                            int area_num = pt / 24;

                            float distance_f = (float)distance / 16.0 * SPEED_US;
                            float fov_h_f = fov_h_group[area_num];

                            // std::cout << fov_h_f << std::endl;
                            float fov_v_f = fov_v;

                            if ((azi_comp_192_.size() != 0) && (ele_comp_192_.size() != 0))
                            {
                                fov_h_f += azi_comp_192_.at(pt);
                                fov_v_f += ele_comp_192_.at(pt);
                            }

                            float azi = fov_h_f / 180.0 * 3.1416;
                            float ele = fov_v_f / 180.0 * 3.1416;

                            PointXYZIT *point = pc->add_point();

                            float x = distance_f * cos(ele) * sin(azi);
                            float y = distance_f * cos(ele) * cos(azi);
                            float z = distance_f * sin(ele);
                            Eigen::Vector3d tp(x, y, z);
                            if (need_transform_)
                            {
                                tp = transform_ * tp;
                            }

#if 1           
                            point->set_x(static_cast<float>(tp.x()));
                            point->set_y(static_cast<float>(tp.y()));
                            point->set_z(static_cast<float>(tp.z()));
#else
                            point->set_x(x);
                            point->set_y(y);
                            point->set_z(z);
#endif

                            point->set_intensity(reflectivity);
                            point->set_timestamp(GetExcitonTimestampNSSampleB(pdata));

                            // printf("Su size = 1058  ------>  x = %f , y = %f , z = %f , reflectivity = %d , timestamp = %d \n", x, y, z, reflectivity, GetExcitonTimestampNSSampleB(pdata));
                        }
                    }
                }

                return;
            }

        } // namespace velodyne
    } // namespace drivers
} // namespace apollo
