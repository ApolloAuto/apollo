/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "rslidar_pointcloud/rslidarParser.h"

#include <pcl/common/time.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "rslidar_pointcloud/util.h"

namespace apollo {
namespace drivers {
namespace rslidar {

	rslidar16Parser::rslidar16Parser() {}
	
	void rslidar16Parser::loadConfigFile(ros::NodeHandle private_nh) {
	
		std::string anglePath, curvesPath, channelPath, curvesRatePath;
		//std::string model;
	
		private_nh.param("curves_path", curvesPath, std::string(""));
		private_nh.param("angle_path", anglePath, std::string(""));
		private_nh.param("channel_path", channelPath, std::string(""));
	
		/// 读参数文件 2018-02-27
		FILE *f_inten = fopen(curvesPath.c_str(), "r");
		int loopi = 0;
		int loopj = 0;
	
		if (!f_inten) {
			ROS_ERROR_STREAM(curvesPath << " does not exist");
		} else {
			while (!feof(f_inten)) {
				float a[16];
				loopi++;
				if (loopi > 7)
					break;

				fscanf(f_inten, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
					   &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
					   &a[13],
					   &a[14], &a[15]);

				for (loopj = 0; loopj < 16; loopj++) {
					aIntensityCal[loopi - 1][loopj] = a[loopj];
				}
			}
				fclose(f_inten);
		}
		//=============================================================
		FILE *f_angle = fopen(anglePath.c_str(), "r");
		if (!f_angle) {
			ROS_ERROR_STREAM(anglePath << " does not exist");
		} else {
			float b[16], d[16];
			int loopk = 0;
			int loopn = 0;
			while (!feof(f_angle)) {
				fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
				loopk++;
				if (loopk > 15) break;
			}
			for (loopn = 0; loopn < 16; loopn++) {
				VERT_ANGLE[loopn] = b[loopn] / 180 * M_PI;
				HORI_ANGLE[loopn] = d[loopn] * 100;
			}
			fclose(f_angle);
		}
	
		//=============================================================
		FILE *f_channel = fopen(channelPath.c_str(), "r");
		if (!f_channel) {
			ROS_ERROR_STREAM(channelPath << " does not exist");
		} else {
			int loopl = 0;
			int loopm = 0;
			int c[41];
			int tempMode = 1;
			while (!feof(f_channel)) {

				fscanf(f_channel,
					   "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
					   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
					   &c[13], &c[14], &c[15],
					   &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26], &c[27],
					   &c[28], &c[29], &c[30],
					   &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40]);

				for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++) {
					g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
				}
				loopm++;
				if (loopm > 15) {
					break;
				}
			}
			fclose(f_channel);
		}

	}
	
	/** Set up for on-line operation. */
	void rslidar16Parser::init_setup() {
		pic.col = 0;
		pic.distance.resize(RS16_DATA_NUMBER_PER_SCAN);
		pic.intensity.resize(RS16_DATA_NUMBER_PER_SCAN);
		pic.azimuthforeachP.resize(RS16_DATA_NUMBER_PER_SCAN);
	}
	
		
	
	int rslidar16Parser::correctAzimuth(float azimuth_f, int passageway) {
		int azimuth;
		if (azimuth_f > 0.0 && azimuth_f < 3000.0) {
			azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
		} else {
			azimuth_f = azimuth_f + HORI_ANGLE[passageway];
		}
		azimuth = (int)azimuth_f;
		azimuth %= 36000;
	
		return azimuth;
	}
	
	/** @brief convert raw packet to point cloud
	 *
	 *	@param pkt raw packet to unpack
	 *	@param pc shared pointer to point cloud (points are appended)
	 */
	void rslidar16Parser::unpack(const rslidar_msgs::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
							bool finish_packets_parse) {

		float azimuth;    //0.01 dgree
		float intensity;
		float azimuth_diff;
		float azimuth_corrected_f;
		int azimuth_corrected;

		const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[42];

		for (int block = 0; block < BLOCKS_PER_PACKET; block++) //1 packet:12 data blocks
		{

			if (UPPER_BANK != raw->blocks[block].header) {
				ROS_INFO_STREAM_THROTTLE(180, "skipping RSLIDAR DIFOP packet");
				break;
			}

			if (tempPacketNum < 20000 && tempPacketNum > 0)//update temperature information per 20000 packets
			{
				tempPacketNum++;
			} else {
				temper = calibration_->computeTemperature(pkt.data[38], pkt.data[39]);
				//ROS_INFO_STREAM("Temp is: " << temper);
				tempPacketNum = 1;
			}

			azimuth = (float) (256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);

			if (block < (BLOCKS_PER_PACKET - 1))//12
			{
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block + 1].rotation_1 + raw->blocks[block + 1].rotation_2;
				azi2 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
				azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);

				//Ingnore the block if the azimuth change abnormal
				if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
					continue;
				}

			} else {
				int azi1, azi2;
				azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
				azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
				azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);

				//Ingnore the block if the azimuth change abnormal
				if (azimuth_diff <= 0.0 || azimuth_diff > 75.0) {
					continue;
				}

			}

			for (int firing = 0, k = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++)//2
			{
				for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)//16	3
				{
					int point_count = pic.col * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
					azimuth_corrected_f =
							azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) /
									   RS16_BLOCK_TDURATION);
					azimuth_corrected = ((int) round(azimuth_corrected_f)) % 36000;//convert to integral value...
					pic.azimuthforeachP[point_count] = azimuth_corrected;

					union two_bytes tmp;
					tmp.bytes[1] = raw->blocks[block].data[k];
					tmp.bytes[0] = raw->blocks[block].data[k + 1];
					int distance = tmp.uint;

					// read intensity
					intensity = raw->blocks[block].data[k + 2];
					intensity = calibration_->calibrateIntensity(intensity, dsr, distance);

					float distance2 = calibration_->pixelToDistance(distance, dsr);
					distance2 = distance2 * DISTANCE_RESOLUTION;

					pic.distance[point_count] = distance2;
					pic.intensity[point_count] = intensity;
				}
			}
			//pic.azimuth[pic.col] = azimuth;
			pic.col++;
		}

		if (finish_packets_parse) {
			// ROS_INFO_STREAM("***************: "<<pic.col);
			pointcloud->clear();
			pointcloud->height = RS16_SCANS_PER_FIRING;
			pointcloud->width = 2 * pic.col;
			pointcloud->is_dense = false;
			pointcloud->resize(pointcloud->height * pointcloud->width);
			for (int block_num = 0; block_num < pic.col; block_num++) {

				for (int firing = 0; firing < RS16_FIRINGS_PER_BLOCK; firing++) {
					for (int dsr = 0; dsr < RS16_SCANS_PER_FIRING; dsr++) {
						int point_count = block_num * SCANS_PER_BLOCK + dsr + RS16_SCANS_PER_FIRING * firing;
						float dis = pic.distance[point_count];
						float arg_horiz = pic.azimuthforeachP[point_count] / 18000 * M_PI;
						float arg_vert = VERT_ANGLE[dsr];
						pcl::PointXYZI point;
						if (dis > DISTANCE_MAX || dis < DISTANCE_MIN)  //invalid data
						{
							point.x = NAN;
							point.y = NAN;
							point.z = NAN;
							point.intensity = 0;
							pointcloud->at(2 * block_num + firing, dsr) = point;
						} else {
							//If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
							//point.x = dis * cos(arg_vert) * sin(arg_horiz);
							//point.y = dis * cos(arg_vert) * cos(arg_horiz);

							//If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
							point.y = -dis * cos(arg_vert) * sin(arg_horiz);
							point.x = dis * cos(arg_vert) * cos(arg_horiz);
							point.z = dis * sin(arg_vert);
							point.intensity = pic.intensity[point_count];
							pointcloud->at(2 * block_num + firing, dsr) = point;

						}
					}
				}
			}
			init_setup();
			pic.header.stamp = pkt.stamp;
		}
	}
	
}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
