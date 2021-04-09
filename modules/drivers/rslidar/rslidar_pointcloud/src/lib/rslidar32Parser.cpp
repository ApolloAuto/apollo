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
	
void rslidar32Parser::loadConfigFile(ros::NodeHandle private_nh) {

std::string anglePath, curvesPath, channelPath, curvesRatePath;
	
private_nh.param("curves_path", curvesPath, std::string(""));
private_nh.param("angle_path", anglePath, std::string(""));
private_nh.param("channel_path", channelPath, std::string(""));
private_nh.param("curves_rate_path", curvesRatePath, std::string(""));
TEMPERATURE_RANGE = 50;


/// 读参数文件 2018-02-27
FILE *f_inten = fopen(curvesPath.c_str(), "r");
int loopi = 0;
int loopj = 0;

if (!f_inten) {
	ROS_ERROR_STREAM(curvesPath << " does not exist");
} else {
	while (!feof(f_inten)) {
		float a[32];
		loopi++;
		if (loopi > 7)
			break;

		fscanf(f_inten,
			   "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			   &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
			   &a[13],
			   &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
			   &a[25], &a[26], &a[27],
			   &a[28], &a[29], &a[30], &a[31]);

			for (loopj = 0; loopj < 32; loopj++) {
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
		float b[32], d[32];
		int loopk = 0;
		int loopn = 0;
		while (!feof(f_angle)) {
			fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
			loopk++;
			if (loopk > 31) break;
		}
		for (loopn = 0; loopn < 32; loopn++) {
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
		int c[51];
		int tempMode = 1;
		while (!feof(f_channel)) {

			fscanf(f_channel,
				   "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
				   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
				   &c[13], &c[14], &c[15],
				   &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26], &c[27],
				   &c[28], &c[29], &c[30],
				   &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40],
				   &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48], &c[49], &c[50]);

			for (loopl = 0; loopl < TEMPERATURE_RANGE+1; loopl++) {
				g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
			}
			loopm++;
			if (loopm > 31) {
				break;
			}
		}
		fclose(f_channel);
	}


	FILE *f_curvesRate = fopen(curvesRatePath.c_str(), "r");
	if (!f_curvesRate) {
		ROS_ERROR_STREAM(curvesRatePath << " does not exist");
	} else {
		int loopk = 0;
		while (!feof(f_curvesRate)) {
			fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
			loopk++;
			if (loopk > (numOfLasers - 1)) break;
		}
		fclose(f_curvesRate);
	}



}

/** Set up for on-line operation. */
void rslidar32Parser::init_setup() {
	pic.col = 0;
	pic.distance.resize(RS32_DATA_NUMBER_PER_SCAN);
	pic.intensity.resize(RS32_DATA_NUMBER_PER_SCAN);
	pic.azimuthforeachP.resize(RS32_DATA_NUMBER_PER_SCAN);

}



int rslidar32Parser::correctAzimuth(float azimuth_f, int passageway) {
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



void rslidar32Parser::unpack(const rslidar_msgs::rslidarPacket &pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
						  bool finish_packets_parse) {
	float azimuth;	//0.01 dgree
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
			if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
				continue;
			}
		} else {
			int azi1, azi2;
			azi1 = 256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2;
			azi2 = 256 * raw->blocks[block - 1].rotation_1 + raw->blocks[block - 1].rotation_2;
			azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);

			//Ingnore the block if the azimuth change abnormal
			if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
				continue;
			}
		}

		//Estimate the type of packet
		union two_bytes tmp_flag;
		tmp_flag.bytes[1] = raw->blocks[block].data[0];
		tmp_flag.bytes[0] = raw->blocks[block].data[1];
		int ABflag = calibration_->isABPacket(tmp_flag.uint);

		int k = 0;
		int index;
		for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)//16	 3
		{
			if (ABflag == 1 && dsr < 16) {
				index = k + 48;
			} else if (ABflag == 1 && dsr >= 16) {
				index = k - 48;
			} else {
				index = k;
			}

			int point_count = pic.col * SCANS_PER_BLOCK + dsr;
			int dsr_temp;
			if (dsr >= 16) { dsr_temp = dsr - 16; }
			else { dsr_temp = dsr; }
			azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
			azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);
			pic.azimuthforeachP[point_count] = azimuth_corrected;

			union two_bytes tmp;
			tmp.bytes[1] = raw->blocks[block].data[index];
			tmp.bytes[0] = raw->blocks[block].data[index + 1];
			int ab_flag_in_block = calibration_->isABPacket(tmp.uint);
			int distance = tmp.uint - ab_flag_in_block * 32768;

			// read intensity
			intensity = (float) raw->blocks[block].data[index + 2];
			intensity = calibration_->calibrateIntensity(intensity, dsr, distance);

			float distance2 = calibration_->pixelToDistance(distance, dsr);
			distance2 = distance2 * DISTANCE_RESOLUTION;

			pic.distance[point_count] = distance2;
			pic.intensity[point_count] = intensity;
		}
		//pic.azimuth[pic.col] = azimuth;
		pic.col++;
	}

	if (finish_packets_parse) {
		// ROS_INFO_STREAM("***************: "<<pic.col);
		pointcloud->clear();
		pointcloud->height = RS32_SCANS_PER_FIRING;
		pointcloud->width = pic.col;
		pointcloud->is_dense = false;
		pointcloud->resize(pointcloud->height * pointcloud->width);
		for (int block_num = 0; block_num < pic.col; block_num++) {

			for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++) {
				int point_count = block_num * SCANS_PER_BLOCK + dsr;
				float dis = pic.distance[point_count];
				float arg_horiz = pic.azimuthforeachP[point_count] / 18000 * M_PI;
				float intensity = pic.intensity[point_count];
				float arg_vert = VERT_ANGLE[dsr];
				pcl::PointXYZI point;
				if (dis > DISTANCE_MAX || dis < DISTANCE_MIN)  //invalid data
				{
					// ROS_INFO_STREAM("***************: "<<dis);
					point.x = NAN;
					point.y = NAN;
					point.z = NAN;
					point.intensity = 0;
					pointcloud->at(block_num, dsr) = point;
				} else {
					//If you want to fix the rslidar Y aixs to the front side of the cable, please use the two line below
					//point.x = dis * cos(arg_vert) * sin(arg_horiz);
					//point.y = dis * cos(arg_vert) * cos(arg_horiz);

					//If you want to fix the rslidar X aixs to the front side of the cable, please use the two line below
					point.y = -dis * cos(arg_vert) * sin(arg_horiz);
					point.x = dis * cos(arg_vert) * cos(arg_horiz);
					point.z = dis * sin(arg_vert);
					point.intensity = intensity;
					pointcloud->at(block_num, dsr) = point;
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
