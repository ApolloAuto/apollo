#include "rslidar_pointcloud/rslidarParser.h"
#include <pcl/common/time.h>
#include <ros/package.h>
#include <ros/ros.h>

namespace apollo {
namespace drivers {
namespace rslidar {

//------------------------------------------------------------
//校准反射强度值
float calibration_parse::calibrateIntensity(float intensity, int calIdx, int distance) {
	int algDist;
	int sDist;
	int uplimitDist;
	float realPwr;
	float refPwr;
	float tempInten;
	float distance_f;
	float endOfSection1;

	int temp = estimateTemperature(temper);

	realPwr = std::max( (float)( intensity / (1+(temp-TEMPERATURE_MIN)/24.0f) ), 1.0f );
	
	if ((int) realPwr < 126)
		realPwr = realPwr * 4.0f;
	else if ((int) realPwr >= 126 && (int) realPwr < 226)
		realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
	else
		realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

	int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
	uplimitDist = g_ChannelNum[calIdx][indexTemper] + 20000;
	//limit sDist
	sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
	sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
	//minus the static offset (this data is For the intensity cal useage only)
	algDist = sDist - g_ChannelNum[calIdx][indexTemper];

	// calculate intensity ref curves
	float refPwr_temp = 0.0f;
	int order = 3;
	endOfSection1 = 500.0f; 
	distance_f = (float)algDist;
	if(distance_f <= endOfSection1)
	{
	  refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - 
	  aIntensityCal[2][calIdx] * distance_f/100.0f) + aIntensityCal[3][calIdx];
	}
	else
	{
	  for(int i = 0; i < order; i++)
	  {
		refPwr_temp +=aIntensityCal[i+4][calIdx]*(pow(distance_f/100.0f,order-1-i));
	  }
	}

	refPwr = std::max(std::min(refPwr_temp,500.0f),4.0f);

	tempInten = (51* refPwr) / realPwr;
	if(numOfLasers == 32){
		tempInten = tempInten * CurvesRate[calIdx];
	}
	tempInten = (int) tempInten > 255 ? 255.0f : tempInten;
	return tempInten;
}

float calibration_parse::pixelToDistance(int pixelValue, int passageway) {
	float DistanceValue;
	int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
	if (pixelValue <= g_ChannelNum[passageway][indexTemper]) {
		DistanceValue = 0.0;
	} else {
		DistanceValue = (float) (pixelValue - g_ChannelNum[passageway][indexTemper]);
	}
	return DistanceValue;
}

int calibration_parse::isABPacket(int distance) {
	int ABflag = 0;
	if ((distance & 32768) != 0) {
		ABflag = 1; // B
	} else {
		ABflag = 0;// A
	}
	return ABflag;
}

//------------------------------------------------------------
float calibration_parse::computeTemperature(
		unsigned char bit1, unsigned char bit2) {
	float Temp;
	float bitneg = bit2 & 128;//10000000
	float highbit = bit2 & 127;//01111111
	float lowbit = bit1 >> 3;
	if (bitneg == 128) {
		Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
	} else {
		Temp = (highbit * 32 + lowbit) * 0.0625f;
	}

	return Temp;
}

int calibration_parse::estimateTemperature(float Temper) {
	int temp = (int)floor(Temper + 0.5);
	if (temp < TEMPERATURE_MIN) {
		temp = TEMPERATURE_MIN;
	} else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
		temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
	}
	
	return temp;
	}
}
}
}
