/******************************************************************************
 * Copyright 2020 The Beijing Smarter Eye Technology Co.Ltd Authors. All
 * Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef DISPARITYPAINTER_H
#define DISPARITYPAINTER_H

#include "dataprocessordef.h"

class DATAPROCESSOR_SHARED_EXPORT DisparityConvertor
{
public:
    DisparityConvertor();
    ~DisparityConvertor();

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * dest: Output, disparity buffer, float type for each point.
    */
    static void convertDisparity2FloatFormat(const unsigned char* src, int width, int height, int bitNum, float* dest);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * posX: Input, point x coordinate, unit: pixel
     * posY: Input, point y coordinate, unit: pixel
     * return: disparity value
    */
    static float getPointDisparityValue(const unsigned char *src, int width, int height, int bitNum, int posX, int posY);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters, unit: mm
     * focus: Input, camera focus that can be get from StereoCalibrationParameters, unit: pixel
     * cx: Input, horizental optical center that can be get from StereoCalibrationParameters, unit: pixel
     * cy: Input, vertical optical center that can be get from StereoCalibrationParameters, unit: pixel
     * posX: Input, point x coordinate, unit: pixel
     * posY: Input, point y coordinate, unit: pixel
     * xDistance: Output, lateral distance value, unit: mm(+-)
     * yDistance: Output, vertical distance value, unit: mm(+-)
     * zDistance: Output, longitudinal distance value, unit: mm
    */
    static void getPointXYZDistance(const unsigned char *src, int width, int height, int bitNum,
                                    float baseline, float focus, int cx, int cy,
                                    int posX, int posY, float &xDistance, float &yDistance, float &zDistance);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters, unit: mm
     * focus: Input, camera focus that can be get from StereoCalibrationParameters, unit: pixel
     * posX0: Input, the first point of rect x coordinate, unit: pixel
     * posY0: Input, the first point of rect point y coordinate, unit: pixel
     * posX1: Input, the forth point of rect point x coordinate, unit: pixel
     * posY1: Input, the forth point of rect point y coordinate, unit: pixel
     * zDistance: Output, longitudinal distance value, unit: mm
    */
    static void getRectZDistance(const unsigned char *src, int width, int height, int bitNum,
                                    float baseline, float focus,
                                    int posX0, int posY0, int posX1, int posY1, float &zDistance);

    /*
     * disparity: Input, disparity buffer, float type for each point.
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters, unit: mm
     * cx: Input, horizental optical center that can be get from StereoCalibrationParameters, unit: pixel
     * xDistance: Output, lateral distance value, unit: mm(+-)
    */
    static void getWholeXDistance(const float* disparity, int width, int height, float baseline, int cx, float* xDistance);

    /*
     * disparity: Input, disparity buffer, float type for each point.
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters, unit: mm
     * cy: Input, vertical optical center that can be get from StereoCalibrationParameters, unit: pixel
     * yDistance: Output, vertical distance value, unit: mm(+-)
    */
    static void getWholeYDistance(const float* disparity, int width, int height, float baseline, int cy, float* yDistance);

    /*
     * disparity: Input, disparity buffer, float type for each point.
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters, unit: mm
     * focus: Input, camera focus that can be get from StereoCalibrationParameters, unit: pixel
     * zDistance: Output, longitudinal distance value, unit: mm
    */
    static void getWholeZDistance(const float* disparity, int width, int height, float baseline, float focus, float* zDistance);

    /*
     * width: Input, disparity width, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters
     * cx: Input, horizental optical center that can be get from StereoCalibrationParameters
     * lookUpTableX: output, 81*2^bitNum*width float buffer, should be allocated before used
    */
    static void generateLookUpTableX(int width, int bitNum, float baseline, int cx, float* lookUpTableX);

    /*
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters
     * cy: Input, vertical optical center that can be get from StereoCalibrationParameters
     * lookUpTableY: output, 81*2^bitNum*height float buffer, should be allocated before used
    */
    static void generateLookUpTableY(int height, int bitNum, float baseline, int cy, float* lookUpTableY);

    /*
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * baseline: Input, dual camera baseline that can be get from StereoCalibrationParameters
     * focus: Input, camera focus that can be get from StereoCalibrationParameters
     * lookUpTableZ: output, 81*2^bitNum float buffer
    */
    static void generateLookUpTableZ(int bitNum, float baseline, float focus, float* lookUpTableZ);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * lookUpTableX: input, can be get by generateLookUpTableX()
     * distanceX: output, the whole pixel X distance buffer, size: width*height, unit: mm(+-)
    */
    static void getWholeXDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableX, float* distanceX);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * bitNum: Input, bit number of disparity float part, based on FrameFormat
     * lookUpTableY: input, can be get by generateLookUpTableY()
     * distanceY: output, the whole pixel Y distance buffer, size: width*height, unit: mm(+-)
    */
    static void getWholeYDistanceByLookupTable(const unsigned char* src, int width, int height, int bitNum, float* lookUpTableY, float* distanceY);

    /*
     * src: Input, dispairy buffer from ADAS device
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * lookUpTableZ: input, can be get by generateLookUpTableZ()
     * distanceZ: output, the whole pixel Z distance buffer, size: width*height, unit: mm
    */
    static void getWholeZDistanceByLookupTable(const unsigned char* src, int width, int height, float* lookUpTableZ, float* distanceZ);

    /*
     * disparity: Input, disparity buffer, float type for each point.
     * width: Input, disparity width, unit: pixel
     * height: Input, disparity height, unit: pixel
     * minDisp: Input, min disparity value
     * maxDisp: Input, max disparity value
     * rgbBuf: Output, rgb disparity buffer, can be showed with color
    */
    static void convertDisparity2RGB(const float* disparity, int width, int height, int minDisp, int maxDisp, unsigned char* rgbBuf);

    /*
     * format: Input, frame format.
     * return: bit number of disparity float part.
    */
    static int getDisparityBitNum(int format);

};

#endif // DISPARITYPAINTER_H
