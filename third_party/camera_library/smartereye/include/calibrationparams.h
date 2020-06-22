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
#ifndef _CALIBRATIONPARAMS_H_
#define _CALIBRATIONPARAMS_H_

//Stereo camera’s intrinsic and extrinsic params.
//The params is suitable after stereo camera calibration and left/right image remap.

struct StereoCalibrationParameters
{
    double focus;                       //(pixel) the camere focus in pixel. The focus value is all the same include left and right camera.

    double cx;                          //(pixel) optical axis X center point. The optical axis center point(X, Y) is all the same include left and right camera
    double cy;                          //(pixel) optical axis Y center point.

    double RRoll;                       //(rad) R-vector include roll, pitch, yaw.
    double RPitch;
    double RYaw;

    double Tx;                          //(mm) Translation matrix.
    double Ty;
    double Tz;
};

struct MonoCalibrationParameters
{
    double fx;                          //(pixel) the axis X focus in pixel.
    double fy;                          //(pixel) the axis Y focus in pixel.

    double cx;                          //(pixel) optical axis X center point. The optical axis center point(X, Y) is all the same include left and right camera
    double cy;                          //(pixel) optical axis Y center point.

    double k1;                          //Radial distortion coefficient
    double k2;
    double k3;

    double p1;                          //Tangential distortion coefficient
    double p2;
};

#endif //_CALIBRATIONPARAMS_H_
