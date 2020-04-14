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
