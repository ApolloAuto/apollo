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
#ifndef LDW_DATA_INTERFACE_H_
#define LDW_DATA_INTERFACE_H_

enum LdwVersion
{
    LDW_VERSION_C1 = 0,
    LDW_VERSION_C2,
    LDW_VERSION_FOUR_LANE_C2
};

enum LdwLaneStyle
{
    LDW_LANE_STYLE_NONE_LANE = 0,
    LDW_LANE_STYLE_PREDICT_LANE,
    LDW_LANE_STYLE_BROKEN_LANE,
    LDW_LANE_STYLE_SOLID_LANE,
    LDW_LANE_STYLE_DOUBLE_BROKEN_LANE,
    LDW_LANE_STYLE_DOUBLE_SOLID_LANE,
    LDW_LANE_STYLE_TRIPLE_LANE,
};

enum LdwSteerStatus
{
    LDW_NORMAL_STEER = 0,
    LDW_STEER_ON_LEFT__LANE,
    LDW_STEER_ON_RIGHT_LANE,
    LDW_STEER_WARNING_LEFT_,
    LDW_STEER_WARNING_RIGHT,
};

enum LdwSoftStatus
{
    LDW_SOFT_DETECTION = 0,
    LDW_SOFT_SELF_LEARNING,
    LDW_SOFT_MANUAL_LEARNING_MODE0,
    LDW_SOFT_MANUAL_LEARNING_MODE1,
};

enum LdwWarningGrade
{
    LDW_WARNING_LOW = 0,
    LDW_WARNING_NORMAL,
    LDW_WARNING_HIGHT,
};

typedef struct LdwLaneBoundary__
{
    int degree;
    float coefficient[4];
}LdwLaneBoundary;

typedef struct LdwLane__
{
    int width;
    int qualityGrade;
    LdwLaneStyle style;
    LdwLaneBoundary left_Boundary;
    LdwLaneBoundary rightBoundary;
}LdwLane;

typedef struct LdwRoadway__
{
    int width[3];
    bool isTracking;
    LdwLane left_Lane;
    LdwLane rightLane;
    LdwLane adjacentLeft_Lane;
    LdwLane adjacentRightLane;
}LdwRoadway;

typedef struct LdwLensInfo__
{
    float xImageFocal;
    float yImageFocal;
    float xRatioFocalToPixel;
    float yRatioFocalToPixel;
    float mountingHeight;
    float mCosRx;
    float mSinRx;
    float mCosRy;
    float mSinRy;
}LdwLensInfo;

typedef struct LdwDataPack__
{
    LdwRoadway roadway;
    LdwSoftStatus softStatus;
    LdwSteerStatus steerStatus;
    LdwLensInfo lens;
}LdwDataPack;


#endif // LDW_DATA_INTERFACE_H_

