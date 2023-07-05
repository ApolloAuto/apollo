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
#ifndef _OBSTACLE_DATA_H_
#define _OBSTACLE_DATA_H_

enum RecognitionType
{
    INVALID = 0,
    VEHICLE,
    PEDESTRIAN,
    CHILD,
    BICYCLE,
    MOTO,
    TRUCK,
    BUS,
    OTHERS,
    ESTIMATED,
    CONTINUOUS
};

struct OutputObstacles
{
    float currentSpeed;                 //(m/second) current frame self vehicle speed
    float frameRate;                    //(fps) frames per second

    unsigned char trackId;              //current obstacle corresponding tracking id in obstacle tracking buffer
    unsigned char trackFrameNum;        //the track frame numbers of the obstacle, increments in 1 each frame until to the max record number, the actual number is (trackFrameNum + 1)
    unsigned char stateLabel;           //classify obstacle for front collision, FC, 0: invalid or continuous obstacle, 1: nearest o  bstacle in warning area, 2: obstacle in waning area, 3: obstacle out of warning area
    unsigned char classLabel;           //the obstacle class label: 0-invalid; 1-warning obstacle; 2-obstacles to be warned; 3-non warning obstacle; 4-left continuous obstacle; 5-right continuous obstacle; 6-estimated vanish obstacle; 7-valid obstacle // original, the obstacle class label: 0-invalid; 1-car; 2-person; 3-continuous obstacle; 4-valid; 5-other
    unsigned char continuousLabel;      //the continuous obstacle class label: 0-invalid; 1-left continuous obstacle; 2-right continuous obstacle
    unsigned char fuzzyEstimationValid; //(0/1) 0: current fuzzy estimation is invalid; 1: current fuzzy estimation is valid
    RecognitionType obstacleType;       //the obstacle Type: INVALID=0,VEHICLE, PEDESTRIAN, ...

    float avgDisp;                      //(pixel) the average disparity of an obstacle with adding infDisp: avgDisp = BF_VALUE/avgDitance+infDis
    float avgDistanceZ;                 //(m) the average Z distance of single obstacle rectangle
    float nearDistanceZ;                //(m) the minimum Z distance of continuous obstacle
    float farDistanceZ;                 //(m) the longest Z distance of continuous obstacle

    float real3DLeftX;                  //(-+m) the left X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
    float real3DRightX;                 //(-+m) the right X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
    float real3DCenterX;                //(-+m) the center X for real 3D coordinate of the obstacle(the origin X is the center of car, right is positive)
    float real3DUpY;                    //(-+m) the up Y for real 3D coordinate of the obstacle(the origin Y is the camera position, down is positive)
    float real3DLowY;                   //(-+m) the Low y for real 3D coordinate of the obstacle(the origin Y is the camera position, down is positive)

    unsigned short firstPointX;         //(pixel) the X-axis of first point of rectangle, the first point :(x, y), left top point of single obstacle/near bottom point of continuous obstacle, full size pixel coordinate
    unsigned short firstPointY;         //(pixel) the Y-axis of first point of rectangle, the first point :(x, y), left top point of single obstacle/near bottom point of continuous obstacle, full size pixel coordinate
    unsigned short secondPointX;        //(pixel) the X-axis of second point of rectangle, the second point:(x+width, y), right top point of single obstacle/near top point of continuous obstacle, full size pixel coordinate
    unsigned short secondPointY;        //(pixel) the Y-axis of second point of rectangle, the second point:(x+width, y), right top point of single obstacle/near top point of continuous obstacle, full size pixel coordinate
    unsigned short thirdPointX;         //(pixel) the X-axis of third point of rectangle, the third point :(x+width, y+height), right bottom point of single obstacle/far top point of continuous obstacle, full size pixel coordinate
    unsigned short thirdPointY;         //(pixel) the Y-axis of third point of rectangle, the third point :(x+width, y+height), right bottom point of single obstacle/far top point of continuous obstacle, full size pixel coordinate
    unsigned short fourthPointX;        //(pixel) the X-axis of fourth point of rectangle, the fourth point:(x,y+height), left bottom point of single obstacle/far bottom point of continuous obstacle, full size pixel coordinate
    unsigned short fourthPointY;        //(pixel) the Y-axis of fourth point of rectangle, the fourth point:(x,y+height), left bottom point of single obstacle/far bottom point of continuous obstacle, full size pixel coordinate

    float fuzzyRelativeDistanceZ;       //(m) estimated relative distance in Z direction
    float fuzzyRelativeSpeedZ;          //(m/second) estimated speed in Z direction of current obstacle
    float fuzzyCollisionTimeZ;          //(second) estimated collision time in Z direction

    unsigned char fuzzyCollisionX;      //(0/1) estimated whether there is collision in X direction
    float fuzzy3DWidth;                 //(m) estimated real 3D width of current obstacle
    float fuzzy3DCenterX;               //(-+m) estimated real 3D position of obstacle center in X direction (the origin X is the center of car, right is positive)
    float fuzzy3DLeftX;                 //(-+m) estimated real 3D position of obstacle left in X direction (the origin X is the center of car, right is positive)
    float fuzzy3DRightX;                //(-+m) estimated real 3D position of obstacle right in X direction (the origin X is the center of car, right is positive)
    float fuzzy3DHeight;                //(m) estimated real 3D height of current obstacle
    float fuzzy3DUpY;                   //(-+m) estimated real 3D position of obstacle up in Y direction (the origin Y is the camera position, down is positive)
    float fuzzy3DLowY;                  //(-+m) estimated real 3D position of obstacle low in Y direction (the origin Y is the camera position, down is positive)

    float fuzzyRelativeSpeedCenterX;    //(m/second) estimated center speed in X direction of current obstacle
    float fuzzyRelativeSpeedLeftX;      //(m/second) estimated left speed in X direction of current obstacle
    float fuzzyRelativeSpeedRightX;     //(m/second) estimated right speed in X direction of current obstacle

#ifdef EXTEND_INFO_ENABLE
    unsigned char storeId;              //current obstacle store id in obstacle detection buffer
#endif //EXTEND_INFO_ENABLE
};

#endif // _OBSTACLE_DATA_H_
