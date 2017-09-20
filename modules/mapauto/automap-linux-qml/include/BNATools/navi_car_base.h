#ifndef NAVI_CAR_BASE_H
#define NAVI_CAR_BASE_H

#define SEARCH_POINT_RATIO 100000.0


/** base define:point/rect/quadrangle */
//point
struct ADPos
{
    double x;    /** x */
    double y;    /** y */
};
typedef struct ADPos BNPos;

//point define(with define) //VPOINTF2
struct _APointF2
{
    float x;        // X
    float y;        // Y
    _APointF2(): x(0), y(0) {}
    _APointF2(float dx, float dy): x(dx), y(dy) {}
};
typedef struct _APointF2 BNPointF2;

//3d point define
struct ADPoint
{
    double x;
    double y;
    double z;
};
typedef struct ADPoint BNPoint;

//3d point define(with funciton) //VDPOINT3
struct _ADPoint3
{
    double x;       // X
    double y;       // Y
    double z;       // Z
    _ADPoint3(): x(0.0), y(0.0), z(0.0) {}
    _ADPoint3(double dx, double dy, double dz): x(dx), y(dy), z(dz) {}
};
typedef struct _ADPoint3 BNPoint3;


//Quadrangle define
struct ADQuadrangle
{
    ADPoint lb;
    ADPoint lt;
    ADPoint rt;
    ADPoint rb;
};
typedef struct ADQuadrangle BNQuadrangle;

//latlon
struct ADLatLon
{
    double longitude;
    double latitude;
};
typedef struct ADLatLon BNLatLon;


//rect define
struct ADRect
{
    int left;
    int right;
    int top;
    int bottom;
};
typedef struct ADRect BNRect;


//size
struct ADSize {
    float width;
    float height;
};
typedef struct ADSize BNSize;

 /** 地图状态结构信息 */
struct BNaviMapStatus
{
    double       fLevel;       // 比例尺，3－19级
    double       fRotation;    // 旋转角度
    double       fOverlooking; // 俯视角度
    BNPoint       ptCenter;     // 地图中心点
    BNQuadrangle  mapRound;     // 屏幕范围 屏幕地理坐标,注意：用户不需要更改此值
    BNRect        winRound;     // 屏幕范围 屏幕坐标,注意：用户不需要更改此值
    BNPoint       ptOffset;     // 偏移量
};


typedef enum _BNavi_Map_Animation_Type_Enum
{
    BNavi_Map_Animation_AnimationNone           = 0,
    BNavi_Map_Animation_AnimationPos            = 0x00000001,
    BNavi_Map_Animation_AnimationRotate         = 0x00000010,
    BNavi_Map_Animation_AnimationOverlook       = 0x00000100,
    BNavi_Map_Animation_AnimationLevel          = 0x00001000,
    BNavi_Map_Animation_AnimationAll            = 0x00001111,

    BNavi_Map_Animation_AnimationInelligent     = 0x10000000,
    BNavi_Map_Animation_AnimationFrogleap		= 0x10000001,
    BNavi_Map_Animation_AnimationFlyover		= 0x10000010,

}BNavi_Map_Animation_Type_Enum;

typedef struct _NA_Map_Mercator_Point
{
    int longitude;                              //墨卡托坐标:经度
    int latitude;                               //墨卡托坐标:纬度
}NA_Map_Mercator_Point;


/** 图区坐标点信息 */
typedef struct _NA_Map_PointInfo_t
{
    NA_Map_Mercator_Point       stPoint;        //坐标点
    unsigned int            	iId;            //id
}NA_Map_PointInfo_t;
#endif // NAVI_CAR_BASE_H
