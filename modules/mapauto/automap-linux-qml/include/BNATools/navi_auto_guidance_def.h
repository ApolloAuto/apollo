#ifndef NAVI_AUTO_GUIDANCE_DEF_H
#define NAVI_AUTO_GUIDANCE_DEF_H

#include "navi_car_adapter_def.h"

NAMESPACE_BAIDU
NAMESPACE_BAIDU_ADAPTER

USING_NAMESPACE_ADAPTER

#define BNAGuidance_LOG

#define BNA_GUIDANCE_FILENAME_MAX_LENGTH 32
#define BNA_GUIDANCE_ROADNAME_MAX_LENGTH 32
#define BNA_GUIDANCE_DISTRICT_MAX_LENGTH 32
#define BNA_GUIDANCE_ADDRESS_MAX_LENGTH 128
#define BNA_GUIDEANCE_HUD_MAX_LANE_COUNT 16
#define BNA_GUIDANCE_VIEWTAG_LENGTH  15
#define BNA_GUIDANCE_LINE_CNT 6
#define BNA_GUIDANCE_RASTER_MAP_BUFFER "ccos.navi.guide.ilsimageviewinfo.buffer"
#define BNA_GUIDANCE_LOCATION_START_CRUISE  20
#define BNA_GUIDANCE_LOCATION_START_CRUISE_SPEED  5

//position
typedef struct _BNAPosPoint
{
    double x;
    double y;
}BNAPosPoint;

//day or night
typedef enum _BNAGuidanceDayNightMode
{
    BNA_GUIDANCE_DAY_NIGHT_MODE_AUTO   = 1,
    BNA_GUIDANCE_DAY_NIGHT_MODE_DAY      = 4,
    BNA_GUIDANCE_DAY_NIGHT_MODE_NIGHT = 5,
}BNAGuidanceDayNightMode;

//speak
typedef enum _BNAGuidanceSpeakMode
{
    BNA_GUIDANCE_SPEAK_MODE_PRO,
    BNA_GUIDANCE_SPEAK_MODE_MUTE,
    BNA_GUIDANCE_SPEAK_MODE_NEWBIE
}BNAGuidanceSpeakMode;

//speak content
typedef enum _BNAGuidanceSpeakContent
{
    BNA_SPEAK_CONTENT_EEYE                                        = 1 << 0,
    BNA_SPEAK_CONTENT_RATE_LIMITING                      = 1 << 1,
    BNA_SPEAK_CONTENT_SAFE_DRIVING                        = 1 << 2,
    BNA_SPEAK_CONTENT_AHEAD_ROAD_CONDITION   = 1 << 3,
    BNA_SPEAK_CONTENT_STRAIGHT_REMIND                = 1 << 4
}BNAGuidanceSpeakContent;

//gps status
typedef enum _BNAGPSStateType
{
    BNA_GPS_STATE_CLOSE      = 0,
    BNA_GPS_STATE_FIXED       = 1,
    BNA_GPS_STATE_NOFIXED = 2
}BNAGPSStateType;

//net status
typedef enum _BNANetStatus
{
    BNA_NET_STATUS_INVALID,
    BNA_NET_STATUS_NO_NET,
    BNA_NET_STATUS_WIFI,
    BNA_NET_STATUS_WWAN,
}BNANetStatus;

//guidance config
typedef struct _BNAGuidanceConfig
{
    BNANetStatus netStatus;
    BNAGuidanceDayNightMode dayNightMode;
    BNAGuidanceSpeakMode    speakMode;
    BNAGuidanceSpeakContent speakContent;
    BNAGPSStateType  gpsStateType;
}BNAGuidanceConfig;

//guide status
typedef enum _BNAGuideStatus
{
    BNA_GUIDE_STATUS_INVALID,
    BNA_GUIDE_STATUS_TRACK,
    BNA_GUIDE_STATUS_REALGUIDE,
    BNA_GUIDE_STATUS_DEMOGUIDE,
    BNA_GUIDE_STATUS_CRUISE,
    BNA_GUIDE_STATUS_LIGHTNAVI,
    BNA_GUIDE_STATUS_GENERICGUIDE
}BNAGuideStatus;

//guide sub status
typedef enum _BNAGuideSubStatus
{
    BNA_GUIDE_STATUS_TYPE_INVALID,
    BNA_GUIDE_STATUS_TYPE_BEGIN,
    BNA_GUIDE_STATUS_TYPE_YAWING,
    BNA_GUIDE_STATUS_TYPE_REROUTEEND,
    BNA_GUIDE_STATUS_TYPE_REROUTECARFREE,
    BNA_GUIDE_STATUS_TYPE_END1,
    BNA_GUIDE_STATUS_TYPE_END2,
    BNA_GUIDE_STATUS_TYPE_BUILDROUTE,
    BNA_GUIDE_STATUS_TYPE_ROUTECHANGING,
    BNA_GUIDE_STATUS_TYPE_ROUTECHANGINGEND
}BNAGuideSubStatus;

//all guide status
typedef struct _BNAGuideStatusS
{
    BNAGuideStatus enStatus;
    BNAGuideSubStatus enSubStatus;
}BNAGuideStatusS;

//locate mode
typedef enum _BNAGuideLocateMode
{
    BNA_GUIDE_LOCATE_MODE_INVALID,
    BNA_GUIDE_LOCATE_MODE_GPS,
    BNA_GUIDE_LOCATE_MODE_ROUTEDEMOGPS,
    BNA_GUIDE_LOCATE_MODE_NEMADEMOGPS,
    BNA_LGUIDE_OCATE_MODE_MANUALDEMOGPS,
}BNAGuideLocateMode;

//layer mode
typedef enum _BNAGuideLayerMode
{
    BNA_GUIDE_LAYER_MODE_GUIDE,
    BNA_GUIDE_LAYER_MODE_ROUTECRUISE,
    BNA_GUIDE_LAYER_MODE_MAP
}BNAGuideLayerMode;

//map view status
typedef enum _BNAGuideViewStatus
{
    BNA_GUIDE_VIEW_STATUS_VIEW_ALL,
    BNA_GUIDE_VIEW_STATUS_BROWSE,
    BNA_GUIDE_VIEW_STATUS_FLLOW_CAR_POSITION
}BNAGuideViewStatus;

typedef enum _BNAGuideViewRotateMode
{
    BNA_GUIDE_VIEW_ROTATE_MODE_MAP,  // 车头朝上，地图动
    BNA_GUIDE_VIEW_ROTATE_MODE_CAR,  // 正北朝上，车标动
}BNAGuideViewRotateMode;

//guide map update type
typedef enum _BNAGuideMapUpdateType
{
    BNA_GUIDEMAP_UPDATE_INVALID,
    BNA_GUIDEMAP_UPDATE_SHOW,
    BNA_GUIDEMAP_UPDATE_UPDATE,
    BNA_GUIDEMAP_UPDATE_HIDE,
}BNAGuideMapUpdateType;

//maneuver type
typedef enum _BNAGuideManeuverType
{
    BNA_GUIDE_MANEUVER_TYPE_INVALID ,
    BNA_GUIDE_MANEUVER_TYPE_FRONT ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_FRONT ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_BACK ,
    BNA_GUIDE_MANEUVER_TYPE_BACK ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_BACK ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_FRONT ,
    BNA_GUIDE_MANEUVER_TYPE_RING ,
    BNA_GUIDE_MANEUVER_TYPE_RING_OUT ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_SIDE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_SIDE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_SIDE_MAIN ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_LEFT_MAIN ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_SIDE_MAIN ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_RIGHT_MAIN,
    BNA_GUIDE_MANEUVER_TYPE_CENTER_MAIN ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_SIDE_IC ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_SIDE_IC ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_LEFT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_RIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_CENTER ,
    BNA_GUIDE_MANEUVER_TYPE_START ,
    BNA_GUIDE_MANEUVER_TYPE_DEST ,
    BNA_GUIDE_MANEUVER_TYPE_VIA1 ,
    BNA_GUIDE_MANEUVER_TYPE_VIA2 ,
    BNA_GUIDE_MANEUVER_TYPE_VIA3 ,
    BNA_GUIDE_MANEUVER_TYPE_VIA4 ,
    BNA_GUIDE_MANEUVER_TYPE_IN_FERRY ,
    BNA_GUIDE_MANEUVER_TYPE_OUT_FERRY ,
    BNA_GUIDE_MANEUVER_TYPE_TOLL_GATE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT_IC ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT_IC ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_LEFT_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_CENTER_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_RIGHT_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_LEFT_IC ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_CENTER_IC ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_RIGHT_IC ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_LEFT_IC_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_CENTER_IC_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_BRANCH_RIGHT_IC_STRAIGHT ,
    BNA_GUIDE_MANEUVER_TYPE_STRAIGHT_2BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_STRAIGHT_2BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_STRAIGHT_3BRANCH_LEFT_BASE  ,
    BNA_GUIDE_MANEUVER_TYPE_STRAIGHT_3BRANCH_MIDDLE_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_STRAIGHT_3BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_2BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_2BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_3BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_3BRANCH_MIDDLE_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_3BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_2BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_2BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_3BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_3BRANCH_MIDDLE_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_3BRANCH_RIGHT_BASE,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_LEFT_BASE,
    BNA_GUIDE_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_RIGHT_BASE,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_LEFT_BASE,
    BNA_GUIDE_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_RIGHT_BASE,
    BNA_GUIDE_MANEUVER_TYPE_BACK_2BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_BACK_2BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_BACK_3BRANCH_LEFT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_BACK_3BRANCH_MIDDLE_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_BACK_3BRANCH_RIGHT_BASE ,
    BNA_GUIDE_MANEUVER_TYPE_RING_FRONT,
    BNA_GUIDE_MANEUVER_TYPE_RING_RIGHT_FRONT,
    BNA_GUIDE_MANEUVER_TYPE_RING_RIGHT,
    BNA_GUIDE_MANEUVER_TYPE_RING_RIGHT_BACK,
    BNA_GUIDE_MANEUVER_TYPE_RING_LEFT_BACK,
    BNA_GUIDE_MANEUVER_TYPE_RING_LEFT,
    BNA_GUIDE_MANEUVER_TYPE_RING_LEFT_FRONT,
}BNAGuideManeuverType;

/** lane item info */
typedef struct _BNAGuidanceLineItem
{
    bool   bIsAdd;
    bool   bIsSub;
    bool   bIsBusLane;

    bool   bFront;
    bool   bFrontIsBright;
    bool   bLeft;
    bool   bLeftIsBright;
    bool   bRight;
    bool   bRightIsBright;
    bool   bBack;
    bool   bBackIsBright;
}BNAGuidanceLineItem;

/** lane info:refer to engine: NE_LaneInfo_t */
typedef struct _BNAGuidanceLaneInfo
{
    BNAGuidanceLineItem      stLanes[BNA_GUIDANCE_LINE_CNT];                               /**< laneInfo, from left to right*/
    unsigned int                       unLaneCnt;                                                                           /**< line Count*/
    int                                        nStartDist;                                                                             /**< start distance, meter */
    int                                        nRemainDist;                                                                        /**< remain distance, meter */
    int                                        nID;                                                                                       /**< ID */
    BNAPosPoint                      stLanePos;                                                                            /**< positon */
    bool                                     bLaneShow;                                                                          /**< lane info should be showed */
}BNAGuidanceLaneInfo;

//simple map info
typedef struct _BNAGuidanceSimpleMapInfo
{
    BNAGuideMapUpdateType  updateType;
    BNAGuideManeuverType    turnType;
    BNAGuideManeuverType    nextTurnType;
    int 			                                startDistance;
    int 			                                remainDistance;               //(int)distance
    int                                            realRemainDistance;
    int 			                                remainTime;
    char			                                iconFileName[BNA_GUIDANCE_FILENAME_MAX_LENGTH ];
    char                                        curRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH ];
    char			                                nextRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH ];
    bool                                        isStraight;
    int                                          distCurToNextTurn;
    bool                                       isHighwayExCurToNextTurn;
    bool                                       isStraightIcon;
}BNAGuidanceSimpleMapInfo;

//remaininfo
typedef struct _BNAGuideRemainInfo
{
    int remainDistance;   //meter
    int remainTime;         //second
}BNAGuideRemainInfo;


//assistant type
typedef enum _BNAGuideAssistantType
{
    BNA_GUIDE_ASSISTANT_TYPE_JOINT,
    BNA_GUIDE_ASSISTANT_TYPE_TUNEL,
    BNA_GUIDE_ASSISTANT_TYPE_BRIDGE,
    BNA_GUIDE_ASSISTANT_TYPE_RAILWAY,
    BNA_GUIDE_ASSISTANT_TYPE_BLIND_BEND,
    BNA_GUIDE_ASSISTANT_TYPE_BLIND_SLOPE,
    BNA_GUIDE_ASSISTANT_TYPE_ROCK_FAll,
    BNA_GUIDE_ASSISTANT_TYPE_ACCIDENT,
    BNA_GUIDE_ASSISTANT_TYPE_SPEED_CAMERA,
    BNA_GUIDE_ASSISTANT_TYPE_TRAFFIC_LIGHT_CAMERA,
    BNA_GUIDE_ASSISTANT_TYPE_DRIVE_AGAINST_CAMERA,
    BNA_GUIDE_ASSISTANT_TYPE_INTERVAL_SPEED_CAMERA,
    BNA_GUIDE_ASSISTANT_TYPE_CHILDREN_ATTENTION,
    BNA_GUIDE_ASSISTANT_TYPE_UNEVEN,
    BNA_GUIDE_ASSISTANT_TYPE_NARROW,
    BNA_GUIDE_ASSISTANT_TYPE_VILIAGE,
    BNA_GUIDE_ASSISTANT_TYPE_SLIP,
    BNA_GUIDE_ASSISTANT_TYPE_OVERTAKE_FORBIDDEN,
    BNA_GUIDE_ASSISTANT_TYPE_HONK,
    BNA_GUIDE_ASSISTANT_TYPE_HILL_SIDE_DANGEROUS,
    BNA_GUIDE_ASSISTANT_TYPE_NARROW_BRIDGE,
    BNA_GUIDE_ASSISTANT_TYPE_CROSS_WIND,
    BNA_GUIDE_ASSISTANT_TYPE_UNDERWATER,
    BNA_GUIDE_ASSISTANT_TYPE_LOW_SPEED,
    BNA_GUIDE_ASSISTANT_TYPE_ZIP_PASS
}BNAGuideAssistantType;

//join type
typedef enum BNAGuideJoinType
{
    BNA_GUIDE_JOIN_TYPE_INVALID = 0x00000000,
    BNA_GUIDE_JOIN_TYPE_LEFT    = 0x00000001,
    BNA_GUIDE_JOIN_TYPE_RIGHT   = 0x00000002,
    BNA_GUIDE_JOIN_TYPE_MAIN    = 0x00000003
}BNAGuideJoinType;

//blind bend type
typedef enum _BNAGuideBlindBendType
{
    BNA_GUIDE_BlINDBEND_TYPE_INVALID       = 0x00000000,
    BNA_GUIDE_BLINDBEND_TYPE_LEFT         = 0x00000001,
    BNA_GUIDE_BlINDBEND_TYPE_RIGHT         = 0x00000002,
    BNA_GUIDE_BlINDBEND_TYPE_REVERSE       = 0x00000003,
    BNA_GUIDE_BlINDBEND_TYPE_CONTINUOUS    = 0x00000004,
}BNAGuideBlindBendType;

//narrow type
typedef enum _BNAGuideNarrowType
{
    BNA_GUIDE_NARROW_TYPE_INVALID ,
    BNA_GUIDE_NARROW_TYPE_LEFT ,
    BNA_GUIDE_NARROW_TYPE_RIGHT ,
    BNA_GUIDE_NARROW_TYPE_BOTH
}BNAGuideNarrowType;

//camera type
typedef enum _BNAGuideCameraType
{
    BNA_GUIDE_CAMERA_TYPE_INVALID ,
    BNA_GUIDE_CAMERA_TYPE_SPEEDLIMMITED ,
    BNA_GUIDE_CAMERA_TYPE_TRAFFICLIGHT ,
    BNA_GUIDE_CAMERA_TYPE_PECCANCY ,
    BNA_GUIDE_CAMERA_TYPE_INTERNAL
}BNAGuideCameraType;

//rock fall
typedef enum _BNAGuideRockFallType
{
    BNA_GUIDE_ROCKFALL_TYPE_INVALID ,
    BNA_GUIDE_ROCKFALL_TYPE_LEFT ,
    BNA_GUIDE_ROCKFALL_TYPE_RIGHT
}BNAGuideRockFallType;

//slop type
typedef enum _BNAGuideSlopeType
{
    BNA_GUIDE_SLOP_TYPE_INVALID ,
    BNA_GUIDE_SLOP_TYPE_UP ,
    BNA_GUIDE_SLOP_TYPE_DOWN ,
    BNA_GUIDE_SLOP_TYPE_CONTINUOUS_DOWN
}BNAGuideSlopeType;

//railway type
typedef enum _BNAGuideRailwayType
{
    BNA_GUIDE_RAILWAY_TYPE_INVALID,
    BNA_GUIDE_RAILWAY_TYPE_MANAGED,
    BNA_GUIDE_RAILWAY_TYPE_UNMANAGED
}BNAGuideRailwayType;

//assistant guidance info
typedef struct _BNAGuideAssistantInfo
{
    BNAGuideMapUpdateType  updateType;
    BNAGuideAssistantType   assistType;
    BNAGuideJoinType             joinType;
    BNAGuideBlindBendType blindBendType;
    BNAGuideNarrowType      narrowType;
    BNAGuideCameraType     cameraType;
    BNAGuideRockFallType     rockFallType;
    BNAGuideSlopeType         slopType;
    BNAGuideRailwayType      railwayType;
    int                                        cameraSpeedLimit;
    int                                        startDistance;
    int                                        remainDistance;
}BNAGuideAssistantInfo;

//grid map kind
typedef enum _BNAGuideGridMapKind
{
    BNA_GUIDE_GRID_MAP_INVALID,
    BNA_GUIDE_GRID_MAP_SUCCESS,
    BNA_GUIDE_GRID_MAP_FAIL,
    BNA_GUIDE_GRID_MAP_NOENOUGHMEMORY,
    BNA_GUIDE_GRID_MAP_INVILIDPARAM,
    BNA_GUIDE_GRID_MAP_FAILFINISH,
    BNA_GUIDE_GRID_MAP_FINISH,
    BNA_GUIDE_GRID_MAP_LACKDATA,
    BNA_GUIDE_GRID_MAP_BASEDATANOEXIST,
    BNA_GUIDE_GRID_MAP_NODATA,
    BNA_GUIDE_GRID_MAP_LACKDATAFALL,
}BNAGuideGridMapKind;

//raster expand map info
typedef struct _BNAGuideRasterExpandMapInfo
{
    BNAGuideMapUpdateType updateType;
    BNAGuideGridMapKind nGridMapKind;
    char usBackGroundMap[BNA_GUIDANCE_FILENAME_MAX_LENGTH];
    int sizeOfBackGroundMap = 0;
    int bgFileId;
    char usArrowMap[BNA_GUIDANCE_FILENAME_MAX_LENGTH];
    int sizeOfArrowMap = 0;
    char usProgressMap[BNA_GUIDANCE_FILENAME_MAX_LENGTH];
    int sizeOfProgressMap = 0;
    char usNextRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];
    int nAddDist;
    int nStartDist;
    int nRemainDist;
    int nRemainTime;
    bool bDownloadDone;
    char cTag[BNA_GUIDANCE_VIEWTAG_LENGTH];
    int imageWidth;
    int imageHeight;
}BNAGuideRasterExpandMapInfo;

//vehicle
typedef struct _BNAGuideVehicleInfo
{
    float                fOriAngle;
    float                fOriSpeed;
    BNAPosPoint stOriPos;//gps origin pos
    float                fAngle;
    float                fSpeed;
    BNAPosPoint stPos;
    char                usRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH ];
    bool                bIsBindRoad;
    int                    unAddDist;
}BNAGuideVehicleInfo;

//road level
typedef enum _BNAGuideRoadLevel
{
    BNA_GUIDE_ROAD_LEVEL_HIGHWAY 		    =	0x00000000 ,
    BNA_GUIDE_ROAD_LEVEL_CITYFASTWAY 	=	0x00000001 ,
    BNA_GUIDE_ROAD_LEVEL_NATIONWAY	 	=	0x00000002 ,
    BNA_GUIDE_ROAD_LEVEL_PROVINCEWAY 	=	0x00000003 ,
    BNA_GUIDE_ROAD_LEVEL_COUNTYWAY	 	=	0x00000004 ,
    BNA_GUIDE_ROAD_LEVEL_TOWNWAY 		=	0x00000005 ,
    BNA_GUIDE_ROAD_LEVEL_OTHERWAY 		=	0x00000006 ,
    BNA_GUIDE_ROAD_LEVEL_LEVEL9WAY	 	=	0x00000007 ,
    BNA_GUIDE_ROAD_LEVEL_FERRYWAY 		    =	0x00000008 ,
    BNA_GUIDE_ROAD_LEVEL_WALKWAY 		    =	0x00000009 ,
    BNA_GUIDE_ROAD_LEVEL_INVALID 		    =	0xFFFFFFFF
}BNAGuideRoadLevel;

//animation type
typedef  enum  _BNAGuideMapAnimationType
{
    BNA_GUIDE_MAP_ANIMATION_NONE              = 0,
    BNA_GUIDE_MAP_ANIMATION_POS                  = 0x00000001,
    BNA_GUIDE_MAP_ANIMATION_ROTATE            = 0x00000010,
    BNA_GUIDE_MAP_ANIMATION_OVERLOOK      = 0x00000100,
    BNA_GUIDE_MAP_ANIMATION_LEVEL               = 0x00001000,
    BNA_GUIDE_MAP_ANIMATION_ALL                   = 0x00001111,
    BNA_GUIDE_MAP_ANIMATION_INELLIGENT     = 0x10000000,
    BNA_GUIDE_MAP_ANIMATION_FROGLEAP       = 0x10000001,
    BNA_GUIDE_MAP_ANIMATION_FLYOVER           = 0x10000010,
}BNAGuideMapAnimationType;


//gps data struct
typedef struct _BNAGuideGPSData
{
    double longitude;
    double latitude;
    float     speed;
    float     bearing;
    float    accuracy;
    float    altitude;
    int       starCnt;
    int        starCntFlag;
    bool    isWifiLoc;
}BNAGuideGPSData;

//highway info
typedef struct _BNAHighWayInfo
{
    BNAGuideMapUpdateType  updateType;                                                                              //update type
    char     usExitHighwayDirectName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];            //road name when exit highway
    char     usExitHighwayNextRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];     //next road name when exit highway
    char     usExitHighwayID[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];                            //id of exiting highway
    char     usHighwayTollGateName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];              //name of nearest toll gate
    char     usHighwaySAName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];                       //name of nearest service area
    char     usHighwayNextSAName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];               //name of next nearest service area
    char     usCurHighwayRoadName[BNA_GUIDANCE_ROADNAME_MAX_LENGTH];            //name of current highway road
    unsigned int       unExitTotalDist;                                                                                            //total distance of the exit highway  //not for upper layer
    unsigned int       unNextGPTotalDist;                                                                                     //total distance of next Guide Point  //not for upper layer
    unsigned int       unTollGateTotalDist;                                                                                    //total distance of toll gate  //not for upper layer
    unsigned int       unSATotalDist;                                                                                              //total distance of service area  //not for upper layer
    unsigned int       unNextSATotalDist;                                                                                      //total distance of next service area  //not for upper layer
    unsigned int       unExitRemainDist;                                                                                       //remain distance of exit
    unsigned int       unNextGPRemainDist;                                                                                //remain distance of next guide point
    unsigned int       unTollGateRemainDist;                                                                               //remain distance of toll gate
    unsigned int       unSARemainDist;                                                                                         //remain distance of service area
    unsigned int       unNextSARemainDist;                                                                                 //remain distance of next service area
    bool       bHideInfo;                                                                                                                  //status of hide highway info
}BNAHighWayInfo;

//type of updating road condition
typedef enum _BNAUpdateRoadConditionEnum
{
    BNA_GUIDE_UPDATEROADCONTION_INVAILD,
    BNA_GUIDE_UPDATEROADCONTION_ALL,
    BNA_GUIDE_UPDATEROADCONTION_ONLYRC,
}BNAUpdateRoadConditionEnum;


//type of road condition
typedef enum _BNARoadConditionType
{
    BNA_GUIDE_ROADCONTION_INVAILD                   = 0x00000000,
    BNA_GUIDE_ROADCONTION_STRAIGHTWAY         = 0x00000001,
    BNA_GUIDE_ROADCONTION_SLOW                        = 0x00000002,
    BNA_GUIDE_ROADCONTION_OBSTRUCTION         = 0x00000003,
    BNA_GUIDE_ROADCONTION_VERYOBSTRUCTION = 0x00000004,
}BNARoadConditionType;

//roadcondition item
typedef struct _BNARoadConditionItem
{
    unsigned int						unEndShapeIdx;
    BNARoadConditionType	enRoadCondition;
    unsigned int                       unEndAddDist;
    unsigned int                       unEndTravelTime;        //0.01s
}BNARoadConditionItem;


NAMESPACE_END
NAMESPACE_END

#endif // NAVI_AUTO_GUIDANCE_DEF_H
