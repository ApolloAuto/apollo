//
//  navi_car_adapter_def.hpp
//  baiduNaviSDK
//
//  Created by huangpanhua on 16/10/20.
//  Copyright © 2016年 baidu. All rights reserved.
//
#ifndef NAVI_CAR_ADAPTER_DEF_H
#define NAVI_CAR_ADAPTER_DEF_H

#include <stddef.h>
#include <stdint.h>
#include <malloc.h>
#ifdef __V_ANDROID_PLATFORM__    // pany for build android so
#undef max
#undef min
#endif
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#define NAMESPACE_BAIDU             namespace baidu{
#define NAMESPACE_BAIDU_ADAPTER     namespace navi_adapter{
#define USING_NAMESPACE_ADAPTER     using namespace baidu; using namespace navi_adapter;
#define NAMESPACE_END               }

#define PRODUCT_NAME                "NAVI_ADAPTER"

typedef enum _BNAScreenType
{
    BNAScreenUndefined = 0,
    BNAScreenCenter,
    BNAScreenCluster,
    BNAScreenHud,
    BNAScreenLeftRear,
    BNAScreenRightRear,
    BNAScreenAssist,
    BNAScreenExtension
}BNAScreenType;

//copy from navi_engine_search_def.h

#define NE_COMMON_CHAR_MAX					32
#define NE_SEARCH_SUG_MAX_RESULT						16
#define NE_SEARCH_POI_NAME_MAX_LENGTH					128
#define NE_SEARCH_POI_PHONE_MAX_LENGTH					128  // 16不够用，比如有分机号码的固话
#define NE_SEARCH_POI_MAX_PHONE_CNT						3	// 3个电话号码已经足够了
#define NE_SEARCH_POI_DESC_MAX_LENGTH					512
#define NE_SEARCH_STREETID_MAX_LENGTH					128
#define NE_SEARCH_POI_UID_MAX_LENGTH					32
#define BNA_LOCATION_MILE_TO_KILOMETER                           1.8                //英里与公里转换比;add by sfy;
#define BNA_LOCATION_DISTRICT_NAME_MAX_LENGTH           32                 //行政区划英文名称最大长度；add by sfy;
#define BNA_LOCATION_POINT_RATIO                                         100000.0     //坐标转换时乘的系数
#define BNA_MAP_CONTROL_SCREEN_SCALE                               1                  //缩放比
#define BNA_MAP_CONTROL_KDEFAULT_RADIUS                         30

#define BNA_MAX_POINTS_FOR_REGION                            512     //max points of self-define region, draw on map
#define NE_SEARCH_POI_CHILDREN_MAX_LENGTH                 8

///////////////////////////////////Navigation///////////////////////////////

typedef enum _BNANaviDayNightMode
{
    BNANaviDayNightModeAuto,
    BNANaviDayNightModeDay,
    BNANaviDayNightModeNight
}BNANaviDayNightMode;

typedef enum _BNANaviSpeakMode
{
    BNANaviSpeakModePro,
    BNANaviSpeakModeMute,
    BNANaviSpeakModeNewbie
}BNANaviSpeakMode;

typedef enum _BNANaviSpeakContent
{
    BNASpeakContentEEye               = 1 << 0,
    BNASpeakContentRateLimiting       = 1 << 1,
    BNASpeakContentSafeDriving        = 1 << 2,
    BNASpeakContentAheadRoadCondition = 1 << 3,
    BNASpeakContentStraightRemind     = 1 << 4
}BNANaviSpeakContent;

typedef struct _BNANavigationConfig
{
    BNANaviDayNightMode dayNightMode;
    BNANaviSpeakMode    speakMode;
    BNANaviSpeakContent speakContent;
    bool                isAutoCruise;
    bool                isRoutePlanOnlinePriority;
    bool                isSearchOnlinePriority;

}BNANavigationConfig;

typedef struct _BNACruiseConfig
{
    bool bCloseSpeedLimit;
    bool bCloseSpeedCamera;
    bool bCloseTrafficLightCameraSpeak;
    bool bClosePeccanryCameraSpeak;
    bool bCloseTrafficSign;
}BNACruiseConfig;

typedef enum _BNANavigationViewStatus
{
    BNA_NAVI_VIEW_STATUS_VIEW_ALL,
    BNA_NAVI_VIEW_STATUS_BROWSE,
    BNA_NAVI_VIEW_STATUS_FLLOW_CAR_POSITION
}BNANavigationViewStatus;

typedef enum _BNANavigationStatusType
{
    BNA_NAVI_STATUS_TYPE_START,
    BNA_NAVI_STATUS_TYPE_YAWING,
    BNA_NAVI_STATUS_TYPE_REROUTE_END, /** yawing and recalculate route end*/
    BNA_NAVI_STATUS_TYPE_REROUTE_CAR_FREE,        /** */
    BNA_NAVI_STATUS_TYPE_APPROACH_DESTINATION,
    BNA_NAVI_STATUS_TYPE_ARRIVED_DESTINATION
}BNANavigationStatusType;

/** guidance status */
typedef enum _BNAGuidanceStatus
{
    BNA_GUIDANCE_STATUS_INVALID,
    BNA_GUIDANCE_STATUS_TRACK,
    BNA_GUIDANCE_STATUS_REALGUIDE,
    BNA_GUIDANCE_STATUS_DEMOGUIDE,
    BNA_GUIDANCE_STATUS_CRUISE,
    BNA_GUIDANCE_STATUS_LIGHTNAVI,
    BNA_GUIDANCE_STATUS_GENERICGUIDE
}BNAGuidanceStatus;

///////////////////////////////////RoutePlan///////////////////////////////

typedef struct _BNACoordinate
{
    double						iLongitude;		/**< 经度*100000 */
    double						iLatitude;		/**< 纬度*100000 */
}BNACoordinate;

//road guide struct end
/**
 * 坐标系类型
 */
typedef enum _BNACoordinateType
{
    BNA_COORDINATE_ORIGINAL_GPS = 0,
    BNA_COORDINATE_BAIDUMAP_SDK = 1,
}BNACoordinateType;

/** 路径点(起点、途径点、目的地) */
typedef struct _BNARouteNode
{

    BNACoordinate                        position;        /** 显示坐标*/
    BNACoordinateType              coordinateType;  /** 坐标类型*/

}BNARouteNode;

typedef enum BNARouteType {
    INVALID,    //<** INVALID Route Type */
    FASTEST,                      //<** FASTEST Route Type */
    SHORTEST,                     //<** SHORTEST Route Type */
    ECO,                          //<** ECO Route Type */
    FREE,                         //<** FREE Route Type */
    HIGHWAY,                      //<** HIGHWAY Route Type */
    STATIC,                       //<** STATIC Route Type */
    CHN_MOST_POPULAR,             //<** 常规路线: Most popular */
    CHN_LESS_TIME,                //<** 时间较短: Less time */
    CHN_SHORTEST_DISTANCE,        //<** 距离较短：Shortest distance */
    CHN_LESS_FEE,                 //<** 收费较少: Less fee */
    CHN_FREE,                     //<** 不收费: Free */
    CHN_LESS_TRAFFIC_JAMS,        //<** 拥堵较少: Less traffic jams */
    CHN_LESS_TRAFFIC_LIGHTS,      //<** 红绿灯少: Less traffic lights */
    CHN_LESS_TURNS,               //<** 转弯较少： Less turns */
    CHN_MOST_OF_BIG_ROADS,        //<** 大路较多: Most of the distance is big road, like highway or fast road */
    CHN_SAVED_BY_USERS,           //<** 轨迹路线: The route saved by users */
    CHN_USER_ONCE_DRIVE_ALONG,    //<** 曾经走过: The route user once drive along */
    CHN_FIRST_SOLUTION,           //<** 推荐路线: If no labels suitable for the first solution according to the other 2 solutions */
    CHN_SECOND_SOLUTION,          //<** 方案二：If no labels suitable for the second solution according to the other 2 solutions */
    CHN_THIRD_SOLUTION,           //<** 方案三：If no labels suitable for the third solution according to the other 2 solution */
    MAX
}BNARouteType;

typedef enum _BNAManeuverType
{
    BNA_MANEUVER_TYPE_INVALID ,			            /**<  无效值 */
    BNA_MANEUVER_TYPE_FRONT ,			            /**<  直行 */
    BNA_MANEUVER_TYPE_RIGHT_FRONT ,		            /**<  右前方转弯 */
    BNA_MANEUVER_TYPE_RIGHT ,			            /**<  右转 */
    BNA_MANEUVER_TYPE_RIGHT_BACK ,		            /**<  右后方转弯 */
    BNA_MANEUVER_TYPE_BACK ,				            /**<  掉头 */
    BNA_MANEUVER_TYPE_LEFT_BACK ,		            /**<  左后方转弯 */
    BNA_MANEUVER_TYPE_LEFT ,				            /**<  左转 */
    BNA_MANEUVER_TYPE_LEFT_FRONT ,		            /**<  左前方转弯 */
    BNA_MANEUVER_TYPE_RING ,				            /**<  环岛 */
    BNA_MANEUVER_TYPE_RING_OUT ,			            /**<  环岛出口 */
    BNA_MANEUVER_TYPE_LEFT_SIDE ,		            /**<  普通/JCT/SAPA二分歧 靠左 */
    BNA_MANEUVER_TYPE_RIGHT_SIDE ,		            /**<  普通/JCT/SAPA二分歧 靠右 */
    BNA_MANEUVER_TYPE_LEFT_SIDE_MAIN ,	            /**<  左侧走本线 */
    BNA_MANEUVER_TYPE_BRANCH_LEFT_MAIN ,             /**<  靠最左走本线 */
    BNA_MANEUVER_TYPE_RIGHT_SIDE_MAIN ,	            /**<  右侧走本线 */
    BNA_MANEUVER_TYPE_BRANCH_RIGHT_MAIN,             /**<  靠最右走本线 */
    BNA_MANEUVER_TYPE_CENTER_MAIN ,                  /**<  中间走本线 */
    BNA_MANEUVER_TYPE_LEFT_SIDE_IC ,		            /**<  IC二分歧左侧走IC */
    BNA_MANEUVER_TYPE_RIGHT_SIDE_IC ,	            /**<  IC二分歧右侧走IC */
    BNA_MANEUVER_TYPE_BRANCH_LEFT ,		            /**<  普通三分歧/JCT/SAPA 靠最左 */
    BNA_MANEUVER_TYPE_BRANCH_RIGHT ,		            /**<  普通三分歧/JCT/SAPA 靠最右 */
    BNA_MANEUVER_TYPE_BRANCH_CENTER ,	            /**<  普通三分歧/JCT/SAPA 靠中间 */
    BNA_MANEUVER_TYPE_START ,			            /**<  起始地 */
    BNA_MANEUVER_TYPE_DEST ,				            /**<  目的地 */
    BNA_MANEUVER_TYPE_VIA1 ,				            /**<  途径点1 */
    BNA_MANEUVER_TYPE_VIA2 ,				            /**<  途径点2 */
    BNA_MANEUVER_TYPE_VIA3 ,				            /**<  途径点3 */
    BNA_MANEUVER_TYPE_VIA4 ,				            /**<  途径点4 */
    BNA_MANEUVER_TYPE_IN_FERRY ,			            /**<  进入渡口 */
    BNA_MANEUVER_TYPE_OUT_FERRY ,			            /**<  脱出渡口 */
    BNA_MANEUVER_TYPE_TOLL_GATE ,                     /**<  收费站 */
    BNA_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT_IC ,        /**<  IC二分歧左侧直行走IC */
    BNA_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT_IC ,       /**<  IC二分歧右侧直行走IC */
    BNA_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT ,           /**<  普通/JCT/SAPA二分歧左侧 直行 */
    BNA_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT ,          /**<  普通/JCT/SAPA二分歧右侧 直行 */
    BNA_MANEUVER_TYPE_BRANCH_LEFT_STRAIGHT ,         /**<  普通/JCT/SAPA三分歧左侧 直行 */
    BNA_MANEUVER_TYPE_BRANCH_CENTER_STRAIGHT ,       /**<  普通/JCT/SAPA三分歧中央 直行 */
    BNA_MANEUVER_TYPE_BRANCH_RIGHT_STRAIGHT ,        /**<  普通/JCT/SAPA三分歧右侧 直行 */
    BNA_MANEUVER_TYPE_BRANCH_LEFT_IC ,               /**<  IC三分歧左侧走IC */
    BNA_MANEUVER_TYPE_BRANCH_CENTER_IC ,             /**<  IC三分歧中央走IC */
    BNA_MANEUVER_TYPE_BRANCH_RIGHT_IC ,              /**<  IC三分歧右侧走IC */
    BNA_MANEUVER_TYPE_BRANCH_LEFT_IC_STRAIGHT ,      /**<  IC三分歧左侧直行 */
    BNA_MANEUVER_TYPE_BRANCH_CENTER_IC_STRAIGHT ,	/**<  IC三分歧中间直行 */
    BNA_MANEUVER_TYPE_BRANCH_RIGHT_IC_STRAIGHT ,     /**<  IC三分歧右侧直行 */
    BNA_MANEUVER_TYPE_STRAIGHT_2BRANCH_LEFT_BASE ,   /**<  八方向靠左直行*/
    BNA_MANEUVER_TYPE_STRAIGHT_2BRANCH_RIGHT_BASE ,  /**<  八方向靠右直行*/
    BNA_MANEUVER_TYPE_STRAIGHT_3BRANCH_LEFT_BASE  ,  /**<  八方向靠最左侧直行*/
    BNA_MANEUVER_TYPE_STRAIGHT_3BRANCH_MIDDLE_BASE , /**<  八方向沿中间直行 */
    BNA_MANEUVER_TYPE_STRAIGHT_3BRANCH_RIGHT_BASE ,  /**<  八方向靠最右侧直行 */
    BNA_MANEUVER_TYPE_LEFT_2BRANCH_LEFT_BASE ,       /**<  八方向左转+随后靠左 */
    BNA_MANEUVER_TYPE_LEFT_2BRANCH_RIGHT_BASE ,      /**<  八方向左转+随后靠右 */
    BNA_MANEUVER_TYPE_LEFT_3BRANCH_LEFT_BASE ,       /**<  八方向左转+随后靠最左 */
    BNA_MANEUVER_TYPE_LEFT_3BRANCH_MIDDLE_BASE ,     /**<  八方向左转+随后沿中间 */
    BNA_MANEUVER_TYPE_LEFT_3BRANCH_RIGHT_BASE ,      /**<  八方向左转+随后靠最右 */
    BNA_MANEUVER_TYPE_RIGHT_2BRANCH_LEFT_BASE ,      /**<  八方向右转+随后靠左 */
    BNA_MANEUVER_TYPE_RIGHT_2BRANCH_RIGHT_BASE ,     /**<  八方向右转+随后靠右 */
    BNA_MANEUVER_TYPE_RIGHT_3BRANCH_LEFT_BASE ,      /**<  八方向右转+随后靠最左 */
    BNA_MANEUVER_TYPE_RIGHT_3BRANCH_MIDDLE_BASE ,    /**<  八方向右转+随后沿中间 */
    BNA_MANEUVER_TYPE_RIGHT_3BRANCH_RIGHT_BASE,      /**<  八方向右转+随后靠最右 */
    BNA_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_LEFT_BASE,  /**<  八方向左前方靠左侧 */
    BNA_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_RIGHT_BASE,  /**<  八方向左前方靠右侧 */
    BNA_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_LEFT_BASE,  /**<  八方向右前方靠左侧 */
    BNA_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_RIGHT_BASE,    /**<  八方向右前方靠右侧 */
    BNA_MANEUVER_TYPE_BACK_2BRANCH_LEFT_BASE ,      /**<  八方向掉头+随后靠左 */
    BNA_MANEUVER_TYPE_BACK_2BRANCH_RIGHT_BASE ,     /**<  八方向掉头+随后靠右 */
    BNA_MANEUVER_TYPE_BACK_3BRANCH_LEFT_BASE ,      /**<  八方向掉头+随后靠最左 */
    BNA_MANEUVER_TYPE_BACK_3BRANCH_MIDDLE_BASE ,    /**<  八方向掉头+随后沿中间 */
    BNA_MANEUVER_TYPE_BACK_3BRANCH_RIGHT_BASE,      /**<  八方向掉头+随后靠最右 */
}BNAManeuverType;

typedef struct _BNARouteSegment
{
    BNAManeuverType maneuverType;
    char    nextRoadName[32];
    char    discription[128];
    int     length;
    int     time;
    BNACoordinate   coordinate;
    int     shapePointIndex;
    int     outLinkAngle;
    int     trafficStatus;
}BNARouteSegment;

typedef struct _BNARouteDetailInfo
{
    BNARouteType     roadType;
    int     length;
    int     time;
    int     fare;
    bool  isTolled;
    bool  isUsingLocalCarInfo;
    int     routeSegmentsCount;
    int     trafficSignalCount;
    double  topLeftLongitude;
    double  topLeftLatitude;
    double  bottomRightLongitude;
    double  bottomRightLatitude;
    BNARouteSegment *routeSegments;
}BNARouteDetailInfo;

typedef enum _BNARoutePlanResultType
{
    BNA_ROUTEPLAN_INVALID,
    BNA_ROUTEPLAN_SUCCEED,
    BNA_ROUTEPLAN_NODE_NOT_READY,
    BNA_ROUTEPLAN_DATA_NOT_READY,
    BNA_ROUTEPLAN_NET_NOT_REACHABLE,
    BNA_ROUTEPLAN_TOO_NEAR,
    BNA_ROUTEPLAN_FAILED,
}BNARoutePlanResultType;

typedef enum _BNARoutePlanNetMode
{
    BNA_ROUTEPLAN_NET_MODE_INVALID,
    BNA_ROUTEPLAN_NET_MODE_ONLINE,
    BNA_ROUTEPLAN_NET_MODE_OFFLINE,
    BNA_
}BNARoutePlanNetMode;

typedef struct _BNARoutePlanResultInfo
{

    BNARoutePlanResultType      resultType;
    int 						routeCount;		/**<  路线数量 */
    BNARoutePlanNetMode         netMode;        /**<  路线规划的网络模式，在线 or 离线 */
    int                         calculateTime;		/**<  算路时间(包含网络不好反复重试时间)，单位毫秒 */

}BNARoutePlanResult;

typedef struct _BNARouteRemainInfo
{
    int remainDistance;     /**< 单位米 */
    int remainTime;         /**< 单位秒 */
}BNARouteRemainInfo;

typedef struct _BNASpeedLimitInfo
{
    int speedLimit;
}BNASpeedLimitInfo;

typedef struct _BNACurrentRoadeNameInfo
{
    short unsigned int roadName[NE_SEARCH_POI_DESC_MAX_LENGTH];
}BNACurrentRoadeNameInfo;

typedef enum _BNAGuidanceInfoResultType
{
    BNA_GUIDANCE_INFO_INVALID,
    BNA_GUIDANCE_INFO_UPDATE

}BNAGuidanceInfoResultType;

/** 简易图信息 */
typedef struct _BNASimpleGuidanceInfo
{
    BNAGuidanceInfoResultType resultType;
    int 			startDistance;									/**<  开始距离，单位米 */
    int 			remainDistance;								/**<  剩余距离（已经四舍五入，单位米 */
    int             realRemainDistance;                            /**<  剩余距离（实际的剩余距离），单位米 */
    int 			remainTime;								/**<  剩余时间 */
    char			iconFileName[ NE_SEARCH_STREETID_MAX_LENGTH ];	/**<  图标文件名 */
    char        currentRoadName[ NE_SEARCH_STREETID_MAX_LENGTH ];    /**<  当前道路名 */
    char			nextRoadName[ NE_SEARCH_STREETID_MAX_LENGTH ];	/**<  下一路名 */
    bool           isStraight;                                  /**<  是否处于顺行模式 */
    BNAManeuverType   turnType;                     /**<  下一转向 */
    BNAManeuverType   nextTurnType;                     /**<  下一转向 */
    int            distanceFromCurrentTurnToNextTurn;                            /**<  下一个转弯到当前转弯的距离 */
    bool           isHighwayExCurrentToNextTurn;                       /**<  下一转向进入Link是否是高速快速 */

}BNASimpleGuidanceInfo;


typedef enum _BNAAssistantType
{
    BNA_ASSISTANT_TYPE_JOINT,
    BNA_ASSISTANT_TYPE_TUNEL,
    BNA_ASSISTANT_TYPE_BRIDGE,
    BNA_ASSISTANT_TYPE_RAILWAY,
    BNA_ASSISTANT_TYPE_BLIND_BEND,
    BNA_ASSISTANT_TYPE_BLIND_SLOPE,
    BNA_ASSISTANT_TYPE_ROCK_FAll,
    BNA_ASSISTANT_TYPE_ACCIDENT,
    BNA_ASSISTANT_TYPE_SPEED_CAMERA,
    BNA_ASSISTANT_TYPE_TRAFFIC_LIGHT_CAMERA,
    BNA_ASSISTANT_TYPE_DRIVE_AGAINST_CAMERA,
    BNA_ASSISTANT_TYPE_INTERVAL_SPEED_CAMERA,
    BNA_ASSISTANT_TYPE_CHILDREN_ATTENTION,
    BNA_ASSISTANT_TYPE_UNEVEN,
    BNA_ASSISTANT_TYPE_NARROW,
    BNA_ASSISTANT_TYPE_VILIAGE,
    BNA_ASSISTANT_TYPE_SLIP,
    BNA_ASSISTANT_TYPE_OVERTAKE_FORBIDDEN,
    BNA_ASSISTANT_TYPE_HONK,
    BNA_ASSISTANT_TYPE_HILL_SIDE_DANGEROUS,
    BNA_ASSISTANT_TYPE_NARROW_BRIDGE,
    BNA_ASSISTANT_TYPE_CROSS_WIND,
    BNA_ASSISTANT_TYPE_UNDERWATER,
    BNA_ASSISTANT_TYPE_LOW_SPEED,
    BNA_ASSISTANT_TYPE_ZIP_PASS
}BNAAssistantType;

typedef enum BNAJoinType
{
    BNA_JOIN_TYPE_INVALID = 0x00000000,
    BNA_JOIN_TYPE_LEFT    = 0x00000001,
    BNA_JOIN_TYPE_RIGHT   = 0x00000002,
    BNA_JOIN_TyPE_MAIN    = 0x00000003
}BNAJoinType;

/** 急转弯类型 */
typedef enum _BNABlindBendType
{
    BNA_BlINDBEND_TYPE_INVALID       = 0x00000000,   /**<  无效值 */
    BNA_BLIND_BEND_TYPE_LEFT         = 0x00000001,   /**<  左急转弯 */
    BNA_BlINDBEND_TYPE_RIGHT         = 0x00000002,   /**<  右急转弯 */
    BNA_BlINDBEND_TYPE_REVERSE       = 0x00000003,   /**<  反向急弯 */
    BNA_BlINDBEND_TYPE_CONTINUOUS    = 0x00000004,   /**<  连续转弯 */
}BNABlindBendType;

/** 道路变窄类型 */
typedef enum _BNANarrowType
{
    BNA_NARROW_TYPE_INVALID ,                    /**<  无效值 */
    BNA_NARROW_TYPE_LEFT ,                       /**<  左侧道路变窄 */
    BNA_NARROW_TYPE_RIGHT ,                      /**<  右侧道路变窄 */
    BNA_NARROW_TYPE_BOTH                         /**<  两侧道路变窄 */
}BNANarrowType;

/** 陡坡类型 */
typedef enum _BNASlopeType
{
    BNA_SLOP_TYPE_INVALID ,                     /**< 无效值 */
    BNA_SLOP_TYPE_UP ,                          /**< 上陡坡 */
    BNA_SLOP_TYPE_DOWN ,                        /**< 下陡坡 */
    BNA_SLOP_TYPE_CONTINUOUS_DOWN                /**< 连续陡坡 */
}BNASlopeType;

/** 落石类型 */
typedef enum _BNARockFallType
{
    BNA_ROCKFALL_TYPE_INVALID ,                  /**< 无效值 */
    BNA_ROCKFALL_TYPE_LEFT ,                     /**< 左侧落石 */
    BNA_ROCKFALL_TYPE_RIGHT                      /**< 右侧落石 */
}BNARockFallType;

/** 铁道口类型 */
typedef enum _BNARailwayType
{
    BNA_RAILWAY_TYPE_INVALID,           /**< 无效值 */
    BNA_RAILWAY_TYPE_MANAGED,           /**< 有人值守 */
    BNA_RAILWAY_TYPE_UNMANAGED          /**< 无人值守 */
}BNARailwayType;

typedef enum _BNACameraType
{
    BNA_CAMERA_TYPE_INVALID ,                    /**< 无效值 */
    BNA_CAMERA_TYPE_SPEEDLIMMITED ,               /**< 限速摄像头 */
    BNA_CAMERA_TYPE_DRIVE_AGAINST ,                   /**< 违章摄像头 */
    BNA_CAMERA_TYPE_TRAFFIC_LIGHT ,               /**< 交通信号灯摄像头 */
    BNA_CAMERA_TYPE_INTERNAL_SPPED_LIMMITED                     /**< 区间测速摄像头 */
}BNACameraType;


typedef struct _BNAAssistantGuidanceInfo
{
    BNAAssistantType type;
    int cameraSpeedLimit;
    int startDistance;
    int remainDistance;
    BNAJoinType joinType;
    BNABlindBendType blindBendType;
    BNANarrowType    narrowType;
    BNACameraType    cameraType;
    BNARockFallType  rockFallType;
    BNASlopeType     slopType;
    BNARailwayType   railwayType;
}BNAAssistantGuidanceInfo;

typedef enum _BNARoutePlanPreferenceType
{
    BNA_ROUTEPLAN_PREFERENCE_TYPE_INVALID = 0,
    BNA_ROUTEPLAN_PREFERENCE_TYPE_RECOMMEND = 1 << 0,
    BNA_ROUTEPLAN_PREFERENCE_TYPE_HIGHWAY = 1 << 1,
    BNA_ROUTEPLAN_PREFERENCE_TYPE_NO_HIGHWAY = 1 << 2,
    BNA_ROUTEPLAN_PREFERENCE_TYPE_NO_TOLL    = 1 << 3,
    BNA_ROUTEPLAN_PREFERENCE_TYPE_AVOID_TRAFFICJAM = 0x000000010

}BNARoutePlanPreferenceType;

typedef enum _BNACarTypeEnum
{
    BNA_CAR_TYPE_06L = 1,
    BNA_CAR_TYPE_08L = 2,
    BNA_CAR_TYPE_09L = 3,
    BNA_CAR_TYPE_DEFAULT= 0 ,
    BNA_CAR_TYPE_10L = 4,
    BNA_CAR_TYPE_10T = 5,
    BNA_CAR_TYPE_11L = 6,
    BNA_CAR_TYPE_12L = 7,
    BNA_CAR_TYPE_12T = 8,
    BNA_CAR_TYPE_13L = 9,
    BNA_CAR_TYPE_13T = 10,
    BNA_CAR_TYPE_14L = 11,
    BNA_CAR_TYPE_14T = 12,
    BNA_CAR_TYPE_15L = 13,
    BNA_CAR_TYPE_15T = 14,
    BNA_CAR_TYPE_16L = 15,
    BNA_CAR_TYPE_16T = 16,
    BNA_CAR_TYPE_18L = 17,
    BNA_CAR_TYPE_18T = 18,
    BNA_CAR_TYPE_19L = 19,
    BNA_CAR_TYPE_19T = 20,
    BNA_CAR_TYPE_20L = 21,
    BNA_CAR_TYPE_20T = 22,
    BNA_CAR_TYPE_21L = 23,
    BNA_CAR_TYPE_21T = 24,
    BNA_CAR_TYPE_22L = 25,
    BNA_CAR_TYPE_22T = 26,
    BNA_CAR_TYPE_23L = 27,
    BNA_CAR_TYPE_23T = 28,
    BNA_CAR_TYPE_24L = 29,
    BNA_CAR_TYPE_24T = 30,
    BNA_CAR_TYPE_25L = 31,
    BNA_CAR_TYPE_25T = 32,
    BNA_CAR_TYPE_26L = 33,
    BNA_CAR_TYPE_27L = 34,
    BNA_CAR_TYPE_27T = 35,
    BNA_CAR_TYPE_28L = 36,
    BNA_CAR_TYPE_28T = 37,
    BNA_CAR_TYPE_29L = 38,
    BNA_CAR_TYPE_30L = 39,
    BNA_CAR_TYPE_30T = 40,
    BNA_CAR_TYPE_32L = 41,
    BNA_CAR_TYPE_32T = 42,
    BNA_CAR_TYPE_33L = 43,
    BNA_CAR_TYPE_34L = 44,
    BNA_CAR_TYPE_35L = 45,
    BNA_CAR_TYPE_35T = 46,
    BNA_CAR_TYPE_36L = 47,
    BNA_CAR_TYPE_36T = 48,
    BNA_CAR_TYPE_37L = 49,
    BNA_CAR_TYPE_38L = 50,
    BNA_CAR_TYPE_38T = 51,
    BNA_CAR_TYPE_39L = 52,
    BNA_CAR_TYPE_40L = 53,
    BNA_CAR_TYPE_40T = 54,
    BNA_CAR_TYPE_42L = 55,
    BNA_CAR_TYPE_43L = 56,
    BNA_CAR_TYPE_44L = 57,
    BNA_CAR_TYPE_44T = 58,
    BNA_CAR_TYPE_45L = 59,
    BNA_CAR_TYPE_46L = 60,
    BNA_CAR_TYPE_47L = 61,
    BNA_CAR_TYPE_47T = 62,
    BNA_CAR_TYPE_48L = 63,
    BNA_CAR_TYPE_48T = 64,
    BNA_CAR_TYPE_50L = 65,
    BNA_CAR_TYPE_50T = 66,
    BNA_CAR_TYPE_52L = 67,
    BNA_CAR_TYPE_53L = 68,
    BNA_CAR_TYPE_54L = 69,
    BNA_CAR_TYPE_55L = 70,
    BNA_CAR_TYPE_56L = 71,
    BNA_CAR_TYPE_57L = 72,
    BNA_CAR_TYPE_58L = 73,
    BNA_CAR_TYPE_60L = 74,
    BNA_CAR_TYPE_60T = 75,
    BNA_CAR_TYPE_62L = 76,
    BNA_CAR_TYPE_62T = 77,
    BNA_CAR_TYPE_63L = 78,
    BNA_CAR_TYPE_64L = 79,
    BNA_CAR_TYPE_65L = 80,
    BNA_CAR_TYPE_65T = 81,
    BNA_CAR_TYPE_66L = 82,
    BNA_CAR_TYPE_67L = 83,
    BNA_CAR_TYPE_68L = 84,
    BNA_CAR_TYPE_71T = 85,
    BNA_CAR_TYPE_73L = 86,
    BNA_CAR_TYPE_78L = 87,
    BNA_CAR_TYPE_78T = 88,
    BNA_CAR_TYPE_80T = 89,
    BNA_CAR_TYPE_97L = 90,
    BNA_CAR_TYPE_98T = 91,
    BNA_CAR_TYPE_120L = 92,
    BNA_CAR_TYPE_127L = 93,
    BNA_CAR_TYPE_COUNT = 94
}BNACarTypeEnum;

typedef struct _BNACarInfo
{
    std::string carNum;
    std::string carProvinceName;
    BNACarTypeEnum carType;
}BNACarInfo;

///////////////////////////////////search///////////////////////////////

typedef struct _BNASearchSugResult
{
    int uSugCount;
    char  usName[NE_SEARCH_SUG_MAX_RESULT][NE_SEARCH_POI_NAME_MAX_LENGTH];
    char  usAddr[NE_SEARCH_SUG_MAX_RESULT][NE_SEARCH_POI_DESC_MAX_LENGTH];
}BNASearchSugResult;

//copy from navi_logic_search_def.hb
/** 搜索结果排序类型 */
typedef enum _BNASearchResultSortTypeEnum
{

    BNA_SEARCH_RESULT_SORT_TYPE_INVALID ,       /**<  无效值 */
    BNA_SEARCH_RESULT_SORT_TYPE_DIST ,              /**<  距离 */
    BNA_SEARCH_RESULT_SORT_TYPE_TEXTMATCH ,  /**<  文本匹配度 */
    BNA_SEARCH_RESULT_SORT_TYPE_HEAT                /**<  热度 */
}BNASearchResultSortTypeEnum;

//change from navi_logic_search_def.h
/** 坐标位置 */
typedef struct _BNASearchPoint
{
    int						iLongitude;		/**< 经度*100000 */
    int						iLatitude;		/**< 纬度*100000 */
}BNASearchPoint;

//copy from navi_logic_search_def.h
/** 圆区域 */
typedef struct _BNASearchCircleRegion
{
    BNASearchPoint 	stCenterPoint;	/**<  中心点坐标，WGS84(国测局) */
    unsigned int 				unRadius;		/**<  半径，单位米 */
}BNASearchCircleRegion;

//change from navi_engine_search_def.h
/** POI children信息 */
typedef struct _BNA_Search_POIChild_Info_t
{
    char						szUid[NE_SEARCH_POI_UID_MAX_LENGTH];	/* POI点的服务端uid */
    BNASearchPoint			childPoint;		/**< 显示坐标 */
    char		childName[NE_SEARCH_POI_NAME_MAX_LENGTH];				/**< POI名称 */
    char		childAliasName[NE_SEARCH_POI_NAME_MAX_LENGTH];		/**< POI别名 */
}BNA_Search_POIChild_Info_t;

/** POI信息 */
typedef struct _BNASearchPoiInfo
{
    unsigned int			unID;    /**< ID */
    unsigned int 			unType; /**< Type */
    BNASearchPoint			stGuidePoint;		/**< 诱导坐标 */
    BNASearchPoint			stViewPoint;		/**< 显示坐标 */
    int						nDistrictID;		/**< 行政区划ID */
    int						nWeight;			/**< POI权重 */
    char		stName[NE_SEARCH_POI_NAME_MAX_LENGTH];				/**< POI名称 */
    char		stAliasName[NE_SEARCH_POI_NAME_MAX_LENGTH];		/**< POI别名 */
    unsigned int						unDistance;			/**< POI距中心点或者起点的直线距离 */
    unsigned int						unPhoneCnt;			/**< 电话数 */
    char	stPhoneTable[ NE_SEARCH_POI_MAX_PHONE_CNT ][NE_SEARCH_POI_PHONE_MAX_LENGTH];     /**< 电话列表 */
    char						usDesc[NE_SEARCH_POI_DESC_MAX_LENGTH];   /**< POI描述，当做地址 */
    unsigned short 					usStreetid[NE_SEARCH_STREETID_MAX_LENGTH];   /**< POI街景Id描述 */
    char						szUid[NE_SEARCH_POI_UID_MAX_LENGTH];	/** POI点的服务端uid */
    bool						isMadian;			/** 标识是否点击麻点*/
    int						FocusIndex;			/** 所点击麻点的索引*/
    BNA_Search_POIChild_Info_t children[NE_SEARCH_POI_CHILDREN_MAX_LENGTH];
    int						unChildrenCnt; /*number of poi children*/
    _BNASearchPoiInfo()
    {
        unID = 0;
        unType = 0;
        isMadian = false;
        unChildrenCnt = 0;
        unDistance = 0;
        unPhoneCnt = 0;
        nDistrictID = 0;
        nWeight = 0;
        memset(stName, 0, NE_SEARCH_POI_NAME_MAX_LENGTH);
        memset(stAliasName, 0, NE_SEARCH_POI_NAME_MAX_LENGTH);
        memset(stPhoneTable, 0, NE_SEARCH_POI_MAX_PHONE_CNT * NE_SEARCH_POI_PHONE_MAX_LENGTH);
        memset(usDesc, 0, NE_SEARCH_POI_DESC_MAX_LENGTH);
        memset(usStreetid, 0, NE_SEARCH_STREETID_MAX_LENGTH * sizeof(unsigned short));
        memset(szUid, 0, NE_SEARCH_POI_UID_MAX_LENGTH);
    }
}BNASearchPoiInfo;


typedef struct _BNASearchResultParams
{
    int requestID;
    int resultCount;
    BNASearchPoiInfo *poiInfo;
}BNASearchResultParams;

/** 简单POI信息 **/
typedef struct _NA_Map_Item_t
{
    char uid[NE_SEARCH_POI_UID_MAX_LENGTH]; /**< POI uid */
    unsigned int uidLength;
    char name[NE_SEARCH_POI_NAME_MAX_LENGTH]; /**< POI 名称 */
    unsigned int nameLength;
    char address[NE_SEARCH_POI_DESC_MAX_LENGTH]; /**< POI 描述，作为地址 */
    unsigned int addrLength;
    double longitude; /**< 经度 */
    double latitude; /**< 纬度 */
}NA_Map_Item_t;

//road guide struct start
#define NE_FILENAME_MAX_LENGTH					32	/**< 文件名最大长度 */
#define NE_ROADNAME_MAX_LENGTH					32	/**< 道路名最大长度 */
#define NE_CROSS_SHAPE_MAX_CNT                  150 /**< 路口诱导标shape最大数量 */
#define NE_HUD_MAX_LANE_COUNT                   16  /**< HUD 最大车线数 */

/** 坐标位置，坐标系类型WGS84 */
typedef struct _AD_NE_Pos_t
{
    double x;	/** x */
    double y;	/** y */
}AD_NE_Pos_t;





// 消息结构定义
typedef struct _NC_MsgParam_t
{
    int   unMsgID ; //消息ID
    int   wParam ;  //消息参数1
    void*   lParam ;  //消息参数2
} NC_MsgParam_t ;


/** 路线计算结果 */
typedef struct _NC_RoutePlan_Result_t
{
    int						unRPPreference;	/**<  路线规划偏好，NE_RoutePlan_Preference_Enum按位与 */
    int						unRouteCnt;		/**<  路线数量 */
    bool					bReRoute;		/**<  是否是偏航重计算 */
    int						unYawJudgeTime; /**<  偏航判断时间，单位毫秒 */
    int						unYawJudgeDist;	/**<  偏航判断距离，单位米 */
    int						unCalcTime;		/**<  算路时间(包含网络不好反复重试时间)，单位毫秒 */
    int                     nRouteCnt;      /**<  算路条数 */
}NC_RoutePlan_Result_t;



///////////////////////////////////定位服务相关///////////////////////////////
//GPS定位服务的信息数据结构体
struct BNA_LOCATION_SERVICE_INFO
{
    char date_time[20];       //时间字符串
    char status;                    //接受状态：A有效定位，V无效定位
    double latitude;            //纬度
    double longitude;         //经度
    char north_south;         //南北极
    char east_west;             //东西经
    double  speed;             //速度
    double high;                 //高度
    int satellite_num;         //卫星个数
    double course;             //方向角度
};

//定位服务的点信息
struct BNA_LOCATION_SERVICE_POINT
{
    double latitude;
    double longitude;
};

//行政区划类型
enum BNA_LOCATION_DISTRICT_TYPE
{
    BNA_LOCATION_DISTRICT_TYPE_INVALID      = -1,     //无效值
    BNA_LOCATION_DISTRICT_TYPE_WORLD       = 0,       //世界
    BNA_LOCATION_DISTRICT_TYPE_NATION      = 1,        //国家
    BNA_LOCATION_DISTRICT_TYPE_PROVINCE  = 2,        //省
    BNA_LOCATION_DISTRICT_TYPE_CITY            = 3,        //市
    BNA_LOCATION_DISTRICT_TYPE_COUNTY     = 4,         //县
};

//行政区划信息
struct BNA_LOCATION_DISTRICT_INFO
{
    int  dist_type;                                                                                          //区域类型，取值对应到上面行政区类型
    int  dist_id;                                                                                               //区域ID
    int  province_id;                                                                                       //省份ID
    int  city_id;                                                                                                //城市ID
    char us_name[BNA_LOCATION_DISTRICT_NAME_MAX_LENGTH];      //英文名称
    BNA_LOCATION_SERVICE_POINT center_point;                                    //中心点坐标
    unsigned int  child_cnt;                                                                           //子行政区数量
};

//方向
enum BNA_LOCATION_DIRECTION_INFO
{
    BNA_LOCATION_DIRECTION_INVALID           = 0,  //无效值
    BNA_LOCATION_DIRECTION_EAST                 = 1,  //东
    BNA_LOCATION_DIRECTION_WEST                = 2,  //西
    BNA_LOCATION_DIRECTION_SOUTH             = 3,  //南
    BNA_LOCATION_DIRECTION_NORTH             = 4,  //北
    BNA_LOCATION_DIRECTION_EAST_NORTH   = 5,  //东北
    BNA_LOCATION_DIRECTION_EAST_SOUTH   = 6,  //东南
    BNA_LOCATION_DIRECTION_WEST_NORTH  = 7,  //西北
    BNA_LOCATION_DIRECTION_WEST_SOUTh   = 8,  //西南
};


///////////////////////////////////底图服务相关///////////////////////////////
//地图浏览模式设置
enum BNA_MAP_CONTROL_MODE_TYPE {
    BNA_MAP_CONTROL_MODE_INVALID = -1,
    BNA_MAP_CONTROL_MODE_HEADINGUP,         //车头朝上
    BNA_MAP_CONTROL_MODE_NORTHUP,           //正北朝上
    BNA_MAP_CONTROL_MODE_BIRDVIEW,          //鸟瞰模式
    BNA_MAP_CONTROL_MODE_DRIVERVIEW         //驾驶模式
};

//地图style设置
enum BNA_MAP_CONTROL_STYLE_TYPE {
    BNA_MAP_CONTROL_STYLE_DEFAULT = 1,        //缺省状态
    BNA_MAP_CONTROL_STYLE_DAY,                      //白天模式
    BNA_MAP_CONTROL_STYLE_NIGHT,                  //夜晚模式
};

//地图Layer类型
enum BNA_MAP_CONTROL_LAYER_TYPE {
    BNA_MAP_CONTROL_LAYER_INVAILD = -1,                     //无效值
    BNA_MAP_CONTROL_LAYER_BASE_MAP,                        //底图图层
    BNA_MAP_CONTROL_LAYER_START,                                //起点
    BNA_MAP_CONTROL_LAYER_END,                                  //终点
    BNA_MAP_CONTROL_LAYER_POI,                                   //普通POI点图层
    BNA_MAP_CONTROL_LAYER_POI_BKG,                           //麻点图层
    BNA_MAP_CONTROL_LAYER_SEARCH_CENTER,             //周边搜索中心点
    BNA_MAP_CONTROL_LAYER_FAV_POI,                            //收葬的普通POI点
    BNA_MAP_CONTROL_LAYER_FAV_MARK,                        //收藏夹地图点选POI
    BNA_MAP_CONTROL_LAYER_ROUTE_NODE,                  //导航路线节点
    BNA_MAP_CONTROL_LAYER_COMPASS,                        //指南针图标
    BNA_MAP_CONTROL_LAYER_ROUTE,                             //导航路线
    BNA_MAP_CONTROL_LAYER_POPUP,                             //气泡图层
    BNA_MAP_CONTROL_LAYER_BASE_POI,                         //底图可点的北京POI
    BNA_MAP_CONTROL_LAYER_ROUTE_SPEC,                    //导航路线详情
    BNA_MAP_CONTROL_LAYER_LOCATION,                        //定位图层
    BNA_MAP_CONTROL_LAYER_GUIDE_LINE,                     //引导线图层,如果增加层定义
//    BNA_MAP_CONTROL_LAYER_FAV_POIL,                          //收藏POI图层
    BNA_MAP_CONTROL_LAYER_DYNAMIC_MAP,                    //动态底图图层
    BNA_MAP_CONTROL_LAYER_AVOID_LINE,                     //路线规避图层
    BNA_MAP_CONTROL_LAYER_JUCV_VIEW,                       //放大图图层
    BNA_MAP_CONTROL_LAYER_TRACK,                              //轨迹图层
    BNA_MAP_CONTROL_LAYER_ROUTE_CRUISE,                //巡航预测图层
    BNA_MAP_CONTROL_LAYER_TRACK_CAR,                       //轨迹车点图层
    BNA_MAP_CONTROL_LAYER_LANDMARK,
    BNA_MAP_CONTROL_LAYER_BILLBOARD_ARC,
    BNA_MAP_CONTROL_LAYER_ROUTE_CAMERA,
    BNA_MAP_CONTROL_LAYER_ROUTE_TRAFFIC_JAM,
    BNA_MAP_CONTROL_LAYER_ROUTE_TRAFFIC_SIGN,
    BNA_MAP_CONTROL_LAYER_ROUTE_ICON,
    BNA_MAP_CONTROL_LAYER_ORG_GPS,
    BNA_MAP_CONTROL_LAYER_FIX_GPS,
    BNA_MAP_CONTROL_LAYER_MMGPS,
    BNA_MAP_CONTROL_LAYER_UGC_POPUP,
    BNA_MAP_CONTROL_LAYER_REROUTE_POPUP,
    BNA_MAP_CONTROL_LAYER_SELECT_LINK,
    BNA_MAP_CONTROL_LAYER_ELECTRIC_VEHICLE,             //电动车行驶距离图层
    BNA_MAP_CONTROL_LAYER_RECT_REGION
};

//定义Latlon
struct BNA_MAP_CONTROL_LATLON {
    double longitude;
    double latitude;
};

//图区坐标点信息
struct BNA_MAP_CONTROL_MERCATOR_POINTINFO {
    BNA_MAP_CONTROL_LATLON       stPoint;        //坐标点
    unsigned int                 point_id;       //id
};

//定义POINT
struct BNA_MAP_CONTROL_POINT {
    double x;
    double y;
    double z;
};

//定义RECT
struct BNA_MAP_CONTROL_RECT {
    int left;
    int right;
    int top;
    int bottom;
};

//定义QUAD
struct BNA_MAP_CONTROL_QUAD {
    BNA_MAP_CONTROL_POINT lb;       //left bottom
    BNA_MAP_CONTROL_POINT lt;        //left top
    BNA_MAP_CONTROL_POINT rb;       //right bottom
    BNA_MAP_CONTROL_POINT rt;        //right top
};

//定义街景数据
struct BNA_MAP_CONTROL_STREET_SCAPE_DATA {
    char status_id[64];         //因为街景现实Android精度不够等原因,体那家字段后续可能会删掉,此项不够合理
};

//定义map_status
struct BNA_MAP_CONTROL_MAP_STATUS {
    double                                     level;                             //比例尺:3-19
    double                                     rotation;                       //旋转角度
    double                                     over_looking;               //俯视角度
    BNA_MAP_CONTROL_POINT pt_center;                     //地图中心点
    BNA_MAP_CONTROL_QUAD map_round;                  //屏幕范围   屏幕地理坐标
    BNA_MAP_CONTROL_RECT   win_round;                   //屏幕范围    屏幕坐标
    BNA_MAP_CONTROL_POINT pt_offset;                      //偏移量
    //BNA_MAP_CONTROL_STREET_SCAPE_DATA street_data; //街景数据
};

//动画格式
 enum  BNA_MAP_CONTROL_ANIMATION_TYPE
{
    BNA_MAP_CONTROL_ANIMATION_NONE              = 0,
    BNA_MAP_CONTROL_ANIMATION_POS                  = 0x00000001,
    BNA_MAP_CONTROL_ANIMATION_ROTATE            = 0x00000010,
    BNA_MAP_CONTROL_ANIMATION_OVERLOOK      = 0x00000100,
    BNA_MAP_CONTROL_ANIMATION_LEVEL               = 0x00001000,
    BNA_MAP_CONTROL_ANIMATION_ALL                   = 0x00001111,
    BNA_MAP_CONTROL_ANIMATION_INELLIGENT     = 0x10000000,
    BNA_MAP_CONTROL_ANIMATION_FROGLEAP       = 0x10000001,
    BNA_MAP_CONTROL_ANIMATION_FLYOVER           = 0x10000010,
};

//屏幕坐标
struct BNA_MAP_CONTROL_SCREEN_POINT{
    int x;
    int y;
};

//坐标位置,坐标系类型 gcj02
struct BNA_MAP_CONTROL_GEO_POINT{
    double x;
    double y;
};

//定义offset点结构
struct BNA_MAP_CONTROL_OFFSET_POINT {
    float x;        // X
    float y;        // Y
    BNA_MAP_CONTROL_OFFSET_POINT(): x(0), y(0) {}
    BNA_MAP_CONTROL_OFFSET_POINT(float dx, float dy): x(dx), y(dy) {}
};

struct BNA_MAP_CONTROL_ITEM_INFO {
    char uid[NE_SEARCH_POI_UID_MAX_LENGTH];         //poi uid
    unsigned int uid_length;

    char name[NE_SEARCH_POI_NAME_MAX_LENGTH];       //poi name
    unsigned int name_length;

    char address[NE_SEARCH_POI_DESC_MAX_LENGTH];    //poi address
    unsigned int addr_length;

    double longitude;                               //经度
    double latitude;                                //纬度
    BNA_MAP_CONTROL_LAYER_TYPE layer_type;          //所在图层
};


typedef struct _BNA_MAP_CONTROL_DYNAMIC_POINT
{
    const char*   name;     /**< POI名称 */
    double  longitude;		/**< 经度*100000 */
    double  latitude;		/**< 纬度*100000 */
    const char*   tag;      /**< 图标标识符 */
} BNA_MAP_CONTROL_DYNAMIC_POINT;

///////////////////////////////////语音服务///////////////////////////////
 struct BNA_VOICE_TYPE_MODE{};

 struct BNA_VOICE_STRING{};

 struct BNA_VOICE_CALLBACK{};

 struct BNA_VOICE_TASK_COMPLETION_HANDLER{};

 struct BNA_VOICE_TASK_PRIORITY{};

 struct BNA_VOICE_TASK{};

  struct BNA_VOICE_VOICE_TYPE{};


///////////////////////////////////离线数据///////////////////////////////

#define BNA_DISTRICT_NAME_MAX_LENGTH (32)

typedef struct
{
    /**
     * @brief mName 省份名字
     */
    char mName[BNA_DISTRICT_NAME_MAX_LENGTH];

    /**
     * @brief mNameLength 省份名字符数
     */
    int mNameLength;

    /**
     * @brief mProvinceId 省ID
     */
    int mProvinceId;

    /**
     * @brief mStatus 引擎状态
     */
    int mStatus;

    /**
     * @brief mTotalDownloadSize 下载数据大小 单位 字节
     */
    unsigned int mTotalDownloadSize;

    /**
     * @brief mDownloadedSize 已下载数据大小 单位 字节
     */
    unsigned int mDownloadedSize;

    /**
     * @brief mDownloadProgress 下载进度
     */
    int mDownloadProgress;

    /**
     * @brief mTotalUpdateSize 更新数据大小 单位 字节
     */
    unsigned int mTotalUpdateSize;

    /**
     * @brief mUpdatedSize 已下载的更新数据大小
     */
    unsigned int mUpdatedSize;

    /**
     * @brief mUpdateProgress 更新进度
     */
    int mUpdateProgress;

    /**
     * @brief mDownloadProgressBy10 引擎返回的下载进度，应除以10再使用
     */
    int mDownloadProgressBy10;

    /**
     * @brief mUpdateProgressBy10 引擎返回的更新进度，应除以10再使用
     */
    int mUpdateProgressBy10;

    /**
     * @brief mTaskStatus 下载任务的状态，供UI使用
     */
    int mTaskUiStatus;

    /**
     * @brief mHasNewerVersion 是否有更新的版本发布
     */
    bool mHasNewerVersion = false;

    /**
     * @brief mIsRequesting 是否正在请求下载中
     */
    bool mIsRequesting = false;

    /**
     * @brief mIsSuspendByNetChange 是否由于网络变化而暂停
     */
    bool mIsSuspendByNetChange = false;

    /**
     * @brief mIsSuspendByPhoneChange 是否由于来电而暂停
     */
    bool mIsSuspendByPhoneChange = false;

    /**
     * @brief mHasNeccessaryData 是否具有完整的数据，只要下载完成，无论是否需要更新，都是true
     */
    bool mHasNeccessaryData = false;

} BNAOfflineDataInfo;

typedef enum
{
    BNA_DATA_STATUS_UNDOWNLOAD = 0,           /**<  未下载 */

    BNA_DATA_STATUS_DOWNLOADING = 1,			/**<  下载中 */

    BNA_DATA_STATUS_DOWNLOADED = 2,			/**<  已下载 */

    BNA_DATA_STATUS_NEED_UPDATE = 3,			/**<  需要更新 */

    BNA_DATA_STATUS_UPDATING = 4,             /**<  更新中   */

    BNA_DATA_STATUS_ALL = 5,                  /**<  所有的状态，上层取列表的时候用 */

} BNAOfflineDataStatus;

 typedef enum
 {
     /**
      * 未下载的
      */
     BNA_UI_STATUS_UNDOWNLOAD = 1,

     /**
      * 正在下载
      */
     BNA_UI_STATUS_DOWNLOADING = 2,

     /**
      * 等待下载
      */
     BNA_UI_STATUS_DOWNLOAD_WAITING = 3,

     /**
      * 暂停下载
      */
     BNA_UI_STATUS_DOWNLOAD_SUSPENDED = 4,

     /**
      * 完成下载
      */
     BNA_UI_STATUS_FINISHED = 5,

     /**
      * 网络异常
      */
     BNA_UI_STATUS_NET_ERROR = 6,

     /**
      * wifi异常
      */
     BNA_UI_STATUS_WIFI_ERROR = 8,

     /**
      * SD卡异常
      */
     BNA_UI_STATUS_SDCARD_ERROR = 9,

     /**
      * 需要更新
      */
     BNA_UI_STATUS_NEED_UPDATE = 10,

     /**
      * 等待更新
      */
     BNA_UI_STATUS_UPDATE_WAITING = 11,

     /**
      * 更新中
      */
     BNA_UI_STATUS_UPDATING = 12,

     /**
      * 暂停更新
      */
     BNA_UI_STATUS_UPDATE_SUSPENDED = 13,

     /**
      * 更新完成
      */
     BNA_UI_STATUS_UPDATE_FINISHED = 14,

     /**
      * 更新重启生效
      */
     BNA_UI_STATUS_UPDATE_RESTART = 15,

     /**
      * 数据merge开始
      */
     BNA_UI_STATUS_UPDATE_MERGE_START = 16,

     /**
      * 数据merge等待
      */
     BNA_UI_STATUS_UPDATE_MERGE_WAIT = 17,

     /**
      * 数据merge成功
      */
     BNA_UI_STATUS_UPDATE_MERGE_SUCCESS = 18,

     /**
      * 数据merge失败
      */
     BNA_UI_STATUS_UPDATE_MERGE_FAIL = 19,

 } BNATaskUiStatus;

#define BNA_MAX_PROVINCE_NUM            (34)

#define BNA_MAX_BUFFER_LENGTH           (16)
#define BNA_MAX_APKINFO_LENGTH          (2048)
#define BNA_MAX_DATA_FILE_MD5_LENGTH    (33)            // 离线dat数据文件的md5值的长度
#define BNA_MAX_DATA_VERSION_LENGTH     (16)            // 格式版本与数据版本长度
#define BNA_MAX_URL_LENGTH              (128)           // URL长度

/** 服务器上所有的版本信息 */
typedef struct
{
    char updateTime[BNA_MAX_BUFFER_LENGTH];         /**<  更新时间 */
    unsigned int timeLength;
    char apkVersion[BNA_MAX_BUFFER_LENGTH];			/**<  安装APK版本号*/
    unsigned int versionLength;
    unsigned int apkSize;                           /**<  APK的大小*/
    char apkInfo[BNA_MAX_APKINFO_LENGTH];           /**<  新建APK的特性*/
    unsigned int infoLength;

    char url[BNA_MAX_URL_LENGTH];					/**<  更新的URL地址*/
    unsigned int urlLength;
    int apkVersionCode;                             /**<  新apk文件的VersionCode,以大小表示是否更新*/
    char MD5[BNA_MAX_DATA_FILE_MD5_LENGTH];         /**<  dat文件m5值 定长32字节 */
    unsigned int md5Length;

} BNANewApkInfo;





///////////////////////////////////Type For Multiple Module///////////////////////////////
typedef enum
  {
      SEARCH_TYPE,
      SEARCHSUG_TYPE,
  }SearchType;

typedef struct _BNASearchRequest
{
    size_t clientId;
    int requestId;
    SearchType type;
    BNASearchCircleRegion circleRegion;
    char* name;
    char* regionName;
    int poiListCount;
    BNASearchPoiInfo* poiInfoList;
    BNASearchSugResult sugPoiTable;
    int districtId;
    int curPage;
    int pageCount;
    bool isLastPage;
    _BNASearchRequest()
    {
        clientId = 0;
        requestId = 0;
        type = SEARCH_TYPE;
        name = NULL;
        regionName = NULL;
        poiListCount = 0;
        poiInfoList = NULL;
        districtId = 0;
        curPage = 0;
        isLastPage = false;
    }

    ~_BNASearchRequest()
    {
        if (name != NULL) {
            free(name);
            name = NULL;
        }
    }

}BNASearchRequest;

typedef struct _BNASearchSugArg
{
    BNASearchRequest * preq;
    char * name;
}BNASearchSugArg;

typedef struct _BNARouteInfo {
    uint8_t index;
    uint8_t routeType;
    int32_t topLeftLongitude;
    int32_t topLeftLatitude;
    int32_t bottomRightLongitude;
    int32_t bottomRightLatitude;
    std::vector<uint8_t> tbts;
    std::vector<std::string> roads;
    std::vector<uint32_t> meters;
    std::vector<uint8_t> trafficStatus;
    uint32_t fare;
    uint32_t trafficLight;
    uint32_t distance;
    uint32_t duration;
    std::vector<std::string> interchanges;
    bool isUsingLocalCarInfo;
}BNARouteInfo;

typedef enum _BNARequestStatus
{

    BNA_REQUEST_STATUS_SUCCESS,
    BNA_REQUEST_STATUS_FAILED,
    BNA_REQUEST_STATUS_TIMEOUT,
    BNA_REQUEST_STATUS_UNKOWN

}BNARequestStatus;

typedef enum _BNARouteGeneratorStatus {
    BNAROUTE_REQUEST_STATUS_NONE,
    BNAROUTE_REQUEST_STATUS_PREPARE,
    BNAROUTE_REQUEST_STATUS_START,
    BNAROUTE_REQUEST_STATUS_REQUESTING,
    BNAROUTE_REQUEST_STATUS_FINSH,
    BNAROUTE_REQUEST_STATUS_CANCEL
}BNARouteGeneratorStatus;

typedef struct _BNARouteGeneratorRequest {
    uint64_t requestId;
    int routeCount;
    BNACoordinate startPos;
    BNACoordinate destPos;
    std::vector<BNACoordinate> wayPoint;
    std::vector<std::string> routeOptions;
    std::vector<uint32_t> routeValues;
    bool useOnline;
    BNARouteGeneratorStatus requestStatus;
    uint64_t handle;
    std::vector<BNARouteInfo> routeInfo;
    BNACarInfo carInfo;

    _BNARouteGeneratorRequest()
    {
        useOnline = true;
    }
} BNARouteGeneratorRequest;

  typedef struct _BNA_Map_ScreenShot_Image_t
  {
      int                        unImageWidth;
      int                        unImageHeight;
      char*                       pbtImageData;
  }BNA_Map_ScreenShot_Image_t;

#endif // NAVI_CAR_ADAPTER_DEF_H
