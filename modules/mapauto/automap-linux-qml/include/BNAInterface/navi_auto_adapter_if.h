//
//  navi_auto_adapter_if.hpp
//  baiduNaviSDK
//
//  Created by griffin on 16/9/22.
//  Copyright © 2016年 baidu. All rights reserved.
//

#ifndef navi_auto_adapter_if_hpp
#define navi_auto_adapter_if_hpp

#include <stdio.h>
#include <string>
#include <vector>

#include "BNATools/navi_car_adapter_def.h"
#include "BNAObserver/IAdapterMsgObserver.h"
#include "BNATools/navi_share_memory.h"

USING_NAMESPACE_ADAPTER

///////////////////////////////////////////Adapter API///////////////////////////////////////////
void BNAInit(const std::string &cfgPath, const std::string &uuid);

//////////////////////////////////////////MapRender API//////////////////////////////////////////
void BNAInitOPenGL(BNAScreenType type);

void BNAResizeMap(int width, int height, BNAScreenType type);

void BNADrawFrameMap(BNAScreenType type);

void BNALoadMapResource(BNAScreenType type);

void BNAOnMapResume(BNAScreenType type);

void BNAOnMapPause(BNAScreenType type);

void BNAResizeMiniMap(int width, int height, BNAScreenType type);

void BNADrawMiniMap(BNAScreenType type);

//////////////////////////////////////////Route Plan//////////////////////////////////////////

/**
 * @brief Set RoutePlan preference
 * @details RoutePlan has some options to set. like avoid traffic jam, recommend route, highway prefer.
 * @param[in] requestId the unique id for you to set the routepaln preference
 * @param[in] useOnline if you change the net mode
 * @param[in] options the options name that you want to set.
 * @param[in] optionValues the values of your options
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNASetRoutePlanPreference(const uint64_t requestId, const bool useOnline, std::vector<std::string>options, std::vector<uint32_t> optionValues);

BNARequestStatus BNAGetRoutePreference(const uint64_t requestId, std::vector<std::string> &options, std::vector<uint32_t> &optionValues);
/**
 * @brief GetAvailableRoutePlanPreferenceList
 * @details Before you set the routePlan preference, you may know the options that engine supported.
 * @param[out] optionList
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGetAvailableRoutePlanPreferenceList( std::vector<std::string> &optionList);

void BNADestroyRouteInfoByRequestId(const uint64_t requestId);

BNARequestStatus BNASetLocalCarInfo(const uint64_t requestId, const BNACarInfo& carInfo);
/**
 * @brief SetStartPosition
 * @details Setting the start point of each route
 * @param[in] requestId
 * @param[in] startPosition
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNASetStartPosition(const uint64_t requestId, const BNACoordinate startPosition);

/**
 * @brief GetStartPosition
 * @details Getting the start point of each route
 * @param[in] requestId the requestId of this routeGenerator
 * @param[out] startPosition the start point
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGetStartPosition(const uint64_t requestId, BNACoordinate &startPosition);

/**
 * @brief CleanStartPosition
 * @details clean the start position, this function just erase position from the cache
 * @param[in] requestId the requestId of this routeGenerator
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNACleanStartPosition(const uint64_t requestId);

/**
 * @brief SetDestinationPosition
 * @details Setting the destination point of each route
 * @param requestId the requestId of this routeGenerator
 * @param destinationPosition the destination  point
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNASetDestinationPosition(const uint64_t requestId, const BNACoordinate destinationPosition);

/**
 * @brief GetDestinationPosition
 * @details Getting the destination point from the cache
 * @param[in] requestId the requestId of each routeGenerator
 * @param[out] destination the destination point
 * @return BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGetDestinationPosition(const uint64_t requestId, BNACoordinate &destination);

/**
 * @brief CleanDestinationPosition
 * @details Erase the destination point from the cache
 * @param requestId[in] the requestId of each routeGenerator
 * @return BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNACleanDestinationPosition(const uint64_t requestId);

/**
 * @brief SetWayPoints
 * @details Setting the waypoints for each routeGenerator
 * @param requestId[in] the requestId of each routeGenerator
 * @param wayPoints[in] the wayPoints array
 * @return BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNASetWayPoints(const uint64_t requestId, std::vector<BNACoordinate> wayPoints);

/**
 * @brief GetWayPoints
 * @details Getting the wayPoints from cache
 * @param[in] requestId the requestId of each routeGenerator
 * @param[out] wayPoints the wayPoints array
 * @return BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGetWayPoints(const uint64_t requestId, std::vector<BNACoordinate> &wayPoints);

/**
 * @brief CleanWayPoints
 * @details Erase the waypoints from the cache
 * @param[in] requestId the requestId of each routeGenerator
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNACleanWayPoints(const uint64_t requestId);

/**
 * @brief Generator begin route generator
 * @details before generator,you should set the start position, destination position and set the route options(preference).
 * @param[in] requestId you should create the unique id by yourself, and cache it.
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGenerator(const uint64_t requestId, uint64_t handle);

/**
 * @brief CancelRoutePlan
 * @details That means you want to delete the cache of the results of this requestId, if this time still have no results, it will be stop the route plan.
 * @param[in] requestId you should create the unique id by yourself, and cache it.
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNACancelRoutePlan(const uint64_t requestId);

/**
 * @brief SelectRoute
 * @details If you want to highlight the route on the map.
 * @param[in] requestId whitch routeGenerator(routePlan) you want to control.
 * @param[in] routeIndex whitch route you want to highlight.
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNASelectRoute(const uint64_t requestId, const int routeIndex);

/**
 * @brief BNAGetRouteGeneratorStatus
 * @details If you want to know the status of the engine generator, like none, finished, cancel.
 * @param[out] GeneratorStatus NONE FINISHED CANCEL
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 */
BNARequestStatus BNAGetRouteGeneratorStatus(const uint64_t requestId, BNARouteGeneratorStatus& generatorStatus);

/**
 * @brief GetRoutesCount
 * @param[in] requestId the unique id to get the right routeInfo
 * @param[out] count
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT....
 */
BNARequestStatus BNAGetRoutesCount(const uint64_t requestId, int &count);

/**
 * @brief BNAGetPlaceId
 * @param[in] requestId
 * @param[in] longitude
 * @param[in] latitude
 * @return BNARequestStatus SUCCESS FAILED TIMEOUT....
 */
BNARequestStatus BNAGetPlaceId(int32_t &longitude, int32_t &latitude, std::string &placeId);

/**
 * @brief GetRouteInfoByRequestIdAndRouteIndex
 * @details Get the routeInfo.
 * @param[in] requestId this is the unique id of each routeGeerator.
 * @param[in] routeIndex  this is the index of each route(0,1,2...).
 * @param[out] routeInfo the routeInfo BNARouteInfo.
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT....
 */
BNARequestStatus BNAGetRouteInfoByRequestIdAndRouteIndex(const uint64_t requestId, const unsigned int routeIndex, BNARouteInfo &routeInfo);

/**
 * @brief Get the selected routeIndex
 * @details  If you want to know which route is highlight on the mapviewer \n
 * @param[in] requestId the right routeplan distinguished by requestId
 * @param[out]routeId the highlight route index
 * @retval BNARequestStatus SUCCESS FAILED TIMEOUT...
 * @author Hansen
 */
BNARequestStatus BNAGetCurrentSelectedRouteID(const int requestId, int& routeID);


BNARequestStatus BNAHasRoutePlanResult(const int requestId, bool &isReady);

void BNAStartRoutePlan(BNARouteNode *routeNodeArr, int nodesCount, BNACoordinateType coodinateType, BNARoutePlanPreferenceType preferenceType, int requestId);

//void BNACancelRoutePlan();

void BNAHasRouteResult();

void BNASetSelectedRouteByRouteID(const int requestId, int routeID);

//void BNAGetRoutePlanResult(BNARoutePlanResult &routePlanResultInfo);

//void BNAGetRouteDetailInfoByID(int routeID, BNARouteDetailInfo &routeInfo);

//void BNASetNavigationConfig(BNANavigationConfig naviConfig);

//////////////////////////////////////////Route Guidance//////////////////////////////////////////

void BNASetRoute(uint64_t routeId, int& result);

void BNAIsRouteSet(bool& routeSet);

void BNASetNavigationMapViewStatus(BNAGuideViewStatus viewStatus, BNAScreenType type);

void BNASetNavigationMapViewRotateMode(BNAGuideViewRotateMode mode, BNAScreenType type);

// Reresh route in guidance
int BNARefreshRoute();

bool BNAStartNavigation(BNAScreenType type, bool isSimulation = false);

bool BNAPauseNavigation();

bool BNAResumeNavigation();

bool BNAStopNavigation(BNAScreenType type);

bool BNASetCruiseConfig(BNACruiseConfig naviConfig);

bool BNAStartRouteCruise(BNAScreenType type = BNAScreenCenter);

bool BNAStopRouteCruise(BNAScreenType type = BNAScreenCenter);

bool BNAGetRouteGuideStatus(BNAGuideStatus& status, BNAGuideSubStatus& subStatus);

bool BNAGetVehicleInfo(BNAGuideVehicleInfo& info);

bool BNAGetSimpleGuidanceInfo(BNAGuidanceSimpleMapInfo &info);

bool BNAGetRouteRemainInfo(BNAGuideRemainInfo &info);

bool BNAGetAssistantGuidanceInfo(BNAGuideAssistantInfo &assistantInfo);

bool BNAGetRasterExpandMapInfo(BNAGuideRasterExpandMapInfo& rasterExpandMapInfo, std::string& bufferPath);

bool BNAGetRasterExpandMapImage(const char* pImagePath, const unsigned int imageType, unsigned char** pByteBuf, unsigned int& length);

bool BNAGetRemainDistance(int& distance);

bool BNAGetRemainTime(int& time);

bool BNAGetRoadName(char* roadName);

bool BNAGetRoadType(int& roadType);

bool BNAGetCurrentAddress(std::vector<std::string>& regin,  int& districtId,  int& zipCode);

bool BNAGetVoiceText(std::string& resText, bool& bPreempt, int& unLevel);

bool BNAGetLaneInfo(BNAGuidanceLaneInfo &laneInfo);

bool BNAGetHighWayInfo(BNAHighWayInfo &highWayInfo);

bool BNASetMapAutoLevelStatus(bool  status);

bool BNASetMapMemoryScale(int  memoryScale);

bool BNASetStopRouteGuideSpeak(bool status);

bool BNASetEnableRoadConditionAutoUpdate(bool bEnable);

bool BNAUpdateRoadCondtion(BNAUpdateRoadConditionEnum updateType = BNA_GUIDE_UPDATEROADCONTION_ONLYRC);

bool BNAGetCarProgress(float& fProgress);

bool BNAGetRoadCondition(std::vector<BNARoadConditionItem>& aRoadCondition);

bool BNAEnterViewAll(int type);

bool BNAExitViewAll();

bool BNASwitchRoute(const bool bSwitch = true, const unsigned int& routeIdx = 0);

bool BNAOnlineChangeRoute();

std::string BNAGetCurRoadNameByPos(int longitude, int latitude);

///////////////////////////////////////////Search API////////////////////////////////////////////


BNARequestStatus BNASearchCreateRequestId(int& requestId);

BNARequestStatus BNASearchDestoryedRequestId(const int requestId);

BNARequestStatus BNASearchCancelByRequestId(int oldRequestId, int& newRequestId);

BNARequestStatus BNASearchAsync(const size_t clientId,
                                const int requestId,
                                BNASearchCircleRegion circleRegion,
                                const char* name,
                                const char* regionName,
                                int districtId,
                                int pageNum,
                                int pageCount);

BNARequestStatus BNASearchSugAsync(const int requestId, const char* name, int districtID);

BNARequestStatus BNASearchGetDistrictInfoByPoint(int32_t longitude, int32_t latitude, int& nDistrictID);

BNARequestStatus BNASearchIsLastPage(const int requestId, bool& isLastPage);

BNARequestStatus BNASearchGetResultItemCount(const int requestId, int& count);

BNARequestStatus BNASearchGetResultItemList(const int requestId, BNASearchPoiInfo** poiInfo);

BNARequestStatus BNASearchGetResultItemByIndex(const int index, const int requestId, BNASearchPoiInfo* poiInfo);

BNARequestStatus BNASearchGetSugResultList(const int requestId, BNASearchSugResult& poiInfo);

void BNASearchByName(int nDistrictID,
                             BNASearchCircleRegion stCircleRegion,
                             char* name,
                             unsigned int enSortType,
                             unsigned int& unPoiCount,
                             BNASearchPoiInfo* naPstPoiTable,
                             unsigned int unPageNumber,
                             bool& pbIsLastPage);

void BNASearchBySug(char* name,
                            int nDistrictID,
                            int unPoiCount,
                            BNASearchSugResult& naPstPoiTable );

void BNAGetDistrictInfoByPoint(double longitude, double latitude, int &nDistrictID);

BNARequestStatus BNASearchByRoute(
        const char *name,
        unsigned int distance,
        BNASearchPoiInfo* naPstPoiTable,
        unsigned int pageNum,
        unsigned int& pageCount,
        bool& pbIsLastPage);

void BNASearchByRouteAsync();
///////////////////////////////////////////Voice API////////////////////////////////////////////
std::vector<int8_t>& BNATextToPcm(const std::string& text);

///////////////////////////////////////////Other API////////////////////////////////////////////
void BNAAttachAdapterObserver(int msgID, IAdapterMsgObserver *observer);

void BNADetachAdapterObserver(int msgID, IAdapterMsgObserver *observer);

void BNACoordinateConvert(const char *from, const char *to, double oldx, double oldy, double &newx, double &newy);


///////////////////////////////////////////定位相关///////////////////////////////////////////
//show position
void BNALocationSetLocationMark(BNAScreenType type);

//hide position
void BNALocationHideLocationMark(BNAScreenType type);

// return MapMatched Location
bool BNALocationGetMapMatchInfo(BNAMapMatchResult& match_result);

//////////////////////////////////////////Simulation//////////////////////////////////////////
bool BNALoadGpsData(std::string gpsFile);

bool BNAStartSimulation(BNAScreenType type);

bool BNASetDemoCarSpeed(float changeSpeed);

bool BNASetSimulationType(BNALocationSimulationType simulationType);


///////////////////////////////////////////语音服务相关///////////////////////////////////////////
/*
 * @func    只合成文本不播放
 *
 * @param   [in]    text //需要播报的文本
 *
 * @return  TrueOrFalse
 */
bool BNAVoiceSynthesizeTTSTextSync(char* text);

/*
 * @func    停止当前播报
 *
 * @param
 *
 * @return
 */
void BNAVoiceStopTTSPlayer();


///////////////////////////////////////////底图相关：渲染、操作、POI操作///////////////////////////////////////////
/*
 * @func    设置地图style：日夜等
 *
 * @param   [in]    map_style //style of map
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetMapStyleType(BNA_MAP_CONTROL_STYLE_TYPE map_style, BNAScreenType type);

/*
 * @func    获取地图mode：正北朝上等
 *
 * @param   [out]    map_mode //mode of map
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetMapStyleType(BNA_MAP_CONTROL_STYLE_TYPE &map_style, BNAScreenType type);

/*
 * @func    获取地图mode：正北朝上等
 *
 * @param   [in]    map_mode //mode of map
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetMapModeType(BNA_MAP_CONTROL_MODE_TYPE map_mode, BNAScreenType type);

/*
 * @func    获取地图style：日夜等
 *
 * @param   [out]    map_style //style of map
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetMapModeType(BNA_MAP_CONTROL_MODE_TYPE &map_mode, BNAScreenType type);

/*
 * @func    设置地图跟随模式
 *
 * @param   [in]    bFollow /是否跟随自车点
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetFollowCar(bool bFollow, BNAScreenType type);

/*
 * @func    获取地图跟随模式
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlIsFollowCar(BNAScreenType type);

/*
 * @func    获取制定图层是否在显示状态
 *
 * @param   [in]    layer_style     //用来指示地图图层的样式类型
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetMapLayerShowStatus(BNA_MAP_CONTROL_LAYER_TYPE layer_style, BNAScreenType type);

/*
 * @func    设置指定图层是否显示
 *
 * @param   [in]    layer_style     //用来指示地图图层的样式类型
 * @param   [in]    is_show          //用来表示该图层是否该显示
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetMapLayerShowStatus(BNA_MAP_CONTROL_LAYER_TYPE layer_style, bool is_show, BNAScreenType type);

/*
 * @func    强制刷新图层数据
 *
 * @param   [in]    layer_style     //用来指示地图图层的样式类型
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlUpdateMapLayer(BNA_MAP_CONTROL_LAYER_TYPE layer_style, BNAScreenType type);

/*
 * @func    获取当前地图状态
 *
 * @param   [out]   ret_map_status       //用来返回指示地图图层的数据状态，包括级别，角度，中心点，屏幕等
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetMapStatus(BNA_MAP_CONTROL_MAP_STATUS& ret_map_status, BNAScreenType type);

/*
 * @func    设置地图状态
 *
 * @param   [in]   map_status       //用来指定地图图层的数据状态，包括级别，角度，中心点，屏幕等
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetMapStatus(BNA_MAP_CONTROL_MAP_STATUS map_status, BNAScreenType type, BNA_MAP_CONTROL_ANIMATION_TYPE animation = BNA_MAP_CONTROL_ANIMATION_NONE, unsigned long animation_time = 300);

bool BNAMapControlLocateMap(int lon, int lat, BNAScreenType type);

/*
 * @func    设置地图状态，有动画效果
 *
 * @param   [in]    map_status                  //用来指定地图图层的数据状态，包括级别，角度，中心点，屏幕等
 * @param   [in]    animation_time           //用来指定地图动画的执行时间等
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlMoveToMapStatus(BNA_MAP_CONTROL_MAP_STATUS map_status, unsigned long animation_time, BNAScreenType type);

/*
 * @func    导航地图的状态转换为地图状态
 *
 * @param   [in]   poi_latlon_list       //用来指定POI坐标的列表
 * @param   [in]   poi_count             //用来指定点的个数
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlMovePoiToCenter(BNA_MAP_CONTROL_LATLON* poi_latlon_list, unsigned int poi_count, BNA_MAP_CONTROL_OFFSET_POINT offset, BNAScreenType type);

/*
 * @func    导入离线地图
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlImportVmpMap();

/*
 * @func    是否打开路况
 *
 * @param   [in]    is_show   //用来指定是否显示路况
 *
 * @return  TrueOrFalse
 */
void BNAMapControlShowTrafficMap(bool is_show, BNAScreenType type);

/*
 * @func    获取到当前的比例尺等级
 *
 * @param   [in]    ret_level   //用来返回比例尺的等级
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetLevel(float& ret_level, BNAScreenType type);

/*
 * @func    放大地图
 *
 * @param
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlZoomIn(BNAScreenType type);

/*
 * @func    缩小地图
 *
 * @param
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlZoomOut(BNAScreenType type);

/*
 * @func    将地图缩放到指定大小，返回比例尺的大小
 *
 * @param   [in]    bound   //用来制定地图的区域范围
 * @param   [out]   ret_new_level   //用来返回当前比例尺的等级
 *
 * @return  TrueOrFalse
 */
void BNAMapControlGetZoomToBound(BNA_MAP_CONTROL_RECT bound, double& ret_new_level, BNAScreenType type);

/*
 * @func    暂停地图引擎
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlOnPause(BNAScreenType type);

/*
 * @func    暂停地图引擎
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlOnResume(BNAScreenType type);

/*
 * @func    设置指南针位置
 *
 * @param   [in]    x   //屏幕坐标x
 * @param   [in]    y   //屏幕坐标y
 * @param   [in]    hide_time   //隐藏时间
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlSetCompassPosition(int x, int y, int hide_time, BNAScreenType type);

/*
 * @func    开始请求地图数据
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlStartMapDataRequest(BNAScreenType type);

/*
 * @func    停止请求地图数据
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlStopMapDataRequest(BNAScreenType type);

/*
 * @func    聚焦地图元素数据
 *
 * @param   [in]    layer_style     图层类型
 * @param   [in]    item_id          地图元素ID
 * @param   [in]    is_focus         是否聚焦
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlFocusItem(BNA_MAP_CONTROL_LAYER_TYPE layer_style, int item_id, bool is_focus, BNAScreenType type);

/*
 * @func    将屏幕坐标转化为地理坐标
 *
 * @param   [in]    screen_point            用来指定屏幕的点坐标
 * @param   [in]    ret_geo_point          用来返回地理的点坐标
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetGeoPosByScreenPos(BNA_MAP_CONTROL_SCREEN_POINT screen_point, BNA_MAP_CONTROL_GEO_POINT& ret_geo_point, BNAScreenType type);

/*
 * @func    将地理坐标转化为屏幕坐标
 *
 * @param   [in]    ret_screen_point     用来返回屏幕的点坐标
 * @param   [in]    geo_point                用来指定地理的点坐标
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlGetScreenPosByGeoPos(BNA_MAP_CONTROL_SCREEN_POINT& ret_screen_point, BNA_MAP_CONTROL_GEO_POINT geo_point, BNAScreenType type);

/*
 * @func    将指定地理区域缩放至指定Bound
 *
 * @param   [in]    top_left_geo_point      左上角地理坐标
 * @param   [in]    bottom_right_geo_point  右下角地理坐标
 * @param   [in]    x                       bound左上角横坐标
 * @param   [in]    y                       bound左上角纵坐标
 * @param   [in]    width                   bound宽
 * @param   [in]    height                  bound高
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlFitGeoAreaToScreen(BNA_MAP_CONTROL_GEO_POINT top_left_geo_point, BNA_MAP_CONTROL_GEO_POINT bottom_right_geo_point,
                                     int x, int y, int width, int height, BNAScreenType type);

/*
 * @func    释放共享的地图的相关数据
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlReleaseShareMapData(BNAScreenType type);

/*
 * @func   更新地图导入的数据
 *
 * @param
 *
 * @return  TrueOrFalse
 */
void BNAMapControlUpdataShareMapData(BNAScreenType type);

/*
 * @func   初始化collada数据配置
 *
 * @param
 *
 * @return  TrueOrFalse
 */
bool BNAMapControlInitColladaConfig(BNAScreenType type);

/*
 * @func   从一点移动到另一个点
 *
 * @param   [in]    fromX   //start: x of screen
 * @param   [in]    fromY   //start: y of screen
 * @param   [in]    toX		//start: x of screen
 * @param   [in]    toY		//end: y of screen
 *
 * @return TrueOrFalse
 */
bool BNAMapControlPressMove(unsigned int fromX, unsigned int fromY,unsigned int toX, unsigned int toY, unsigned int speed, BNAScreenType type);

/*
 * @func   获得点击点的POI信息
 *
 * @param   [in]    x   //x of screen
 * @param   [in]    y   //y of screen
 * @param   [out]   uid //uid of poi
 *
 * @return TrueOrFalse
 */
bool BNAMapControlGetSelectItemInfo(unsigned int x, unsigned int y, BNA_MAP_CONTROL_ITEM_INFO &poi_info, BNAScreenType type);

/*
 * @func   获得长按点击点的信息
 *
 * @param   [in]    x   //x of screen
 * @param   [in]    y   //y of screen
 * @param   [out]   uid //uid of poi
 *
 * @return TrueOrFalse
 */
bool BNAMapControlGetAntiGeoInfo(unsigned int x, unsigned int y, BNA_MAP_CONTROL_ITEM_INFO &poi_info, BNAScreenType type);

/*
 * @func	POI添加Mark
 *
 * @param	[in]	bundleInfo	//bundleinfo
 *
 * @return	TrueOrFalse
 */
bool BNAMapControlAddPopUpData(BNA_MAP_CONTROL_LATLON geo_point, BNAScreenType type) ;

/*
 * @func	POI添加BkgPoint
 *
 * @param	[in]	pPointInfoList	//pPointInfoList
 * @param   [in]    iCount          //count of Point
 * @param   [in]    sugPoint        //suggest Point
 *
 * @return	TrueOrFalse
 */
bool BNAMapControlAddBkgData(const BNA_MAP_CONTROL_MERCATOR_POINTINFO*  p_point_info_list, int id_count, BNAScreenType type) ;

/*
 * @func    展示路线规划结果
 * @param   [in]    routes_id       //路线规划结果的唯一标识符
 * @return
 */
bool BNAMapControlAddRoutes(const uint64_t routes_id, BNAScreenType type);

/*
 * @func    展示电动车可行驶距离
 * @param   [in]    distance       //单位为米
 * @return
 */
bool BNAMapControlSetAvailableDistanceForEV(unsigned int distance, BNAScreenType type);

bool BNAMapControlSetDynamicMapPoints(BNA_MAP_CONTROL_DYNAMIC_POINT* points, int count, BNAScreenType type);

bool BNAMapControlSetDynamicMapImages(const char* tag, char* buffer, int width, int height, int channel, BNAScreenType type);

bool BNAMapControlCleanDynamicMapPoints(const char* tag, BNAScreenType type);

bool BNAMapControlCleanDynamicMapImages(const char* tag, BNAScreenType type);

bool BNAMapControlCleanDynamicMapAllData(BNAScreenType type);

//////////////////////////////////////////离线数据///////////////////////////////////////////

void BNAGetOfflineDataList(BNAOfflineDataInfo *list, unsigned int &size, BNAOfflineDataStatus status = BNA_DATA_STATUS_ALL);

void BNACheckNewVersion(bool &bAppUpdate, BNANewApkInfo &apkInfo, bool &bDataUpdate, int *provinceId, unsigned int count);

void BNAStartDownloadRequest(int provinceId);

void BNAStartDownloadData(int provinceId);

void BNASuspendDownloadData(int provinceId);

void BNAStartUpdateData(int provinceId);

void BNASuspendUpdateData(int provinceId);

void BNACancelUpdateData(int provinceId);

void BNASuspendAllDownload();

void BNARemoveData(int provinceId);

void BNAMapControlSetRegionData(const double*  p_point_info_list, int id_count, BNAScreenType type);

void BNAMapControlSetRegionStyle(float r,
                                 float g,
                                 float b,
                                 float a,
                                 BNAScreenType type);

void BNAMapControlClearRegionData(BNAScreenType type);

void BNAMapControlSetLineData(const double*  p_point_info_list, int id_count, BNAScreenType type);

void BNAMapControlSetLineStyle(float r,
                               float g,
                               float b,
                               float a,
                               float line_width,
                               BNAScreenType type);

void BNAMapControlClearLineData(BNAScreenType type);

#endif /* navi_auto_adapter_if_hpp */
