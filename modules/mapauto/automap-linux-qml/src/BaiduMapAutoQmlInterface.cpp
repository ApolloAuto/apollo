#include "BaiduMapAutoQmlInterface.h"
#include <iostream>
#include "utils/def.h"
#include "utils/amgeotools.h"
#include <QTime>
#include <iostream>
#include <stdio.h>
#include "BDPOISearchFilter.h"
#include "BDVersionInfo.h"
#include <unistd.h>

NAMESPACE_START
#define DEV_WINDOW_WIDTH  1280
#define DEV_WINDOW_HEIGHT 720

BaiduMapAutoQmlInterface::BaiduMapAutoQmlInterface(void)
{
    connect(this, SIGNAL(windowChanged(QQuickWindow*)),
            this, SLOT(OnWindowChanged(QQuickWindow*)));
    mDrawTimer = new QTimer(this);
    connect(mDrawTimer, SIGNAL(timeout()), this, SLOT(render()));
    mMouseDragTimer = new QTimer(this);
    mMouseDragTimer->setInterval(5000);
//    connect(mMouseDragTimer, SIGNAL(timeout()), this, SLOT(followCar()));

}

#define CFG_DATA_CONFIG "/opt/BaiduMapAuto/bnav"
void BaiduMapAutoQmlInterface::initBaiduMapEngine()
{

    BDGlobalInit(CFG_DATA_CONFIG, "38f9852a8359d77f53a24145fadc5d81");
    mMapMode = AM_MAP_MODE_NORTH;
    mMapAutoStatus = BNAMapAutoStatusNomal;

    mScreenType = BNAScreenCenter;

    // Map View Module
    m_pMap = new BDMapViewer(BDMapType::FRONT,  DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT);
    m_pMap->setEventListener(this);
    m_pMap->setElementVisible(BDMapElement::TRAFFIC, true);
//    m_pMap->setMapTheme(BDMapTheme::NIGHT_MODE1);
    m_pMap->setMapLevel(18);
//    m_pMap->setMapOffset(0,-200);
    m_pMap->enableFollowingCar();
    m_pMap->setViewMode(BDMapViewMode::NORTHUP);


//    m_pSubMap = new BDMapViewer(BDMapType::ASSISTANT,  DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT);
//    m_pSubMap->setMapTheme(BDMapTheme::NIGHT_MODE1);

    //打開路況
    m_pMap->setElementVisible(BDMapElement::TRAFFIC, true);
    mTrafficSwitchValue =  true;

    m_bShowMainMap = true;

    // POI Search Module
    m_pSearch = new BDPOISearch();
    m_pSearch->setEventListener(this);

    // Route Generator Module
    m_pRouteGenerator = new BDRouteGenerator();
    m_pRouteGenerator->setEventListener(this);

    //GuidanceModule
    m_pRouteGuide = BDRouteGuide::getInstance();
    m_pRouteGuide->setBasicEventListener(this);
    m_pRouteGuide->setViewEventListener(this);

    std::string num = BDGlobalGetTrialNumber();
    std::string isbn = BDGlobalGetMapAutoISBN();
    std::string version = BDGlobalGetMapAutoVersion();
    printf("num = %s, isbn = %s, version=%s\n",num.c_str(), isbn.c_str(), version.c_str());
//    setDefaultMapLocation();
}

void BaiduMapAutoQmlInterface::OnWindowChanged(QQuickWindow *pWindow)
{
    if(pWindow == Q_NULLPTR)
    {
        return;
    }
    connect( pWindow, SIGNAL( beforeRendering( ) ),
             this, SLOT( render( ) ), Qt::DirectConnection );
    pWindow->setClearBeforeRendering( false );
}

void BaiduMapAutoQmlInterface::exitApplication()
{
    exit(0);
}

/**
*	设置路况开关
*
*		@param value		[in] 开关值
*
*		@return 无
*		@note	无
*/
void BaiduMapAutoQmlInterface::setTrafficSwitch(bool value)
{
    if (m_pMap->isElementVisible(BDMapElement::TRAFFIC) != value)
    {
        mTrafficSwitchValue = value;
        AMSettingManager::getInstance()->setValue(Setting_Key_Traffic_Onoff, mTrafficSwitchValue);
        m_pMap->setElementVisible(BDMapElement::TRAFFIC, value);
    }
}
/**
 *	获取路况开关值
 *
 *		@param 无
 *
 *		@return 开关值
 *		@note	无
 */
bool BaiduMapAutoQmlInterface::getTrafficSwitch(void)
{
    return mTrafficSwitchValue;
}
/**
 *	渲染方法
 *
 *		@param 无
 *
 *		@return 无
 *		@note	引擎初始化放在这里，因为这时Surface才创建好
 */
void BaiduMapAutoQmlInterface::render()
{
    static  bool init_flag = false;
    if (!init_flag)
    {
        initBaiduMapEngine();
        init_flag = true;
    }
    if (m_bShowMainMap)
    {
        if (m_pMap){
            m_pMap->draw();
        }
    }
    else
    {
        if (m_pSubMap)
        {
            m_pSubMap->draw();
        }
    }
    return;

}

void BaiduMapAutoQmlInterface::Release( void )
{
    qDebug( "BaiduMapAutoQmlInterface::Release called." );
}

/**
*	设置默认位置
*
*		@param 无
*
*		@return 无
*		@note	无
*/
void BaiduMapAutoQmlInterface::setDefaultMapLocation(void)
{
    m_pMap->setMapLevel(18);
    double sLongtitude = 0.0f;
    double sLatitude = 0.0f;
    AMGeoTools::locate(sLongtitude, sLatitude);
    BDGeoCoord geoCoord(sLongtitude * 100000, sLatitude * 100000, 0);

    m_pMap->setMapPosition(geoCoord, true);
//    m_pMap->setMapPosition(BDGeoCoord(0, 0, 0), true);
}

/**
*	设置视角模式
*
*		@param mode [in] 模式值
*
*		@return 无
*		@note	这里的Mode value使用的是宏，而Widget中使用的是Enum
*/
void BaiduMapAutoQmlInterface::setMapMode(int mode)
{
    if (mode == AM_MAP_MODE_NORTH)
    {
        BDMapViewMode mode = BDMapViewMode::NORTHUP;
        m_pMap->setViewMode(mode);
    }
    else if (mode == AM_MAP_MODE_DIRECT)
    {
        BDMapViewMode mode = BDMapViewMode::HEADINGUP;
        m_pMap->setViewMode(mode);
    }
    else if (mode == AM_MAP_MODE_NEED_FIX)
    {

    }
    mMapMode = mode;
}

/**
*	获取视角模式值
*
*		@param 无
*
*		@return 模式值
*		@note	无
*/
int BaiduMapAutoQmlInterface::getMapMode(void)
{
    return mMapMode;
}

/**
*	放大显示比例
*
*		@param animated [in] 是否需要动画:true or false
*		@param interval [in] 动画间隔时间:S
*
*		@return true or false
*		@note	无
*/

int BaiduMapAutoQmlInterface::zoomIn(bool animated, unsigned int interval)
{
//    //for test setspeed; add by sfy; 2017-7-24
//    m_pRouteGuide->setSimulationSpeed(10);

//    //for test view all; add by sfy; 2017-7-24
//    m_pRouteGuide->enterViewAll(100, 200,  DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT-100, DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT);

    int cur_level = m_pMap->getMapLevel();
    int set_level_value = cur_level + interval;
    if (set_level_value < m_pMap->getMaxMapLevel())
    {
        m_pMap->setMapLevel(set_level_value);
        BDResult result1 = m_pRouteGuide->setMapMemoryScale(set_level_value);
        return set_level_value;
    }
    return cur_level;
}

/**
*	缩小显示比例
*
*		@param animated [in] 是否需要动画:true or false
*		@param interval [in] 动画间隔时间:S
*
*		@return true or false
*		@note	无
*/
int BaiduMapAutoQmlInterface::zoomOut(bool animated, unsigned int interval)
{
//    //for test setspeed; add by sfy; 2017-7-24
//    BDFloat ag = -10;
//    m_pRouteGuide->setSimulationSpeed(ag);

//    //for test view all; add by sfy; 2017-7-24
//    m_pRouteGuide->exitViewAll();

    int cur_level = m_pMap->getMapLevel();
    int set_level_value = cur_level - interval;
    if (set_level_value >= m_pMap->getMinMapLevel())
    {
        m_pMap->setMapLevel(set_level_value);
        BDResult result = m_pRouteGuide->setMapMemoryScale(set_level_value);
        return set_level_value;
    }
    return cur_level;
}
/**
*	导航中刷新更优路线
*
*/
void BaiduMapAutoQmlInterface::refreshRoute()
{
    m_pRouteGuide->refreshRoute();
    printf("小度已为你切换到更优路线");
}

void BaiduMapAutoQmlInterface::zoomInHeightPrecision(float interval)
{
    float level = m_pMap->getMapLevelHightPrecision();
    level += interval;
    if (level < m_pMap->getMaxMapLevel())
    {
        m_pMap->setMapLevelHightPrecision(level);
    }
    else
    {
        m_pMap->setMapLevel(m_pMap->getMaxMapLevel());
    }
}

void BaiduMapAutoQmlInterface::zoomOutHeightPrecision(float interval)
{
    float level = m_pMap->getMapLevelHightPrecision();
    level -= interval;
    if (level > m_pMap->getMinMapLevel())
    {
        m_pMap->setMapLevelHightPrecision(level);
    }
    else
    {
        m_pMap->setMapLevel(m_pMap->getMinMapLevel());
    }
}

/**
*	图区移动处理
*
*		@param mouse_x [in] x
*		@param mouse_y [in] y
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::mouseMove(int last_x, int last_y, int mouse_x, int mouse_y)
{
    if( mMapAutoStatus == BNAMapAutoStatusGuideance || mMapAutoStatus == BNAMapAutoStatusCruise)
    {
        BNASetNavigationMapViewStatus(BNA_GUIDE_VIEW_STATUS_BROWSE, BNAScreenCenter);
        mMouseDragTimer->start();
    }
    m_pMap->dragMap(last_x, last_y, mouse_x, mouse_y, 0);
    //    BDPoint pt(mouse_x, mouse_y);
    //    BDGeoCoord geoCoord;
    //    m_pMap->screenToGeoCoord(pt, geoCoord);
    //    m_pMap->setMapPosition(geoCoord, true);
    mLastTouchX = mouse_x;
    mLastTouchY = mouse_y;
    setMapMode(AM_MAP_MODE_NEED_FIX);
}
void BaiduMapAutoQmlInterface::focusPoi(int screen_x, int screen_y)
{
    baidu::mapauto::navi::search::BDPOIInfo  poi_info;
    if (m_pMap->getSelectItemInfo(screen_x, screen_y, poi_info) != BDResult::OK)
    {
        m_pMap->hidePoiMarker();
        emit noneSelected();
    }
}

void BaiduMapAutoQmlInterface::focusRoute(int screen_x, int screen_y)
{
    unsigned int indexRoute;
    if (m_pMap->getSelectRouteInfo(screen_x, screen_y, indexRoute) == BDResult::OK)
    {
        m_pMap->chooseRoute(indexRoute);
        //emit noneSelected();
    }
}

/**
*   反Geo
*
*		@param mouse_x [in] x
*		@param mouse_y [in] y
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::focusAntiGeo(int screen_x, int screen_y)
{
    baidu::mapauto::navi::search::BDPOIInfo  poi_info;
    if (m_pMap->getAntiGeoInfo(screen_x, screen_y, poi_info) != BDResult::OK)
    {
        m_pMap->hidePoiMarker();
        emit noneSelected();
    }
}

void BaiduMapAutoQmlInterface::focusPoiList(int index)
{
    // 仅适用于只有一个m_pSearch
    if (!m_pSearch)
    {
        return;
    }
    BDInt32 count = 10;
    std::vector<BDPOIInfo> tmp_list;
    std::vector<BDGeoCoord> poi_list;
    m_pSearch->getResultItemList(0, count, tmp_list);
    for (int i = 0; i < tmp_list.size(); i++)
    {
        BDPOIInfo tmppoi = tmp_list[i];
        poi_list.push_back(tmppoi.getPosition());
    }

    if (index >= count && index < 0)
    {
        return;
    }

    if (m_pMap)
    {
        m_pMap->addBkgPoiMarker(poi_list);
        m_pMap->addPoiMarker(poi_list[index]);
    }
    focusLonLat(poi_list[index]);
}

/**
*   根据坐标定位中心点
*
*		@param longitude [in] longitude
*		@param latitude [in]  latitude
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::focusLonLat(BDGeoCoord geoCoord)
{
    if (m_pMap)
    {
        m_pMap->setMapPosition(geoCoord, true);
    }
}

void BaiduMapAutoQmlInterface::onMapLevelChanged(const BDMapViewer &object, const BDUInt32 &level)
{

}

void BaiduMapAutoQmlInterface::onMapPositionChanged(const BDMapViewer &object, const BDGeoCoord &coord)
{

}

void BaiduMapAutoQmlInterface::onMapScreenShotFinished(const BDMapViewer &object)
{
    int height = 0;
    int width = 0;
    char** buf;
    m_pMap->getScreenShotResult(height, width, buf);
}

void BaiduMapAutoQmlInterface::onMapPoiClicked(const BDMapViewer &object, const BDPOIInfo &poi_info)
{
    m_pMap->addPoiMarker(poi_info.getPosition());
    focusLonLat(poi_info.getPosition());

    double gcj_lon = (double)poi_info.getPosition().getLongitude() / 100000;
    double gcj_lat = (double)poi_info.getPosition().getLatitude() / 100000;
    double distance = AMGeoTools::distanceFromHere(gcj_lon, gcj_lat);

    emit poiSelected(QString::fromStdString(poi_info.getName()), QString::fromStdString(poi_info.getAddress().getFullAddress()), distance);
}

void BaiduMapAutoQmlInterface::onMapBkgPoiMarkerClicked(const BDMapViewer &object, const BDUInt32 &index)
{
    // 仅适用于只有一个m_pSearch
    if (!m_pSearch)
    {
        return;
    }
    BDPOIInfo poi_info;
    if (m_pSearch->getResultItem(index, poi_info) == BDResult::OK)
    {
        m_pMap->addPoiMarker(poi_info.getPosition());
        focusLonLat(poi_info.getPosition());

        double gcj_lon = (double)poi_info.getPosition().getLongitude() / 100000;
        double gcj_lat = (double)poi_info.getPosition().getLatitude() / 100000;
        double distance = AMGeoTools::distanceFromHere(gcj_lon, gcj_lat);

        emit poiSelected(QString::fromStdString(poi_info.getName()), QString::fromStdString(poi_info.getAddress().getFullAddress()), distance);
    }
}

/**
*   发起检索
*
*		@param addr [in] 检索地址
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::doSearch(QString addr)
{
    mSearchAddr = addr;
    BDPOISearchFilter filter;
    filter.setSearchType(BDPOISearchType::POI);
    filter.setSortType(BDSortType::DISTANCE);

    BNACoordinate cur_pos;
    AMGeoTools::locate(cur_pos.iLongitude, cur_pos.iLatitude);
    BDGeoCoord  pos;
    pos.setLatitude(cur_pos.iLatitude * 100000);
    pos.setLongitude(cur_pos.iLongitude * 100000);
//    pos.setLatitude(3672808);
//    pos.setLongitude(11920123);
    filter.setBoundary(pos, 0);

    BDAddress address;
    address.setRegionCode(131);
    filter.setAddress(address);
    filter.setExactMatchMode(true);
    static std::string tmpstr;
    tmpstr = mSearchAddr.toStdString();
    filter.setKeyword(tmpstr);

    m_pSearch->setSearchFilter(filter);
    m_pSearch->search();
    emit goResultListByWord(mSearchAddr);
    mMapAutoStatus = BNAMapAutoStatusSearch;
}

void BaiduMapAutoQmlInterface::doSearchByRoute()
{
    unsigned int count = 20;
    unsigned int pagenum = 0;
    std::vector<BDPOIInfo> reList;
    bool lastpage = false;
    m_pSearch->searchByRoute(std::string("卫生间"), count, pagenum, reList, 50, lastpage);
}

void BaiduMapAutoQmlInterface::doSearchSug(QString addr)
{
    m_pSearch->searchSug(addr.toStdString());
}

 void BaiduMapAutoQmlInterface::onSearchSugResultUpdate(BDPOISearch* pSearch)
 {
     emit clearSearchSugResultlist();
     std::vector<BDPOISugInfo> sugResultList;
     pSearch->getSugResultItemList(sugResultList);
     std::vector<BDPOISugInfo>::iterator it = sugResultList.begin();
     int i = 0;
     for (; it != sugResultList.end(); it++)
     {
         i++;
         QString poi_name = QString::fromStdString(it->name);
         QString poi_addr = QString::fromStdString(it->address);
         emit setSearchSugResultItemData(poi_name, poi_addr, i);
     }
 }

/**
*     获取检索结果 CallBack
*
*		@param
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::onSearchResultUpdate(BDPOISearch* pSearch, const BDInt32& status, const BDInt32& count)
{

    BDInt32 poi_count = count;
    std::vector<BDPOIInfo> tmplist;
    pSearch->getResultItemList(0, count, tmplist);
    for (int ii = 0; ii < count; ii++)
    {
        BDPOIInfo tmppoi = tmplist[ii];
        std::string tmpname = tmppoi.getName();
    }
    if(poi_count == 0)
    {
        return;
    }
    if(poi_count > 10)
    {
        poi_count = 10;
    }
    double lat, lng;
    bool b_has_result = false;
    BDPOIInfo poi_info;
    for (int i = 0; i < poi_count; i++)
    {

        pSearch->getResultItem(i, poi_info);
        if (poi_info.getName().length() <= 0)
        {
            continue;
        }
        BDGeoCoord coor = poi_info.getPosition();
        std::vector<std::string> keyList;
        poi_info.getOptionKeyList(keyList);
       // printf("#############keyList size=%d\n",keyList.size());

        lng  = (double)(coor.getLongitude()) / (double)100000;
        lat  = (double)(coor.getLatitude()) / (double)100000;
        double distance = AMGeoTools::distanceFromHere(lng, lat);
        b_has_result = true;
        QString poi_name = QString::fromStdString(poi_info.getName());
        QString poi_addr = QString::fromStdString(poi_info.getAddress().getFullAddress());
        emit setSearchResultItemData(poi_name, poi_addr, distance, i + 1, lng, lat);
    }
    if (!b_has_result)
    {
        emit setSearchResultItemData("", "", 0, 0, 0, 0);
    }
}


/**
*   发起路径规划
*
*		@param longitude [in] longitude
*		@param latitude [in]  latitude
*
*		@return
*		@note	无
*/
void BaiduMapAutoQmlInterface::startRoutePlan(double longitude, double latitude)
{
    //for test route cruise; add by sfy; 2017-08-10
    m_pMap->disableFollowingCar();

    // 1st set startPosition
    BNACoordinate cur_pos;
    AMGeoTools::locate(cur_pos.iLongitude, cur_pos.iLatitude);
    BDGeoCoord start_geo_coord;
    start_geo_coord.setLatitude(cur_pos.iLatitude * 100000);
    start_geo_coord.setLongitude(cur_pos.iLongitude * 100000);
    BDRoutePosition start_pos;
    start_pos.setPosition(start_geo_coord);
    m_pRouteGenerator->setStartPosition(start_pos);

    // 2nd set destPosition
    BDGeoCoord dest_geo_coord;
    dest_geo_coord.setLatitude(latitude * 100000);
    dest_geo_coord.setLongitude(longitude * 100000);
    BDRoutePosition dest_pos;
    dest_pos.setPosition(dest_geo_coord);
    m_pRouteGenerator->setDestination(dest_pos);

    BDRouteOption routeOption;
    std::vector<std::string> optionList;
    routeOption.getOptionList(optionList);
    routeOption.setOption(optionList[2],0);
    routeOption.enableOnlineMode(true);
    m_pRouteGenerator->addRouteOption(routeOption);

    // 3rd generator
    m_pRouteGenerator->generate();
    mMapAutoStatus = BNAMapAutoStatusRouteplan;
    emit displayRouteplan();
}
/**
*   获取路径规划结果
*
*		@param none
*
*		@return
*		@note	emit display routeplan result view signal
*/
void BaiduMapAutoQmlInterface::onGenerateStatusUpdated(const BDRouteGenerator& generator, const BDRouteGeneratingStatus& status, const BDByte& numOfRoutes)
{
    printf("****pany   in onGenerateStatusUpdated\n");
    // 防止偏航时路径重新规划产生消息后View误画
    if (mMapAutoStatus != BNAMapAutoStatusRouteplan)
    {
        return;
    }
    int route_cnt = numOfRoutes > 3 ? 3: numOfRoutes;
    BDGeoArea geo_area;
    BDRouteInfo route_info;
    std::vector<baidu::mapauto::navi::route::BDRouteInfo> routeInfos;

    for (int i = 0; i < route_cnt; i++)
    {
        int index = i + 1;
        m_pRouteGenerator->getRouteInfo(i, route_info);
        routeInfos.push_back(route_info);
        emit setRoutePlanResultItemData(index, route_info.getTotalTime(), route_info.getTotalDistance(), route_info.getTrafficLight());
    }

    // 在mapView添加路线
//    BDUInt64 request_id = route_info.getId();
//    request_id /= 10;
    m_pMap->addRoutes(routeInfos);

    // 调整底图显示比例
    geo_area = route_info.getArea();
    baidu::mapauto::common::BDRectangle rect = {0, 0, DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT};
    m_pMap->fitGeoAreaToScreen(geo_area, rect);
}

/**
*   取消路径规划
*
*		@param none
*
*		@return
*		@note
*/
void  BaiduMapAutoQmlInterface::cancelRoutePlan()
{
    m_pRouteGenerator->cancel();
}

/**
*   开始导航
*
*		@param none
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::startGuide()
{
    int routeplan_level = m_pMap->getMapLevel();

//    BDResult result = m_pRouteGuide->startRouteGuidance();
    BDResult result = m_pRouteGuide->startSimulation();
//    m_pRouteGuide->setSimulationSpeed(60); //speed add 10; add by sfy; 2017-7-24

    // Close autoLevel
 //   BDResult result1 = m_pRouteGuide->setMapMemoryScale(routeplan_level);
   // BDResult result2 = m_pRouteGuide->setMapAutoLevelStatus(false);

//    m_pRouteGuide->startRouteGuidance();

    mMapAutoStatus = BNAMapAutoStatusGuideance;
}

/**
*   取消导航
*
*		@param none
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::stopNavigation()
{
    m_pRouteGuide->stopSimulation();
    setMapMode(mMapMode);
    mMapAutoStatus = BNAMapAutoStatusNomal;

}

// Guidance callback
void BaiduMapAutoQmlInterface::onRouteGuideStatusUpdated(const BDRouteGuideStatus &status)
{

}

void BaiduMapAutoQmlInterface::onRouteCruiseAvailableUpdated(bool& status)
{

}
void BaiduMapAutoQmlInterface::onGuideChanged()
{

}


/**
*   诱导turn info信息
*
*		@param turnInfoList
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::onTurnInfoUpdated(const std::vector<BDTurnInfo>& turnInfoList)
{
    BDTurnInfo turn_info = turnInfoList[0];

    // Next road name
    QString next_road_name = QString::fromStdString(turn_info.getNextRoadName());

    // Next road distance
    int neext_road_distance = turn_info.getRemainDistance();

    // Dreict icon
    QString guide_icon_file_name = "images/nsdk_drawable_rg_ic_" \
            + QString::fromStdString(turn_info.getIconFileName());

    // Remain time
    int route_index = 0;
    BDUInt32 remain_time;
    m_pRouteGuide->getRemainTime(route_index, remain_time);

    QTime time = QTime::currentTime().addSecs(remain_time);
    QString str_remin_time = time.toString();
    str_remin_time = str_remin_time.left(str_remin_time.length() - 3) + "到达";

    // Remain distance
    BDUInt32 remain_distance;
    m_pRouteGuide->getRemainDistance(route_index,remain_distance);
    emit setGuidanceData(guide_icon_file_name, remain_distance, \
                         str_remin_time,  neext_road_distance, next_road_name);

    //for test road condition; add by sfy;
    std::vector<BDRoadCondition> arrRoadCondition;
    float progess;
    BDResult res1 = m_pRouteGuide->getRoadCondition(arrRoadCondition);
    BDResult res2 = m_pRouteGuide->getCarProgress(progess);
    int type1 = arrRoadCondition[arrRoadCondition.size() - 1].getEnRoadCondition();
    res1 = BDResult::OK;
}

void BaiduMapAutoQmlInterface::onRouteDeviated()
{

}

void BaiduMapAutoQmlInterface::onRoadNameUpdated(const std::string& roadName)
{
     printf("roadName = %s", roadName.c_str());
}

void BaiduMapAutoQmlInterface::onHighwayInfoUpdated(const BDHighwayInfo& highwayInfo)
{
}

void BaiduMapAutoQmlInterface::onAddressUpdated(const baidu::mapauto::common::BDAddress& address)
{

}

void BaiduMapAutoQmlInterface::onRoadTypeUpdated(const BDRoadType& roadType)
{

}

void BaiduMapAutoQmlInterface::onSpeechUpdated(const int32_t& type, const std::string& speech)
{

}

void BaiduMapAutoQmlInterface::onSpeechUpdated(const int32_t& type, const BDByte* pPCMData, const BDUInt32& nDataSize)
{

}

void BaiduMapAutoQmlInterface::onWaypointArrived(const BDUInt32& pointIdx)
{

}

void BaiduMapAutoQmlInterface::onDestinationArrived()
{
    stopNavigation();
    emit destinationArrived();
}

void BaiduMapAutoQmlInterface::onDestParkAvailableUpdated(bool& status) {

}

void BaiduMapAutoQmlInterface::onAvoidRouteMsgUpdated(bool& status) {

}

void BaiduMapAutoQmlInterface::onRoadConditionUpdated(bool& status) {

}

void BaiduMapAutoQmlInterface::onChangeRouteUpdated(const BDRouteGuideChangeRouteType& status) {

}

void BaiduMapAutoQmlInterface::onLaneInfoUpdated(const BDLaneInfo& laneInfo)
{

}

void BaiduMapAutoQmlInterface::onRouteReGenerated(const BDRouteReGeneratingStatus& status)
{

}
void BaiduMapAutoQmlInterface::onRefreshRouteUpdated(const BDRefreshRouteStatusType& result)
{
    printf("onRefreshRouteUpdated resutl =%d\n", result);
}

void onVirtualMapViewUpdated(const BDVirtualMapViewInfo& vInfo){

}

void BaiduMapAutoQmlInterface::onILSImageViewUpdated(const BDILSImageViewInfo& hInfo) {
    //get buffer
    std::string pBGStr = hInfo.getBackGoundMap();
    std::string pArrowStr = hInfo.getArrowMap();
    const char* pBGMap = pBGStr.c_str();
    const char* pArrowMap = pArrowStr.c_str();
    unsigned int bgLength = 0;
    unsigned int arrowLength = 0;
    unsigned char* pbgByteBuf = NULL;
    unsigned char* parrowByteBuf = NULL;
    m_pRouteGuide->getRasterExpandMapImage(pBGMap, 1, &pbgByteBuf, bgLength);
    m_pRouteGuide->getRasterExpandMapImage(pArrowMap, 0, &parrowByteBuf, arrowLength);

    //for test file type
//    std::FILE *a = fopen("/home/l3/workspace/ENGINE/Gen5-HNavigation/image/1.jpg", "wb");
//    if(a == NULL)
//    {
//        return;
//    }
//    fwrite(pbgByteBuf, bgLength, 1, a);
//    std::FILE *b = fopen("/home/l3/workspace/ENGINE/Gen5-HNavigation/image/2.png", "wb");
//    fwrite(parrowByteBuf, arrowLength, 1, b);
    std::string fileName1 = "/home/l3/workspace/ENGINE/Gen5-HNavigation/image/1.jpg";
    std::string fileName2 = "/home/l3/workspace/ENGINE/Gen5-HNavigation/image/2.png";
    setRasterMapInfoData(fileName1);
}


void BaiduMapAutoQmlInterface::onHideILSImageViewUpdated(bool& status) {

}


/**
*   选择指定路线
*
*		@param index [in] 道路索引
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::selectRouteByID(int index)
{
    BDRouteInfo route_info;
    m_pRouteGenerator->getRouteInfo(index, route_info);

    m_pMap->highlightRoute(route_info.getId());
}


/**
*   清除路线
*
*		@param
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::clearRoutes()
{
    m_pMap->clearRoutes();
}

void BaiduMapAutoQmlInterface::onTestBtn1()
{
//        //百度大厦
//            int longitude = 11631326;
//            int latitude = 4004778;

//        //奎科大厦
//            int longitude = 11630679;
//            int latitude = 4004173;

//        //北京首都国际机场
//            int longitude = 11662187;
//            int latitude = 4006176;

//        //北京欢乐谷
//            int longitude = 11649510;
//            int latitude = 3986800;

//        //西湖
//            int longitude = 12013492;
//            int latitude = 3025154;

//        //嘉兴大剧院
//            int longitude = 12075974;
//            int latitude = 3073808;

//        //天安门
//            int longitude = 11639759;
//            int latitude = 3990877;

//        //西安北站
//            int longitude = 10893847;
//            int latitude = 3437615;

//        //亚龙湾站
//            int longitude = 10960228;
//            int latitude = 1830256;

//        //海淀公园
//            int longitude = 11629523;
//            int latitude = 3998697;

//        //五彩城
//            int longitude = 11633548;
//            int latitude = 4003021;

//        //北京体育大学
//            int longitude = 11631936;
//            int latitude = 4002343;

//        //香港大学
//            int longitude = 11414270;
//            int latitude = 2228011;

//        //香港旺角
//            int longitude = 11417428;
//            int latitude = 2231688;

//        //深圳百度大厦
//            int longitude = 11394236;
//            int latitude = 2252453;

//        //北京南站
//            int longitude = 11637870;
//            int latitude = 3986480;

//        //什刹海地铁站
//            int longitude = 11639618;
//            int latitude = 3993781;

//        //上地创业大厦
//            int longitude = 11631074;
//            int latitude = 4003662;

//    static bool b = false;
//    if (!b)
//    {
//        std::vector<common::BDGeoCoord> geoArr;

//        common::BDGeoCoord tmpCoord1;
//        tmpCoord1.setLongitude(11631326);
//        tmpCoord1.setLatitude(4004173);
//        geoArr.push_back(tmpCoord1);

//        common::BDGeoCoord tmpCoord2;
//        tmpCoord2.setLongitude(11633548);
//        tmpCoord2.setLatitude(4003021);
//        geoArr.push_back(tmpCoord2);

//        common::BDGeoCoord tmpCoord3;
//        tmpCoord3.setLongitude(11662187);
//        tmpCoord3.setLatitude(4006176);
//        geoArr.push_back(tmpCoord3);

//        common::BDGeoCoord tmpCoord4;
//        tmpCoord4.setLongitude(11639759);
//        tmpCoord4.setLatitude(3990877);
//        geoArr.push_back(tmpCoord4);

//        common::BDGeoCoord tmpCoord5;
//        tmpCoord5.setLongitude(11629523);
//        tmpCoord5.setLatitude(3998697);
//        geoArr.push_back(tmpCoord5);
//        m_pMap->setMapRegionStyle(0.0f, 0.6f, 0.1f, 0.3f);
//        m_pMap->setMapRegionData(geoArr);
//        b = true;
//    }
//    else
//    {
//        m_pMap->clearMapRegionData();
//        b = false;
//    }
//    doSearchByRoute();

    m_pMap->screenShot(300, 300);
}

void BaiduMapAutoQmlInterface::onTestBtn2()
{

}

/**
*   车标跟随
*
*		@param
*
*		@return
*		@note
*/
void BaiduMapAutoQmlInterface::followCar()
{
    m_pMap->setNavigationMapViewStatus(BDNaviMapViewStatus::NAVI_VIEW_STATUS_FLLOW_CAR_POSITION);
    BNASetNavigationMapViewRotateMode(BNA_GUIDE_VIEW_ROTATE_MODE_CAR, BNAScreenCenter);
    mMouseDragTimer->stop();
}

static BDFloat angle = 0;
void BaiduMapAutoQmlInterface::tryLuck()
{
    //Switch Map View
//    m_bShowMainMap = !m_bShowMainMap;

    ///*---------------------------------*///
//    angle += 5.0f;
//    if (angle >= 360.0f)
//    {
//        angle -= 360.0f;
//    }
//    m_pMap->setMapAngle(angle);
    ///*---------------------------------*///
    if (mMapAutoStatus == BNAMapAutoStatusNomal)
    {
        m_pMap->disableFollowingCar();
        m_pRouteGuide->startRouteCruise();
        BNASetNavigationMapViewRotateMode(BNA_GUIDE_VIEW_ROTATE_MODE_CAR, BNAScreenCenter);//正北朝上
//        BNASetNavigationMapViewRotateMode(BNA_GUIDE_VIEW_ROTATE_MODE_MAP, BNAScreenCenter);//车头朝上
        mMapAutoStatus = BNAMapAutoStatusCruise;
    }
    else
    {
        m_pMap->enableFollowingCar();
        m_pRouteGuide->stopRouteCruise();
        mMapAutoStatus = BNAMapAutoStatusNomal;
    }
}

static bool bShow = true;
void BaiduMapAutoQmlInterface::mapStatusTest()
{
//    BNAMapControlSetMapLayerShowStatus(BNA_MAP_CONTROL_LAYER_LOCATION, bShow, BNAScreenCenter);
//    bShow = !bShow;

//    if (bShow)
//    {
//        BNAOnMapPause(BNAScreenCenter);
//    }
//    else
//    {
//        BNAOnMapResume(BNAScreenCenter);
//    }
//    bShow = !bShow;

//    if (!bShow)
//    {
//        m_pMap->setMapTheme(BDMapTheme::DAY_MODE1);
//    }
//    else
//    {
//        m_pMap->setMapTheme(BDMapTheme::NIGHT_MODE1);
//    }
//    bShow = !bShow;

    followCar();
}

NAMESPACE_END
