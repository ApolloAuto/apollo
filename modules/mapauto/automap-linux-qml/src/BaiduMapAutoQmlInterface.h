#ifndef BAIDUMAPAUTOQMLINTERFACE_H
#define BAIDUMAPAUTOQMLINTERFACE_H
#include <QQuickItem>
#include <QTimer>
#include <QQuickWindow>
#include <QBuffer>

//#include "BNAInterface/navi_auto_adapter_if.h"
//#include "BNATools/navi_car_adapter_def.h"
//#include "BNATools/navi_car_base.h"

#include "utils/def.h"
#include "utils/amsettingmanager.h"

#include "BDMapViewer.h"
#include "BDPOISearch.h"
#include "BDCommonTypes.h"
#include "BDNaviTypes.h"
#include "BDRouteGenerator.h"
#include "BDRouteGuide.h"
#include <string>
#include "BDCommonTypes.h"
#include "BDGlobalInit.h"

NAMESPACE_START
#define MULTI_SCREEN        // 使用多屏模式时打开此宏
#define AM_MAP_MODE_NORTH  0  // 指北模式 dss
#define AM_MAP_MODE_DIRECT  1 // 2.5D模式
#define AM_MAP_MODE_NEED_FIX 2 // 地图移动过
using namespace baidu::mapauto;
using namespace  baidu::mapauto::navi;
using namespace  baidu::mapauto::navi::map;
using namespace  baidu::mapauto::common;
using namespace  baidu::mapauto::navi::search;
using namespace  baidu::mapauto::navi::route;
using namespace baidu::mapauto::navi::guide;

typedef enum _MAPAUTO_STATUS
{
    BNAMapAutoStatusNomal = 0,
    BNAMapAutoStatusSearch,
    BNAMapAutoStatusRouteplan,
    BNAMapAutoStatusCruise,
    BNAMapAutoStatusGuideance
}MAPAUTO_STATUS;

class BaiduMapAutoQmlInterface :
        public QQuickItem,
        public IBDMapViewerListener,
        public IBDPOISearchListener,
        public IBDRouteGeneratorListener,
        public IBDRouteGuideBasicListener,
        public IBDRouteGuideViewListener
{
    Q_OBJECT

public:
    explicit  BaiduMapAutoQmlInterface(void);

    /**
    *	设置路况开关
    *
    *		@param value		[in] 开关值
    *
    *		@return 无
    *		@note	无
    */
    Q_INVOKABLE void setTrafficSwitch(bool value);

    /**
    *	获取路况开关值
    *
    *		@param 无
    *
    *		@return 开关值
    *		@note	无
    */
    Q_INVOKABLE bool getTrafficSwitch(void);

    /**
    *	设置视角模式
    *
    *		@param mode [in] 模式值
    *
    *		@return 无
    *		@note	这里的Mode value使用的是宏，而Widget中使用的是Enum
    */
    Q_INVOKABLE void setMapMode(int mode);

    /**
    *	获取视角模式值
    *
    *		@param 无
    *
    *		@return 模式值
    *		@note	无
    */
    Q_INVOKABLE int getMapMode(void);

    /**
    *	设置默认位置
    *
    *		@param 无
    *
    *		@return 无
    *		@note	无
    */
    Q_INVOKABLE void setDefaultMapLocation(void);

    /**
    *	放大显示比例
    *
    *		@param animated [in] 是否需要动画:true or false
    *		@param interval [in] 动画间隔时间:S
    *
    *		@return true or false
    *		@note	无
    */
    Q_INVOKABLE int zoomIn(bool animated, unsigned int interval);

    /**
    *	缩小显示比例
    *
    *		@param animated [in] 是否需要动画:true or false
    *		@param interval [in] 动画间隔时间:S
    *
    *		@return true or false
    *		@note	无
    */
    Q_INVOKABLE int zoomOut(bool animated, unsigned int interval);

    Q_INVOKABLE void zoomInHeightPrecision(float interval);
    Q_INVOKABLE void zoomOutHeightPrecision(float interval);
    Q_INVOKABLE void refreshRoute();
    /**
    *	图区移动处理
    *
    *		@param mouse_x [in] x
    *		@param mouse_y [in] y
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void mouseMove(int last_x, int last_y, int mouse_x, int mouse_y);

    Q_INVOKABLE void focusPoi(int screen_x, int screen_y);
    Q_INVOKABLE void focusRoute(int screen_x, int screen_y);


    /**
    *   反Geo
    *
    *		@param mouse_x [in] x
    *		@param mouse_y [in] y
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void focusAntiGeo(int screen_x, int screen_y);

    /**
    *   Poi列表图区展示
    *
    *		@param index [in] 选中POI索引
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void focusPoiList(int index);

    /**
    *   发起检索
    *
    *		@param 发起检索 [in] addr
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void doSearch(QString addr);

    /**
    *   发起检索
    *
    *		@param 发起检索 [in] addr
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void doSearchByRoute();

    /**
    *   发起检索sug
    *
    *		@param 发起检索 [in] addr
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void doSearchSug(QString addr);

    /**
    *   根据坐标定位中心点
    *
    *		@param longitude [in] longitude
    *		@param latitude [in]  latitude
    *
    *		@return
    *		@note	无
    */
    void focusLonLat(BDGeoCoord geoCoord);

    /**
    *   发起路径规划
    *
    *		@param longitude [in] longitude
    *		@param latitude [in]  latitude
    *
    *		@return
    *		@note	无
    */
    Q_INVOKABLE void startRoutePlan(double longitude, double latitude);

    /**
    *   开始导航
    *
    *		@param none
    *
    *		@return
    *		@note
    */
    Q_INVOKABLE void startGuide();

    /**
    *   取消路径规划
    *
    *		@param none
    *
    *		@return
    *		@note
    */
    Q_INVOKABLE void cancelRoutePlan();

    /**
    *   取消导航
    *
    *		@param none
    *
    *		@return
    *		@note
    */
    Q_INVOKABLE void stopNavigation();
    /**
    *   选择指定路线
    *
    *		@param index [in] 道路索引
    *
    *		@return
    *		@note
    */

    Q_INVOKABLE void selectRouteByID(int index);

    /**
    *   清除路线
    *
    *		@param none
    *
    *		@return
    *		@note
    */
    Q_INVOKABLE void clearRoutes();

    /**
    *   清除路线
    *
    *		@param none
    *
    *		@return
    *		@note
    */
    Q_INVOKABLE void onTestBtn1();

    Q_INVOKABLE void onTestBtn2();

    Q_INVOKABLE void tryLuck();

    Q_INVOKABLE void mapStatusTest();

    Q_INVOKABLE void exitApplication();

signals:
    void poiSelected(QString name, QString address, double distance);  // 选中POI
    void noneSelected(); // 取消选中POI
    void startRenderTimer();
    void goResultListByWord(const QString addr);
    void setSearchResultItemData(QString name, QString address, double distance, int index, double longitude, double latitude);
    void setSearchSugResultItemData(QString name, QString address, int index);
    void clearSearchSugResultlist();
    void displayRouteplan();
    void setRoutePlanResultItemData(int index, int time, int length, int traffic_signal_count);
    void setGuidanceData(QString guide_image_url, int remain_distance, QString remain_time,  int next_road_distance, QString next_road_name);
    void setRasterMapInfoData(std::string fileName);
    void destinationArrived();
public slots:
    void render(void);
    void followCar();

private:
    void initBaiduMapEngine();

    // Map Viewer callback
    virtual void onMapPositionChanged(const BDMapViewer& object, const common::BDGeoCoord& coord);
    virtual void onMapLevelChanged(const BDMapViewer& object, const BDUInt32& level);
    virtual void onMapPoiClicked(const BDMapViewer& object, const search::BDPOIInfo& info);
    virtual void onMapBkgPoiMarkerClicked(const BDMapViewer& object, const BDUInt32& index);
    virtual void onMapScreenShotFinished(const BDMapViewer &object);

    // Search Listener callback
    virtual void onSearchResultUpdate(BDPOISearch* pSearch, const BDInt32& status, const BDInt32& count);

    virtual void onSearchSugResultUpdate(BDPOISearch* pSearch);

    // Gernerator listener callback
    virtual void onGenerateStatusUpdated(const BDRouteGenerator& generator, const BDRouteGeneratingStatus& status, const BDByte& numOfRoutes = 0);

    // Guide base BasicListener
    virtual void onRouteGuideStatusUpdated(const BDRouteGuideStatus& status);
    virtual void onRouteCruiseAvailableUpdated(bool& status);
    virtual void onGuideChanged();
    virtual void onTurnInfoUpdated(const std::vector<BDTurnInfo>& turnInfoList);
    virtual void onRouteDeviated();
    virtual void onRoadNameUpdated(const std::string& roadName);
    virtual void onHighwayInfoUpdated(const BDHighwayInfo& highwayInfo);
    virtual void onAddressUpdated(const baidu::mapauto::common::BDAddress& address);
    virtual void onRoadTypeUpdated(const BDRoadType& roadType);
    virtual void onSpeechUpdated(const int32_t& type, const std::string& speech);
    virtual void onSpeechUpdated(const int32_t& type, const BDByte* pPCMData, const BDUInt32& nDataSize);
    virtual void onWaypointArrived(const BDUInt32& pointIdx);
    virtual void onDestinationArrived();
    virtual void onDestParkAvailableUpdated(bool& status);
    virtual void onAvoidRouteMsgUpdated(bool& status);
    virtual void onRoadConditionUpdated(bool& status);
    virtual void onChangeRouteUpdated(const BDRouteGuideChangeRouteType& status);
    virtual void onLaneInfoUpdated(const BDLaneInfo& laneInfo) ;
    virtual void onRouteReGenerated(const BDRouteReGeneratingStatus& status);
    virtual void onRefreshRouteUpdated(const BDRefreshRouteStatusType& result);
    //Guide viewListener
    virtual void onILSImageViewUpdated(const BDILSImageViewInfo& hInfo);
    virtual void onHideILSImageViewUpdated(bool& status);

protected slots:
    void OnWindowChanged( QQuickWindow* pWindow );
    void Release( void );

private:
    QTimer *mDrawTimer;   // Engine draw timer
    QTimer *mMouseDragTimer;   // Mouse drag timer

    bool mTrafficSwitchValue;
    int mMapMode;
    int mLastTouchX;
    int mLastTouchY;
    AMPoiInfo mPoiInfo;
    QString mSearchAddr;
    MAPAUTO_STATUS mMapAutoStatus;

#ifdef MULTI_SCREEN
    BNAScreenType mScreenType;
#endif

    // pGen5
   BDMapViewer* m_pMap;
   BDPOISearch* m_pSearch;
   BDRouteGenerator* m_pRouteGenerator;
   BDRouteGuide* m_pRouteGuide;

   BDMapViewer* m_pSubMap;
   bool m_bShowMainMap;
};
NAMESPACE_END
#endif // BAIDUMAPAUTOQMLINTERFACE_H
