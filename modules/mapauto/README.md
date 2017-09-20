[TOC]

# 开放平台Demo使用说明

##一、资源文件配置

1.将/opt/目录设置为可读写权限 chmod u+rw opt/
2.将bnav中的资源文件放到/opt/BaiduMapAuto/bnav/目录下（此目录可配置，请修改BaiduMapAutoQmlInterface.cpp中CFG_DATA_CONFIG变量）离线数据更新请按照bnav/ReadMe.txt文档更新

##二、Linux环境安装依赖库
1.安装编译工具集
安装命令：

    sudo apt-get install build-essential

安装完成后验证gcc／g++版本
PS: if meet ” E: Unable to locate package essential” ,please update apt-get
sudo apt-get update

2.安装OpenGL工具集
安装opengl 3命令：

    sudo apt-get install freeglut3-dev

安装opengl es命令：

    sudo apt-get install libgles2-mesa-dev

3.安装freetype
更新索引列表 

    apt-get update
    sudo apt-get install libfreetype6-dev

4.curl库

    将 jumbo_lib 解压到  $(HOME)/.jumbo/lib下

##三、Demo工程配置

建立so软链接：进入 automap-linux-qml/libs/ 目录 ，执行 sh  ln_sh.sh

#功能模块使用说明

 - 开发环境：Ubuntu14.04 + Qt 5.7.0，使用qml开发UI, 工程名字automap-linux-qml.pro
 - ubuntu 环境搭建 http://agroup.baidu.com/automap_linux/md/article/152991
 - 引擎使用流程简介：（请结合BaiduMapAutoQmlInterface.cpp）    按功能模块划分为mapView、search、routeGenerator、routeGuide；   
    每个模块都要创建新实例，调用相应的API接口，API详见doc;
    
##mapView模块
 

> mapView实例创建一定放在在SurfaceView创建好之后进行

**1、初始化底图**
 

    BDMapViewer* m_pMap;
     m_pMap = new BDMapViewer(BDMapType::FRONT,  DEV_WINDOW_WIDTH, DEV_WINDOW_HEIGHT);
    Parameters
    [in]	type	map type such as front, rear left and so on
    [in]	width	Sub-Surface's width value
    [in]	height	Sub-Surface's height value
    //设置地图显示level
    m_pMap->setMapLevel(14);
    //设置地图中心点偏移：比如导航过程中设置车标偏下显示或者显示路口放大图的时候，设置导航路线偏右显示
    m_pMap->setMapOffset(0，-100);
    //设置日夜景模式
    m_pMap->setMapTheme(BDMapTheme::NIGHT_MODE1);
	
**2、在绘制线程以40ms左右周期调用** mapView->draw()

    m_pMap.draw();
	 
**3、监听消息**
添加监听

    m_pMap->setEventListener(this);
底图等级变化

    void MyBDMapViewerListener::onMapPositionChanged(const BDMapViewer& object, const common::BDGeoCoord& coord) {
        ...
    }
    Parameters
    [in]	pos	The geo-position of the center of BDMapViewer instance

**4、多边形绘制**
    
示例代码

	//多边形定点定义，坐标为国测局经纬度坐标，并乘以100000，顺时针方向绘制定点
	std::vector<common::BDGeoCoord> geoArr;

        common::BDGeoCoord tmpCoord1;
        tmpCoord1.setLongitude(11631326);
        tmpCoord1.setLatitude(4004173);
        geoArr.push_back(tmpCoord1);

        common::BDGeoCoord tmpCoord2;
        tmpCoord2.setLongitude(11633548);
        tmpCoord2.setLatitude(4003021);
        geoArr.push_back(tmpCoord2);

        common::BDGeoCoord tmpCoord3;
        tmpCoord3.setLongitude(11662187);
        tmpCoord3.setLatitude(4006176);
        geoArr.push_back(tmpCoord3);

        common::BDGeoCoord tmpCoord4;
        tmpCoord4.setLongitude(11639759);
        tmpCoord4.setLatitude(3990877);
        geoArr.push_back(tmpCoord4);

        common::BDGeoCoord tmpCoord5;
        tmpCoord5.setLongitude(11629523);
        tmpCoord5.setLatitude(3998697);
        geoArr.push_back(tmpCoord5);
    //样式定义，rgba，值在0~1之间
    m_pMap->setMapRegionStyle(0.0f, 0.6f, 0.1f, 0.3f);
    //设置数据并开始绘制
    m_pMap->setMapRegionData(geoArr);
	//清除多边形绘制
    m_pMap->clearMapRegionData();
    
底图POI点击（POI详情数据需要进一步修改 ）

**5、自定义底图标注**

示例代码——添加标注点

    std::vector<HDynamicMapPoint> array;
    HDynamicMapPoint point0, point1, point2;
    //大厦
    point0.name = "百度大厦";
    point0.point.setLongitude(11630218);
    point0.point.setLatitude(4005007);
    point0.tag = "TYPE_1";
    array.push_back(point0);
    //奎科
    point1.name = "奎科科技大厦";
    point1.point.setLongitude(11630691);
    point1.point.setLatitude(4004174);
    point1.tag = "TYPE_1";
    array.push_back(point1);
    //科技园
    point2.name = "百度科技园";
    point2.point.setLongitude(11627512);
    point2.point.setLatitude(4004396);
    point2.tag = "TYPE_2";
    array.push_back(point2);

    m_pMap->setDynamicMapPoints(array);

示例代码——设置Marker Icon

    QImage* img = new QImage(16, 16, QImage::Format_RGBA8888);
    int iTWidth = img->width();
    int iTHeight = img->height();
    int iChannel = 4;
    img->fill(QColor(255, 0, 0, 200));
    m_pMap->setDynamicMapImages("TYPE_1", (HChar *)img->bits(), iTWidth, iTHeight, iChannel);
    delete img;

注：引擎会对图片buffer进行深拷贝，因此上层内存需自己管理

##search模块

> 检索区分区域检索和周边检索，通过设置不同的search filter来区别
> 检索模块支持sug检索

1、初始化Search模块

    BDPOISearch* m_pSearch;
    m_pSearch = new BDPOISearch();
2、设置检索filter（检索相关参数属性，sug检索无需设置filter）
a、初始化Filter

    BDPOISearchFilter filter;
b、filter设置区域检索或者周边检索，两种检索为互斥的，即filter.setAddress(address)和filter.setBoundary(pos, 1000)同时只使用一个。
**区域检索：**
	   

    BDAddress address;
    address.setRegionCode(131);//设置区域id为北京
    filter.setAddress(address);
**周边检索：**
要求半径大于0，单位米，代码示例：

     BDGeoCoord  pos;
	 pos.setLatitude(3672808);
	 pos.setLongitude(11920123);
	 filter.setBoundary(pos, 1000);
c、翻页功能




	 filter.setPageOption(BDUInt32 curPage, BDUInt32 pageCount)； // curPage:当前页序号，  pageCount：每页的检索结果数量
	 BDPOISearch::isLastPage();  // 判断是否为最后一页
3、设置检索的关键字：filter设置setKeyword

	filter.setKeyword("天安门");
4、search中设置filter

    m_pSearch->setSearchFilter(filter);
    
5、发起检索/sug检索

    m_pSearch->search();
    m_pSearch->searchSug("肯");//检索sug无需设置filter
    

6、 设置监听获取检索结果回调
重载IBDPOISearchListener，并实现该类中的虚函数方法。代码示例：

    class BaiduMapAutoInterface ：public IBDPOISearchListener
    {
    	m_pSearch->setEventListener(this);
    	//获取检索的结果
    	void onSearchResultUpdate(BDPOISearch* pSearch, const BDInt32& status, const BDInt32& count)
    	{//逻辑代码
    	}
    	//获取sug（输入联想）的结果
	    void onSearchSugResultUpdate(BDPOISearch* pSearch)
    	{//逻辑代码
    	}
    }
7、 跨城市检索结果说明
      跨城市检索结果会返回目的地所在的城市名字，拿到返回结果的RegionCode，重新发起区域检索。 这种情景的标识为：BDPOIInfo::getUid()为空。
      例如：检索“上海 KFC”
      结果返回，如果info.getAddress().getRegion()[0]为""，说明返回的是城市列表，列表包括上海和上海的regioncode，再使用regioncode重新发起区域检索。

8、POI详情检索结果
检索示例  http://dev.mapauto.baidu.com/detail?qt=infauto&uid=784961807213499085 

9、检索结果地图气泡展示

 1）根据以上步骤发起检索，等待检索结果通知
 2）获取检索结果列表
 

    searchApi->getResultItemCount(count);
    int offset = 0;
    std::vector<BDPOIInfo> resultList;
    searchApi->getResultItemList(offset, count, resultList);

 3）将检索结果的坐标存入vector
 

    std::vector<BDGeoCoord> coords;
    for(int i = 0; i < resultList.size(); ++i)
    {
        coords.push_back(resultList[i].getPosition());
    }

 4）调用底图接口添加气泡，检索结果为红色气泡，红色气泡标有poi的index
 

     m_pMap->addBkgPoiMarker(coords);
 
 10、屏幕上点击poi
 
 1）捕获屏幕的点击事件，获取屏幕坐标，调用mapview接口getSelectItemInfo得到poiInfo

        QPoint curPoint = event->pos();
        BDPOIInfo info;
        m_pMap->getSelectItemInfo(curPoint.rx(), curPoint.ry(), info);
 2)实现两个消息回调，在回调函数中调用选择poi的接口addPoiMarker(common::BDGeoCoord& coord);
 在调用getSelectItemInfo时会调用相应的回调函数，选中后，底图上该点会弹出蓝色气泡
 

 - onMapPoiClicked为底图上的poi点
 
 - onMapBkgPoiMarkerClicked为检索结果的红色气泡的poi点

注：若点击底图空白区域则获取不到poi信息，因此没有气泡显示 

        void MapViewerListener::onMapPoiClicked(const BDMapViewer &object, const BDPOIInfo &info)
       {
             map->addPoiMarker(info.getPosition());
       }
       void MapViewerListener::onMapBkgPoiMarkerClicked(const BDMapViewer &object, const BDUInt32 &index)
       {
             map->addPoiMarker(resultList[index].getPosition());
       }
11、隐藏气泡

    m_pMap->hideBkgPoiMarker();   //隐藏红色气泡
    m_pMap->hidePoiMarker();      //隐藏蓝色气泡
12、百度区域查询接口说明

	BDDistrictInfo districtInfo;
	m_pSearch->getTopDistrict(districtInfo);   //获取全国行政区信息
	std::vector<BDDistrictInfo> childDistrictInfo;
	m_pSearch->getChildDistrictInfo(districtInfo, childDistrictInfo);//获取districtInfo子行政区信息，即所有省级信息
    BDDistrictInfo districtInfo;
    m_pSearch->getDistrictInfoById(1, districtInfo);//获取某个id对应的政区信息 
    
  
##routeGenerator模块
1、初始化 

    BDRouteGenerator* m_pRouteGenerator;
    m_pRouteGenerator = new BDRouteGenerator();
2、设置起点，终点，代码示例

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

 3、设置检索参数，通过routeOption来设置
 
    BDRouteOption routeOption;
    std::vector<std::string> optionList;
    //获取可设置的参数list 
    routeOption.getOptionList(optionList);
    //设置为高速优先
    routeOption.setOption("BNA_ROUTEPLAN_PREFERENCE_TYPE_HIGHWAY",1);
    //设置为在线检索模块
    routeOption.enableOnlineMode(true);
    //设置车牌号，本地化限行
    routeOption.setOption("BNA_ROUTEPLAN_PREFERENCE_TYPE_CARNUM",1);
    BDCarInfo carinfo("P261U6", "津", BDCarTypeEnum::BD_CAR_TYPE_14L);
    m_pRouteGenerator->setLocalCarInfo(carinfo);
    //将设置option给Generator
    m_pRouteGenerator->addRouteOption(routeOption);
   算路偏好使用说明：
 -BNA_ROUTEPLAN_PREFERENCE_TYPE_RECOMMEND           推荐路线
 - BNA_ROUTEPLAN_PREFERENCE_TYPE_HIGHWAY                  高速优先
 - BNA_ROUTEPLAN_PREFERENCE_TYPE_NO_TOLL                  少收费
 - BNA_ROUTEPLAN_PREFERENCE_TYPE_AVOID_TRAFFICJAM  躲避拥堵
 - BNA_ROUTEPLAN_PREFERENCE_TYPE_CARNUM                    车牌限行
    eg:  以下设置是高速优先和躲避拥堵同时生效
		   routeOption.setOption("BNA_ROUTEPLAN_PREFERENCE_TYPE_HIGHWAY", 1); // 高速优先， 第二个参数为1时生效，为0时不生效
		   routeOption.setOption("BNA_ROUTEPLAN_PREFERENCE_TYPE_AVOID_TRAFFICJAM", 1)；
	 注意：*高速优先和少收费是互斥关系；车牌限行和其它项同时使用；推荐路线只能和车牌限行同时使用；躲避拥堵除不能和推荐一起使用外，可以和其它项同时使用*   

4、设置途经点
       
         BDRoutePosition  wpt_pos;
		 BDGeoCoord       geo_coord;
		 geo_coord.setLatitude( lat );
         geo_coord.setLongitude( lon );
		 wpt_pos.setPosition( geo_coord);
         m_pRouteGenerator->addWaypoint(wpt_pos);
   下次算路前要清除途经点：

         m_pRouteGenerator->clearWaypoints();
   
 5、发起算路
 
    m_pRouteGenerator->generate();

 6、算路结果监听并绘制在底图上
 重载IBDRouteGeneratorListener，并实现该类中的虚函数方法`onGenerateStatusUpdated（）`。代码示例：

    class BaiduMapAutoInterface ：public IBDRouteGeneratorListener
    {
    m_pRouteGenerator->setEventListener(this);
    // 获取路径规划结果
    void BaiduMapAutoInterface::onGenerateStatusUpdated(const BDRouteGenerator& generator, const BDRouteGeneratingStatus& status, const BDByte& numOfRoutes)
    {
       //获取返回的路线数量
       int route_cnt = numOfRoutes > 3 ? 3: numOfRoutes;
        BDGeoArea geo_area;
        BDRouteInfo route_info;
        //获取道路路线数组给mapview显示
       std::vector<baidu::mapauto::navi::route::BDRouteInfo> routeInfos;
       for (int i = 0; i < route_cnt; i++)
        {
            int index = i + 1;
            m_pRouteGenerator->getRouteInfo(i, route_info);
            routeInfos.push_back(route_info);
            emit setRoutePlanResultItemData(index, route_info.getTotalTime(),
            route_info.getTotalDistance(), route_info.getTrafficLight());
        }
    
       // 在mapView添加路线
       m_pMap->addRoutes(routeInfos);
    
       // 调整底图显示比例
       geo_area = route_info.getArea();
       baidu::mapauto::common::BDRectangle rect = {0, 0, 800, 600};
       m_pMap->fitGeoAreaToScreen(geo_area, rect);
      }
    }

##routeGuide模块
1、初始化

    BDRouteGuide* m_pRouteGuide
    m_pRouteGuide = BDRouteGuide::getInstance();
    m_pRouteGuide->setBasicEventListener(this);
    m_pRouteGuide->setViewEventListener(this);

2、开启导航或开启模拟导航
（1）开启导航：

    BDResult result = m_pRouteGuide->startRouteGuidance();

（2）开启模拟：

    BDResult result = m_pRouteGuide->startSimulation();

3、模拟导航设置速度：

    float speed = 10;                        //>0,speed+; <0, speed-
    m_pRouteGuide->setSimulationSpeed(10); 

4、TBT消息处理

    void onTurnInfoUpdated(const std::vector<BDTurnInfo>& turnInfoList)
    {
        BDTurnInfo turn_info = turnInfoList[0];

    // Next road name
    QString next_road_name = QString::fromStdString(turn_info.getNextRoadName());

    // Next road distance
    int neext_road_distance = turn_info.getRemainDistance();

    // Remain time
    int route_index = 0;
    BDUInt32 remain_time;
    m_pRouteGuide->getRemainTime(route_index, remain_time);

    // Remain distance
    BDUInt32 remain_distance;
    m_pRouteGuide->getRemainDistance(route_index,remain_distance);
    }
5、路口放大图
（1）消息通知后，更新过程

    void onILSImageViewUpdated(const BDILSImageViewInfo& hInfo) {
        //get buffer
        const char* pBGMap = hInfo.getBackGoundMap().c_str();
        const char* pArrowMap = hInfo.getArrowMap().c_str();
        unsigned int bgLength = 0;
        unsigned int arrowLength = 0;
        unsigned char* pbgByteBuf = NULL;
        unsigned char* parrowByteBuf = NULL;
        m_pRouteGuide->getRasterExpandMapImage(pBGMap, 1, &pbgByteBuf, bgLength);
        m_pRouteGuide->getRasterExpandMapImage(pArrowMap, 0, &parrowByteBuf, arrowLength);
        //load file
        ...
        }
    （

2）关闭路口放大图：收到下面的消息并收起路口放大图

    void onHideILSImageViewUpdated(bool& status) {
    }

6、智能比例尺

    // Close autoLevel
    BDResult result1 = m_pRouteGuide->setMapMemoryScale(BNAScreenCenter, level);
    BDResult result2 = m_pRouteGuide->setMapAutoLevelStatus(BNAScreenCenter, false);

7、路况数据：绘制光柱图

    std::vector<BDRoadCondition> arrRoadCondition;
    float progess;
    BDResult res1 = m_pRouteGuide->getRoadCondition(arrRoadCondition);
    BDResult res2 = m_pRouteGuide->getCarProgress(progess);

8、停车场处理逻辑
（1）确认收到消息

    void onDestParkAvailableUpdated(bool& status) {}

（2）发起停车场信息检索
（3）检索结果返回后，显示在地图上，并拼接出停车场推荐的文本，进行语音播报
（4）当用户点击“停车场”并确定“停在这里”时，获得当前点选停车场的位置
（5）重新设定起点为当前点位置：通过GetMapMatchInfo拿到
（6）设置终点为停车场位置
（7）发起算路
（8）调用switchRoute切换到算路结果中的第一条数据

9、全览态功能
（1）进入全览态：

    m_pRouteGuide->enterViewAll(SIMPLE_LAND);
    /**< SIMPLE_LAND参考BDRouteGuideViewAllType */

（2）退出全览态：

    m_pRouteGuide->exitViewAll();

10、巡航模式
（1）未开启导航模式下，地理位置连续变化数达到一定阈值并且速度达到一定阈值，会抛出可以开启消息

    onRouteCruiseAvailableUpdated();

（2）根据需要可以调用接口，开启巡航

    startRouteCruise();

（3）在一定条件（如用户操作底图）下，可以调用接口关闭巡航

    stopRouteCruise();

11、主辅路切换功能：
（1）主辅路的切换及消隐都是通过消息控制

    onChangeRouteUpdated(BDRouteGuideChangeRouteType resChangeRoute);

其中，BDRouteGuideChangeRouteType各定义如下：

    CHANGETO_MAINROUTE,                      //切换到主路
    CHANGETO_SALVEROUTE,                    //切换到辅路
    CHANGETO_PARALLELROUTE,              //切换到平行路线
    HIDECHANGEROUTE,                            //隐藏切换信息

（2）调用接口完成道路切换

    m_pRouteGuide->onlineChangeRoute();
12、获取当前路名(底图非导航模式使用，导航模式推荐onRoadNameUpdated())：

	m_pRouteGuide->getCurRoadNameByPos(longitude, latitude);
	

其中longitude、latitude格式为gcj02 * 100000

13、在导航中路线刷新

	m_pRouteGuide->refreshRoute();
刷新完成后在onRefreshRouteUpdated回调方法中返回刷新结果，结果类型说明：

	typedef enum _BDRefreshRouteStatusType {
    REFRESH_ROUTE_INVALID = -1,   /**< invalid*/
    REFRESH_ROUTE_SUCCESS,        /**< refresh success*/
    REFRESH_ROUTE_FAILED,         /**< refresh failed*/
    REFRESH_ROUTE_NEWROUTE,       /**< update new Route*/
    REFRESH_ROUTE_NO_NEWROUTE,    /**< no Route*/
    REFRESH_ROUTE_NET_TIMEOUT,    /**< timeout*/
    REFRESH_ROUTE_OTHER_ROUTE    /**< other route*/
	}BDRefreshRouteStatusType;





##Location模块

> Location模块功能包括两部分：
> 1. 支持LocationManager的状态操作，如开启、暂停、恢复、停止，以及更换LocationDriver进行模拟导航等。
> 2. 支持同步和异步获取Location相关信息。

1、初始化和销毁Location模块：

    // 初始化
    BDLocation* m_pLocation;
    m_pLocation = BDLocation::getInstance();
    m_pLocation->setEventListener(listener);
    // 销毁
    m_pLocation->unsetEventListener();
    
2、操作LocationManager的运行状态：

    m_pLocation->start(); // 开启Location线程
    m_pLocation->stop(); // 结束Location线程
    m_pLocation->pause(); // 暂停Location线程
    m_pLocation->resume(); // 恢复Location线程
    // 切换成NMEA和NaviTrack模拟器进行模拟导航。(interval是采样时间，单位是us；speed单位是m/s)
    m_pLocation->setNmeaSimulator(trackFile， interval, speed));
    m_pLocation->setNaviTrackSimulator(trackFile, interval, speed));
    // 切换成算路结果进行模拟导航，暂未实现
    m_pLocation->setRouteSimulator(); // 暂未实现
    // 切换会正常导航模式（目前支持passive driver，如果不同平台，需要单独配置，如ccos driver）
    m_pLocation->unsetSimulator();  // 目前只支持passive driver

3、同步获取Location相关信息：

    // 同步获取Location结构
    BDLocationInfo LocationInfo；
    BDResult ret = m_pLocation->getLocationInfo(LocationInfo);
    if (ret == BDResult::ERROR) {
	    // bdlog_e 记录出错信息
	}

4、异步获取Location相关信息：

    // 异步获取Location结构
    class LocationApi : public IBDLocationListener {
	    BDLocationInfo m_clLocationInfo；
	    
	    void onLocationInfoUpdated(const BDLocationInfo& LocationInfo) {
		    // 周期触发
		    // bdlog_v 记录LocationInfo输出数据
		    doSomeThingWithLocationInfo(LocationInfo);
		}
    }
	m_pLocationApi = new LocationApi();
	m_pLocation->setEventListener(m_pLocationApi);
	// onLocationInfoUpdated triggered periodly
    
##MapMatching模块

> MapMatching模块实现了巡航态、导航态下MapMatching数据输出的功能，以及惯导数据输出的功能。
> 目前并未实现惯导功能，只提供了惯导数据输出接口。

1、初始化和销毁MapMatching模块：

    // 初始化
    BDMapMatching* m_pMapMatching;
    m_pMapMatching = BDMapMatching::getInstance();
    m_pMapMatching->setEventListener(listener);
    // 销毁
    m_pMapMatching->unsetEventListener();
    
2、同步获取MapMatching相关信息：

    // 同步获取MapMatching结构
    BDMapMatchingInfo MapMatchingInfo；
    BDResult ret = m_pMapMatching->getMapMatchingInfo(MapMatchingInfo);
    if (ret == BDResult::ERROR) {
	    // bdlog_e 记录出错信息
	}

4、异步获取MapMatching相关信息：

    // 异步获取MapMatching结构
    class MapMatchingApi : public IBDMapMatchingListener {
	    BDMapMatchingInfo m_MapMatchingInfo；
	    
	    void onMapMatchingInfoUpdated(
		    const BDMapMatchingInfo& MapMatchingInfo) {
		    // 周期触发
		    // bdlog_v 记录MapMatchingInfo输出数据
		    doSomeThingWithMapMatchingInfo(MapMatchingInfo);
		}
    }
	m_pMapMatchingApi = new MapMatchingApi();
	m_pMapMatching->setEventListener(m_pMapMatchingApi);
	// onMapMatchingInfoUpdated triggered periodly