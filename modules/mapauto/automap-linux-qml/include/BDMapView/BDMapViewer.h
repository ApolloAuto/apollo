/**
 * @file BDMapViewer.h
 * @author Infotainment Software Development Team
 *
 * Copyright (c) 2017  Baidu MapAuto Company,
 * All Rights Reserved.
 *
 * Use and copying of this software and preparation of derivative works
 * based upon this software are permitted. Any copy of this software or
 * of any derivative work must include the above copyright notice, this
 * paragraph and the one after it.  Any distribution of this software or
 * derivative works must comply with all aplicable laws.
 *
 * This software is made available AS IS, and COPYRIGHT OWNERS DISCLAIMS
 * ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION CONTAINED HEREIN, ANY
 * LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS USE IS
 * EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
 * NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWER_H
#define BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWER_H

#include <vector>

#include <BDNaviTypes.h>
#include <BDMapViewerTypes.h>
#include <BDPOIInfo.h>
#include <BDRouteInfo.h>
#include <BNAObserver/IAdapterMsgObserver.h>
#include <BDPoint.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace map {

//Forward class since IBDMapViewerListener uses BDMapViewer
class BDMapViewer;
/**
 * @brief The IBDMapViewerListener class
 * @details This interface class should be implemented in a inherited class and registed to BDMapViewer instance
 * @author Wuwei(wuwei08@baidu.com)
 */
class IBDMapViewerListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDMapViewerListener();

    /**
     * @brief This function will be invoked when the center position of the BDMapViewer instance is changed
     * @details This function can be invoked by BDMapViewer.setMapPosition and BDMapViewer.enableFollowingCar()
     * @param[in] pos The geo-position of the center of BDMapViewer instance
     * @note Example Code
     * @code
       void MyBDMapViewerListener::onMapPositionChanged(const BDMapViewer& object, const common::BDGeoCoord& coord) {
           ...
       }
     * @endcode
     */
    virtual void onMapPositionChanged(const BDMapViewer& object, const common::BDGeoCoord& coord) = 0;

    /**
     * @brief This function will be invoked when the map level of the BDMapViewer instance is changed
     * @details This function can be invoke by the result of BDMapViewer.setMapLevel and auto zoom in/out during driving guidance
     * @param[in] level The map level of BDMapViewer instance
     * @note Example Code
     * @code
       void MyBDMapViewerListener::onMapLevelChanged(const BDMapViewer& object, const BDUInt32& level) {
           HLog::i("BDMapViewer", "Map level is changed to %d", level);
       }
     * @endcode
     */
    virtual void onMapLevelChanged(const BDMapViewer& object, const BDUInt32& level) = 0;

    /**
     * @brief This function can be invoked during getSelectItemInfo or getAntiGeoInfo if poi or basemap is clicked
     * @param object[out] BDMapViewer instance
     * @param info[out] Poi info of the chosen point
     */
    virtual void onMapPoiClicked(const BDMapViewer& object, const search::BDPOIInfo& info) = 0;

    /**
     * @brief This function can be invoked during getSelectItemInfo if BkgPoiMarker is clicked
     * @param object[out] BDMapViewer instance
     * @param index[out] Index of the Poi in Poi Array, starting from 0
     */
    virtual void onMapBkgPoiMarkerClicked(const BDMapViewer& object, const BDUInt32& index) = 0;
    virtual void onMapScreenShotFinished(const BDMapViewer& object) = 0;
};

/**
 * @brief The BDMapViewer class
 * @details This class provides control interfaces for map viewer
 * @author Wuwei(wuwei08@baidu.com)
 */
class BDMapViewer : public baidu::navi_adapter::IAdapterMsgObserver {
public:
    /**
     * @brief The constructor which should be initiated in order to perform map viewer
     */
    BDMapViewer();
    /**
     * @brief When the map viewer do not need anymore, this should be called
     */
    ~BDMapViewer();

    /**
     * @brief The constructor which should be initiated in order to perform map viewer with Sub-surface's width and height.
     * @details Create BDMapViewer object and setting width and height of Sub-surface.
     * @param[in] type map type such as front, rear left and so on
     * @param[in] width Sub-Surface's width value
     * @param[in] height Sub-Surface's height value
     */
    BDMapViewer(const BDMapType& type, const BDUInt32& width, const BDUInt32& height);

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDMapViewer(const BDMapViewer& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDMapViewer& operator=(const BDMapViewer& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDMapViewer& object) const;

    /**
     * @brief This function is to set event listener instance to get map viewer events
     * @details Map position changing and map level changing events can be monitored by registering a listener via this function.
     * @param[in] pListener The pointer of IBDMapViewerListener instance
     * @sa com::navi::IBDMapViewerListener
     */
    void setEventListener(IBDMapViewerListener* listener);

    /**
     * @brief This function is to draw a frame of map once
     * @details When the mapview need be displayed, call this function to draw a frame.
     */
    void draw();

    /**
     * @brief This function is to enable following car mode
     * @details While isFollowingCar is true, BDMapViewer.setMapPosition is ignored.\n
     * To move map while isFollowingCar is true, BDMapViewer::enableFollowingCar() should be called
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult enableFollowingCar();

    /**
     * @brief This function is to disable following car mode
     * @details While isFollowingCar is true, BDMapViewer.setMapPosition is ignored.\n
     * To move map while isFollowingCar is true, BDMapViewer::disableFollowingCar() should be called
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult disableFollowingCar();

    /**
     * @brief This function is to get the current follow car mode
     * @details While isFollowCar is true, BDMapViewer.setMapPosition is ignored.\n
     * To move map while isFollowCarEnabled is true, BDMapViewer::enableFollowingCar() should be called
     * @retval true following car is turned on
     * @retval false following car is turned off
     */

    BDBool isFollowingCarEnabled() const;
    /**
     * @brief This function is to change map position or move map
     * @details In general, this function is used to move map according to user input(touch and jog key).
     * @param[in] coord destination position where should be changed/moved to
     * @param[in] bSmoothMove true: map will be moved with animation
     * false: map position will be changed without animation
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       BDMapViewer mapViewer = new BDMapViewer();
       void moveTo(int x, int y) {
           BDPoint pt = {x,y};
           BDGeoCoord coord;
           mapViewer->screenToLonLat(pt, coord);
           mapViewer->setMapPosition(coord);
       }
     * @endcode
     * @todo bSmoothMove should be implemented
     */
    BDResult setMapPosition(const common::BDGeoCoord& coord, const BDBool& smoothMove = false);

    /**
     * @brief This function is to get the current map position
     * @details Out parameter, coord is given as the center position of map viewer.
     * @param[out] coord The reference of allocated BDGeoCoord type variable
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       BDMapViewer mapViewer = new BDMapViewer();
       BDGeoCoord coord;
       mapViewer->getMapPosition(coord);
     * @endcode
     */
    BDResult getMapPosition(common::BDGeoCoord& coord) const;

    /**
     * @brief [Not implemented yet] This function is to set map angle.
     * @details Map angle is calculated based on z-axis on north direction
     * This function can be worked when only the getFollowingCarMode() is HNAVIFOLLOWCARMODE_OFF
     * @param[in] degree Angle in degree
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @todo This function is not used. Deprecating should be considered.
     */
    BDResult setMapAngle(const BDFloat& degree);

    /**
     * @brief This function is to get the current map angle
     * @details Map angle is calculated based on z-axis on north direction
     * @retval float Angle in degree
     */
    BDFloat getMapAngle() const;

    /**
     * @brief This function is to translate display point to geo-position
     * @details In general, this functions is used to move touched map position.
     * @param[in] pt The display point in the BDMapViewer instance
     * @param[out] coord Translated geo-position
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       BDMapViewer mapViewer = new BDMapViewer();
       BDPoint pt = {640,360};
       BDGeoCoord coord;
       mapViewer->screenToGeoCoord(pt, coord);
       mapViewer)->setMapPosition(coord);
     * @endcode
     */
    BDResult screenToGeoCoord(const common::BDPoint& pt, common::BDGeoCoord& coord) const;

    /**
     * @brief This function is to translate geo-position to display point
     * @details Display point system is based on the relative position of map viewer dimension.
     * However this function is not used anywhere currently. Deprecating should be considered.
     * @param[in] coord The geo-position in the BDMapViewer instance
     * @param[out] pt Translated display point
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult geoCoordToScreen(const common::BDGeoCoord& coord, common::BDPoint& pt) const;

    /**
     * @brief This function is to set map level
     * @details level is not in meter. Refer to level description
     * @param[in] level Set current Map level
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa getMapLevel
     */
    BDResult setMapLevel(const BDUInt32& level);

    /**
     * @brief This function is to get the current zoom level
     * @details return current Map level
     * @retval BDUInt32 Map level
     * @sa setMapLevel
     */
    BDUInt32 getMapLevel() const;

    /**
     * @brief This function is to set map center offset
     * @details
     * @param[double x,double y]  is map center offset
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa setMapOffset
     */
    BDResult setMapOffset(double x,double y);

    /**
     * @brief This function is to get the Maximum zoom level
     * @details return Maximum level of navigation system
     * @retval BDUInt32 Map level
     */
    BDUInt32 getMaxMapLevel() const;

    /**
     * @brief This function is to get the Minimum zoom level
     * @details return Minimum level of navigation system
     * @retval BDUInt32 Map level
     */
    BDUInt32 getMinMapLevel() const;

    /**
     * @breif This function is to set the view mode
     * @details The view mode is such as HEADINGUP, NORTHUP, BIRDVIEW and DRIVERVIEW.\n
     * In general, setting view mode is used when user clicks compass icon on map viewer.
     * @param[in] mode view mode of map
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       BDMapViewer mapViewer = new BDMapViewer();
       void MainWindow::onHeadingUpButtonClicked() {
           mapViewer->setViewMode(BDMapViewMode::HEADINGUP);
       }
     * @endcode
     * @sa getViewMode
     */
    BDResult setViewMode(const BDMapViewMode& mode);

    /**
     * @brief This function is to get the current view mode
     * @details Default view mode is BDMapViewMode::NORTHP.
     * @retval BDMapViewMode view mode of map
     * @sa setViewMode
     */
    BDMapViewMode getViewMode() const;

    /**
     * @brief This function is to fit geo-area to the current screen area
     * @details This function is used to show entire route on a specfic rect
     * @pre In general, area should be given by HRouteSearch::getRouteArea()
     * @param[in] area The geo-area of the BDMapViewer instance
     * @param[in] screenRect The screen rect of the BDMapViewer instance
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       baidu::mapauto::common::BDRectangle rect = {0, 0, 1280, 720};
       BDGeoArea area;
       mapViewer->fitGeoAreaToScreen(area, rect);
     * @endcode
     */
    BDResult fitGeoAreaToScreen(const common::BDGeoArea& area, const common::BDRectangle& screenRect);

    /**
     * @brief This function is to set map theme
     * @details Map theme(Day/Night mode) is changed by user select or auto mode
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult setMapTheme(const BDMapTheme& theme);

    /**
     * @brief This function is to get current map theme
     * @details Current map theme is returned with BDMapTheme
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDMapTheme getMapTheme() const;

    /**
     * @brief This function is to clear all added routes
     * @details All Added routes is removed and disappeared on map viewer.
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @note Example Code
     * @code
       BDMapViewer mapViewer = new BDMapViewer();
       void MainWindow::onCancelRouteGuidance() {
           ...
           mapViewer->clearRoutes();
           ...
       }
     * @endcode
     */
    BDResult clearRoutes();

    /**
     * @brief This function is to get poiinfo of the poi which user clicked
     * @param[in] x screen coordinate x
     * @param[in] y screen coordinate y
     * @param[out] result: poiinfo
     * @retval HResult::OK The operation is done properly
     * @retval HResult::UNAVAILABLE The navi engine is not available
     */
    BDResult getSelectItemInfo(unsigned int x, unsigned int y, baidu::mapauto::navi::search::BDPOIInfo &poi_info);

    /**
     * @brief This function is to get routeIndex of routeplan which user clicked
     * @param[in] x screen coordinate x
     * @param[in] y screen coordinate y
     * @param[out] result: routeIndex of routeplan
     * @retval HResult::OK The operation is done properly
     * @retval HResult::UNAVAILABLE The navi engine is not available
     */
    BDResult getSelectRouteInfo(unsigned int x, unsigned int y, unsigned int& routeIndex);

    /**
     * @brief This function is to get poiinfo near by geo x,y
     * @details Current map theme is returned with BDMapTheme
     * @param[in] x screen coordinate x
     * @param[in] y screen coordinate y
     * @param[in] result: poiinfo
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult getAntiGeoInfo(unsigned int x, unsigned int y, baidu::mapauto::navi::search::BDPOIInfo &poi_info);

    /**
     * @brief This function is to add pop-up above a specific poi
     * @param coord gcj coordinate
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR There is something error
     */
    BDResult addPoiMarker(common::BDGeoCoord &coord);

    /**
     * @brief This function is to add background pop-up above a group of pois
     * @param geoArr poi gcj coordinate array
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR There is something error
     */
    BDResult addBkgPoiMarker(std::vector<common::BDGeoCoord> &geoArr);

    BDResult hidePoiMarker();

    BDResult hideBkgPoiMarker();

    /**
     * @brief This function is to highlight from added routes
     * @details Hightlighted routes is shown on map viewer.
     * @param[in] uid HRouteSearch search result uid which is highlighted.
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult highlightRoute(const BDUInt64& uid);

    /**
     * @brief This function is to set a chosen route and remove other routes
     * @details setChosenRoute is shown on map viewer.
     * @param[in] uid HRouteSearch search result uid which is highlighted.
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult chooseRoute(const BDUInt64& index);

    /**
     * @brief This function is to set map element
     * @details Selected map element is shown or not shown on the map viewer
     * @param[in] element BDMapElement option which is shown or not
     * @param[in] bVisible if value is not provided, true will be set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult setElementVisible(const BDMapElement& element, const BDBool& visible = true);

    /**
     * @brief This function is to check element is visible or not
     * @details Before set setElementVisible, check element's visibility
     * @param[in] element BDMapElement option which is shown or not
     * @retval true Element can be turned on
     * @retval false Element cannot be turned on
     */
    BDBool isElementVisible(const BDMapElement& element) const;

    /**
     * @brief This function is to resize the VBGL.
     * @details
     * @param[in] width, height
     * @retval true success
     * @retval false false
     */
    BDResult resize(BDUInt64 w, BDUInt64 h);

    /**
     * @brief This function is to add routes
     * @details If this operation is successful, routes are displayed on map viewer.
     * @param[in] routeHashKey HRouteSearch instance which has valid search result
     * @retval void
     */
    void addRoutes(const std::vector<baidu::mapauto::navi::route::BDRouteInfo> routeInfos);

    /**
     * @brief This function can drag map from start point to end point in the screen.
     * @details Users can drag map by this function.
     * If users call this function in fly gesture, non-zero speed should be passed.
     * Otherwise, speed should be set to 0.
     * @param[in] fromX from point, x, in the screen
     * @param[in] fromY from point, y, in the screen
     * @param[in] toX end point, x, in the screen
     * @param[in] toY end point, y, in the screen
     * @retval void
     */
    void dragMap(unsigned int fromX, unsigned int fromY,unsigned int toX, unsigned int toY, unsigned int speed);

    /**
     * @brief This function can start map render when map paused.
     * @details
     * @retval void
     */
    void MapResume();

    /**
     * @brief This function can stop map render.
     * @details
     * @retval void
     */
    void MapPause();

    /**
     * @brief This function is to set the current zoom level
     * @retval void
     */
    void setMapLevelHightPrecision(float level);

    /**
     * @brief This function is to get the current zoom level
     * @details return current level float
     * @retval float Map level
     */
    float getMapLevelHightPrecision();

    /**
     * @brief This function set mapview status in navigation.
     * @details
     * @retval BDResult
     */
    BDResult setNavigationMapViewStatus(const BDNaviMapViewStatus &status);

    /**
     * @brief This function set dynamic map points, each point uses tag to identify icon.
     * This function will reset all dynamic points data
     * @param points Array of dynamic map points
     * @param count Number of dynamic map points
     * @retval HResult
     */
    BDResult setDynamicMapPoints(const std::vector<BDDynamicMapPoint> points);

    /**
     * @brief This function add dynamic map points, each point uses tag to identify icon
     * @param points Array of dynamic map points
     * @param count Number of dynamic map points
     * @retval HResult
     */
    BDResult addDynamicMapPoints(const std::vector<BDDynamicMapPoint> points);

    /**
     * @brief This function set a dynamic map icon for a specific tag. This method
     * will deep copy image data, so users need to deal with memory of buffer
     * @param tag Identify a type for a icon or a point
     * @param buffer Image buffer, only support RGB888 or RGBA8888 currently.
     * @param width Image width
     * @param height Image height
     * @param channel Image channel, 3 for RGB888 and 4 for RGBA8888
     * @retval HResult
     */
    BDResult setDynamicMapImages(const std::string tag, BDChar* buffer, BDInt32 width, BDInt32 height, BDInt32 channel);

    /**
     * @brief Clean dynamic map points
     * @param tag Identify a type for a point
     * @retval HResult
     */
    BDResult cleanDynamicMapPoints(const std::string tag);

    /**
     * @brief Clean dynamic map images
     * @param tag
     * @retval HResult
     */
    BDResult cleanDynamicMapImages(const std::string tag);

    /**
     * @brief Clean all data of dynamic map
     * @retval HResult
     */
    BDResult cleanDynamicMapAllData();

    /**
     * @brief This function can draw a region on  mapview.
     * @details
     * @param[in] geoArr, coordinate of region.
     * @retval BDResult
     */
    BDResult setMapRegionData(const std::vector<common::BDGeoCoord> geoArr);

    /**
     * @brief This function can set color to a region on  mapview.
     * @details
     * @param[in] color rgba.
     * @retval BDResult
     */
    BDResult setMapRegionStyle(float r,
                              float g,
                              float b,
                              float a);

    /**
     * @brief This function can clear a region on  mapview.
     * @details
     * @retval BDResult
     */
    BDResult clearMapRegionData();

     /**
     * @brief This function can draw a line by user self-define point on mapview.
     * @details
     * @param[in] geoArr, coordinate of region.
     * @retval HResult
     */
    BDResult setMapLineData(const std::vector<common::BDGeoCoord> geoArr);

    /**
     * @brief This function can set color to a line on mapview.
     * @details
     * @param[in] color rgba.
     * @retval HResult
     */
    BDResult setMapLineStyle(float r,
                            float g,
                            float b,
                            float a,
                            float line_width);

    /**
     * @brief This function can clear a line on  mapview.
     * @details
     * @retval HResult
     */
    BDResult clearMapLineData();

    BDResult screenShot(int height, int width);

    BDResult getScreenShotResult(int& height, int& width, char** image);

private:
    class BDMapViewerImpl;
    BDMapViewerImpl* m_pImpl;
    bool Update(int unMsgID, int unArg1, void *nArg2);

private:
    static int _s_instance_count;
    static std::map<BNAScreenType, uint64_t> _s_handle_map;  //Map screen type to mapview handle
};

} // namespace map
} // namespace navi
} // namespace mapauto
} // baidu
#endif //BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWER_H
