/**
 * @file HRouteGuideType.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDETYPES_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDETYPES_H

#include <BDNaviTypes.h>
#include <BDTurnInfo.h>
#include <BDLaneInfo.h>
#include <BDSafetyInfo.h>
#include <BDCameraInfo.h>
#include <BDILSImageViewInfo.h>
#include <BDJunctionViewInfo.h>
#include <BDVirtualMapViewInfo.h>
#include <BDAddress.h>
#include <BDHighwayInfo.h>
#include <BDRoadCondition.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief BDRouteGuideStatus enum
 */
enum class BDRouteGuideStatus {
    NOT_AVAILABLE,          //**< All guide is NOT available */
    GENERAL_GUIDING,        //**< General guiding (Watchdog) */
    ROUTE_GUIDING,          //**< General guiding + Route guiding */
    MAX,
};

/** 
 * @brief BDRouteReGeneratingStatus enum
 */
enum class BDRouteReGeneratingStatus {
    STARTED,    //**< Route Re-Generating is started */
    FINISHED,   //**< Route Re-Generating is finished successfully */
    FAILED,     //**< Route Re-Generating is failed because of error */     
    MAX
};

typedef enum _BDRouteGuideSimulationType
{
    NAVITRACK = 1,
    NMEA,
    CCOS,
    DESAYSV,
    ROUTE,
    ANDROID_DEV,
    DEFAULT
}BDRouteGuideSimulationType;

typedef enum _BDRouteGuideViewAllType {
    SIMPLE_PORT = 1,                            /**< view all for Showing SimpleInfo in vertical screen */
    RASTER_PORT,                                  /**< view all for Showing RasterInfo in vertical screen */
    SIMPLE_LAND,                                  /**< view all for Showing SimpleInfo in cross screen */
    RASTER_LAND,                                  /**< view all for Showing RasterInfo in cross screen */
    ROUTE_DETAIL_PORT,                      /**< view all for Showing RouteDetail in vertical screen , similar to 1*/
    ROUTE_DETAIL_LAND                      /**< view all for Showing RouteDetail in cross screen  , similar to 3*/
}BDRouteGuideViewAllType;


typedef enum _BDRouteGuideChangeRouteType {
    CHANGETO_MAINROUTE = 1,
    CHANGETO_SLAVEROUTE,
    CHANGETO_PARALLELROUTE,
    HIDECHANGEROUTE
}BDRouteGuideChangeRouteType;

typedef enum _BDRefreshRouteStatusType {
    REFRESH_ROUTE_INVALID = -1,   /**< invalid*/
    REFRESH_ROUTE_SUCCESS,        /**< refresh success*/
    REFRESH_ROUTE_FAILED,         /**< refresh failed*/
    REFRESH_ROUTE_NEWROUTE,       /**< update new Route*/
    REFRESH_ROUTE_NO_NEWROUTE,    /**< no Route*/
    REFRESH_ROUTE_NET_TIMEOUT,    /**< timeout*/
    REFRESH_ROUTE_OTHER_ROUTE    /**< other route*/
}BDRefreshRouteStatusType;

} // namespcae guide
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDETYPES_H
