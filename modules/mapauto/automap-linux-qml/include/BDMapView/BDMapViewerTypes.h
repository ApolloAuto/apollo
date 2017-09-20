/**
 * @file BDMapViewerTypes.h
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

#ifndef BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWERTYPES_H
#define BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWERTYPES_H

namespace baidu{
namespace mapauto {
namespace navi {
namespace map {

enum class BDMapElement {
    TRAFFIC,            /**<TRAFFIC condition */
//    CAR,                    /**< Current Car position */
//    CURSOR,                /**< Center Cursor while dragging  */
//    GOAL_LINE,            /**<  Car to goal line */
//    CURSOR_LINE,         /**< Car to cusor line */
//    CONGESTION,          /**< CTT(Congestion Travel time) line */
//    PATTERN_BUILDING,   /**< 3D Pattern building  */
//    BOOKMARK,             /**< Point of bookmark */
//    ONEWAY,                /**< One way sign */
//    U_TURN,                /**< U-Turn sign  */
//    ROAD_NAME,            /**< Road name  */
//    GPS_TRACKING,        /**< GPS Tracking infomation */
//    GPS_MATCHING_INFO,  /**< GPS, DR, Map matching infomation for Engineering mode */
//    AROUND_POI,           /**< Around POI information */
//    ACCIDENT_INFO,       /**< Accident information */
    MAX
};

enum class BDNaviMapViewStatus{
    NAVI_VIEW_STATUS_VIEW_ALL,
    NAVI_VIEW_STATUS_BROWSE,
    NAVI_VIEW_STATUS_FLLOW_CAR_POSITION,
    MAX
};

enum class BDMapTheme {
    DAY_MODE1,       /**< Map theme for Day mode 1  */
    DAY_MODE2,       /**< Map theme for Day mode 2 (Optional) */
    NIGHT_MODE1,    /**< Map theme for Night mode 1  */
    NIGHT_MODE2,    /**< Map theme for Night mode 2 (Optional) */
    MAX
};

enum class BDMapViewMode {
    ERROR = -1,    /**< Error  */
    HEADINGUP,    /**< Heading up  */
    NORTHUP,      /**< North up  */
    BIRDVIEW,     /**< Bird view  */
    DRIVERVIEW,   /**< Driver view  */
    MAX,
};

enum class BDMapType {
    FRONT,           /**< Front  */
    ASSISTANT,      /**< Assistant  */
    REAR_LEFT,      /**< Rear Left  */
    REAR_RIGHT,     /**< Rear Right  */
    MAX,
};

enum class BDGuideMapType {
    JUNCTION_VIEW,      /**< Juction view  */
    VIRTUAL_MAP,        /**< Virtual map  */
    MAX,
};

class BDDynamicMapPoint {
public:
    std::string name;           /**< POI Name */
    common::BDGeoCoord point;   /**< Geo Coordinate */
    std::string tag;            /**< Icon Identity */
};

} // namespace map
} // namespace navi
} // namespace mapauto
} // baidu

#endif // BAIDU_MAPAUTO_NAVI_MAP_BDMAPVIEWERTYPES_H
