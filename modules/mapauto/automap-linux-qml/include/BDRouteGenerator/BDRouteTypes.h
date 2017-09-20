/**
 * @file BDRouteTypes.h
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

#ifndef BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTETYPES_H
#define BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTETYPES_H

#include <BDNaviTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace route {

/**
 * @brief This class illustrates the type of the route.
 */
enum class BDRouteType {
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
};

/**
 * @brief This class illustrates the route position type
 */
enum class BDRoutePositionType {
    INVALID,     //<** INVALID Position Type */
    NORMAL,      //<** NORMAL position */
    HIGHWAY,     //<** The position is on HIGHWAY */
    OVERPASS,   //<** The position is on the bridge */
    UNDERGROUD, //<** Ths position is under ground */
    BRIDGE,      //<** This position is over the bride */
    MAX
};

/**
 * @brief This class illustrates the route generating status
 */
enum class BDRouteGeneratingStatus {
    NONE,          //<** No generation progressed */
    GENERATING,  //<** Route is generating */
    FINISHED,    //<** Route generation is finished */
    CANCELLED,  //<** Route generation is cancelled */
    ERROR,      //<** Route generation is failed */
};

} // namespcae route
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTETYPES_H
