/**
 * @file BDRouteInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEINFO_H
#define BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEINFO_H

#include <BDRouteTypes.h>
#include <BDRoutePosition.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace route {


//maping the tbts values
typedef enum _BDManeuverType
{
    BD_MANEUVER_TYPE_INVALID ,			            /**<  无效值 */
    BD_MANEUVER_TYPE_FRONT ,			            /**<  直行 */
    BD_MANEUVER_TYPE_RIGHT_FRONT ,		            /**<  右前方转弯 */
    BD_MANEUVER_TYPE_RIGHT ,			            /**<  右转 */
    BD_MANEUVER_TYPE_RIGHT_BACK ,		            /**<  右后方转弯 */
    BD_MANEUVER_TYPE_BACK ,				            /**<  掉头 */
    BD_MANEUVER_TYPE_LEFT_BACK ,		            /**<  左后方转弯 */
    BD_MANEUVER_TYPE_LEFT ,				            /**<  左转 */
    BD_MANEUVER_TYPE_LEFT_FRONT ,		            /**<  左前方转弯 */
    BD_MANEUVER_TYPE_RING ,				            /**<  环岛 */
    BD_MANEUVER_TYPE_RING_OUT ,			            /**<  环岛出口 */
    BD_MANEUVER_TYPE_LEFT_SIDE ,		            /**<  普通/JCT/SAPA二分歧 靠左 */
    BD_MANEUVER_TYPE_RIGHT_SIDE ,		            /**<  普通/JCT/SAPA二分歧 靠右 */
    BD_MANEUVER_TYPE_LEFT_SIDE_MAIN ,	            /**<  左侧走本线 */
    BD_MANEUVER_TYPE_BRANCH_LEFT_MAIN ,             /**<  靠最左走本线 */
    BD_MANEUVER_TYPE_RIGHT_SIDE_MAIN ,	            /**<  右侧走本线 */
    BD_MANEUVER_TYPE_BRANCH_RIGHT_MAIN,             /**<  靠最右走本线 */
    BD_MANEUVER_TYPE_CENTER_MAIN ,                  /**<  中间走本线 */
    BD_MANEUVER_TYPE_LEFT_SIDE_IC ,		            /**<  IC二分歧左侧走IC */
    BD_MANEUVER_TYPE_RIGHT_SIDE_IC ,	            /**<  IC二分歧右侧走IC */
    BD_MANEUVER_TYPE_BRANCH_LEFT ,		            /**<  普通三分歧/JCT/SAPA 靠最左 */
    BD_MANEUVER_TYPE_BRANCH_RIGHT ,		            /**<  普通三分歧/JCT/SAPA 靠最右 */
    BD_MANEUVER_TYPE_BRANCH_CENTER ,	            /**<  普通三分歧/JCT/SAPA 靠中间 */
    BD_MANEUVER_TYPE_START ,			            /**<  起始地 */
    BD_MANEUVER_TYPE_DEST ,				            /**<  目的地 */
    BD_MANEUVER_TYPE_VIA1 ,				            /**<  途径点1 */
    BD_MANEUVER_TYPE_VIA2 ,				            /**<  途径点2 */
    BD_MANEUVER_TYPE_VIA3 ,				            /**<  途径点3 */
    BD_MANEUVER_TYPE_VIA4 ,				            /**<  途径点4 */
    BD_MANEUVER_TYPE_IN_FERRY ,			            /**<  进入渡口 */
    BD_MANEUVER_TYPE_OUT_FERRY ,			            /**<  脱出渡口 */
    BD_MANEUVER_TYPE_TOLL_GATE ,                     /**<  收费站 */
    BD_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT_IC ,        /**<  IC二分歧左侧直行走IC */
    BD_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT_IC ,       /**<  IC二分歧右侧直行走IC */
    BD_MANEUVER_TYPE_LEFT_SIDE_STRAIGHT ,           /**<  普通/JCT/SAPA二分歧左侧 直行 */
    BD_MANEUVER_TYPE_RIGHT_SIDE_STRAIGHT ,          /**<  普通/JCT/SAPA二分歧右侧 直行 */
    BD_MANEUVER_TYPE_BRANCH_LEFT_STRAIGHT ,         /**<  普通/JCT/SAPA三分歧左侧 直行 */
    BD_MANEUVER_TYPE_BRANCH_CENTER_STRAIGHT ,       /**<  普通/JCT/SAPA三分歧中央 直行 */
    BD_MANEUVER_TYPE_BRANCH_RIGHT_STRAIGHT ,        /**<  普通/JCT/SAPA三分歧右侧 直行 */
    BD_MANEUVER_TYPE_BRANCH_LEFT_IC ,               /**<  IC三分歧左侧走IC */
    BD_MANEUVER_TYPE_BRANCH_CENTER_IC ,             /**<  IC三分歧中央走IC */
    BD_MANEUVER_TYPE_BRANCH_RIGHT_IC ,              /**<  IC三分歧右侧走IC */
    BD_MANEUVER_TYPE_BRANCH_LEFT_IC_STRAIGHT ,      /**<  IC三分歧左侧直行 */
    BD_MANEUVER_TYPE_BRANCH_CENTER_IC_STRAIGHT ,	/**<  IC三分歧中间直行 */
    BD_MANEUVER_TYPE_BRANCH_RIGHT_IC_STRAIGHT ,     /**<  IC三分歧右侧直行 */
    BD_MANEUVER_TYPE_STRAIGHT_2BRANCH_LEFT_BASE ,   /**<  八方向靠左直行*/
    BD_MANEUVER_TYPE_STRAIGHT_2BRANCH_RIGHT_BASE ,  /**<  八方向靠右直行*/
    BD_MANEUVER_TYPE_STRAIGHT_3BRANCH_LEFT_BASE  ,  /**<  八方向靠最左侧直行*/
    BD_MANEUVER_TYPE_STRAIGHT_3BRANCH_MIDDLE_BASE , /**<  八方向沿中间直行 */
    BD_MANEUVER_TYPE_STRAIGHT_3BRANCH_RIGHT_BASE ,  /**<  八方向靠最右侧直行 */
    BD_MANEUVER_TYPE_LEFT_2BRANCH_LEFT_BASE ,       /**<  八方向左转+随后靠左 */
    BD_MANEUVER_TYPE_LEFT_2BRANCH_RIGHT_BASE ,      /**<  八方向左转+随后靠右 */
    BD_MANEUVER_TYPE_LEFT_3BRANCH_LEFT_BASE ,       /**<  八方向左转+随后靠最左 */
    BD_MANEUVER_TYPE_LEFT_3BRANCH_MIDDLE_BASE ,     /**<  八方向左转+随后沿中间 */
    BD_MANEUVER_TYPE_LEFT_3BRANCH_RIGHT_BASE ,      /**<  八方向左转+随后靠最右 */
    BD_MANEUVER_TYPE_RIGHT_2BRANCH_LEFT_BASE ,      /**<  八方向右转+随后靠左 */
    BD_MANEUVER_TYPE_RIGHT_2BRANCH_RIGHT_BASE ,     /**<  八方向右转+随后靠右 */
    BD_MANEUVER_TYPE_RIGHT_3BRANCH_LEFT_BASE ,      /**<  八方向右转+随后靠最左 */
    BD_MANEUVER_TYPE_RIGHT_3BRANCH_MIDDLE_BASE ,    /**<  八方向右转+随后沿中间 */
    BD_MANEUVER_TYPE_RIGHT_3BRANCH_RIGHT_BASE,      /**<  八方向右转+随后靠最右 */
    BD_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_LEFT_BASE,  /**<  八方向左前方靠左侧 */
    BD_MANEUVER_TYPE_LEFT_FRONT_2BRANCH_RIGHT_BASE,  /**<  八方向左前方靠右侧 */
    BD_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_LEFT_BASE,  /**<  八方向右前方靠左侧 */
    BD_MANEUVER_TYPE_RIGHT_FRONT_2BRANCH_RIGHT_BASE,    /**<  八方向右前方靠右侧 */
    BD_MANEUVER_TYPE_BACK_2BRANCH_LEFT_BASE ,      /**<  八方向掉头+随后靠左 */
    BD_MANEUVER_TYPE_BACK_2BRANCH_RIGHT_BASE ,     /**<  八方向掉头+随后靠右 */
    BD_MANEUVER_TYPE_BACK_3BRANCH_LEFT_BASE ,      /**<  八方向掉头+随后靠最左 */
    BD_MANEUVER_TYPE_BACK_3BRANCH_MIDDLE_BASE ,    /**<  八方向掉头+随后沿中间 */
    BD_MANEUVER_TYPE_BACK_3BRANCH_RIGHT_BASE,      /**<  八方向掉头+随后靠最右 */
}BDManeuverType;

/**
 * @brief This is the data class holds the generated route information
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class BDRouteInfo {
public:
    /**
     * @brief The constructor which should be initiated in order to use this class
     */
    BDRouteInfo();

    /**
     * @brief The constructor with pre-loaded values
     * @param[in] uid Unique ID (0 : INVALID)
     * @param[in] routeType The type of route
     * @param[in] points The point of start, waypoints, and destination
     * @param[in] geoArea The geographical area
     * @param[in] tbts Turn-By-Turn Information
     * @param[in] roads The list of main roads
     * @param[in] meters The list of meters for each roads
     * @param[in] trafficStatus The traffic status for each roads
     * @param[in] fare The fare if exist
     * @param[in] distance The distance of the route
     * @param[in] duration Time that will take
     * @param[in] interchanges The list of interchanges
     */
    BDRouteInfo(const BDUInt64& uid,
               const BDRouteType& routeType,
               const std::vector<BDRoutePosition>& points,
               const baidu::mapauto::common::BDGeoArea& geoArea,
               const std::vector<BDByte>& tbts,
               const std::vector<std::string>& roads,
               const std::vector<BDUInt32>& meters,
               const std::vector<BDByte>& trafficStatus,
               const BDUInt32& fare,
               const BDUInt32& trafficLight,
               const BDUInt32& distance,
               const BDUInt32& duration,
               const std::vector<std::string>& interchanges,
               const BDBool isUsingLocalCarInfo
              );

    /**
     * @brief The destructor must be called when the route info is not needed anymore
     */
    ~BDRouteInfo();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDRouteInfo(const BDRouteInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDRouteInfo& operator=(const BDRouteInfo& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDRouteInfo& object) const;

    /**
     * @brief This function is to get the route unique id
     * @retval BDUInt64 0 if route uid is not set, otherwise unique id
     */
    BDUInt64 getId() const;

    /**
     * @brief This function is to get the generated route type
     * @retval BDRouteType The route type
     */
    BDRouteType getRouteType() const;

    /**
     * @brief This function is to get the list of points
     * @details The list of points include at least 2 (start, destination), and it can have waypoints
     * @param[out] points The receive container for the list
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR The getting points have problem
     */
    BDResult getPoints(std::vector<BDRoutePosition>& points) const;

    /**
     * @brief This function is to get the coordinate area of the route
     * @retval BDGeoArea The geo graphical area
     * //@param[out] area The geographical area
     * //@retval BDResult::OK The operation is done properly
     * //@retval BDResult::ERROR The geo area does not exist
     */
    baidu::mapauto::common::BDGeoArea getArea() const;

    /**
     * @brief This function is to get the list of main streets
     * @param[out] roads The list of main roads for the route
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR The list of roads are not exist
     */
    BDResult getMainRoads(std::vector<BDByte>& tbts, std::vector<std::string>& roads, std::vector<BDUInt32>& meters, std::vector<BDByte>& trafficStatus) const;

    /**
     * @brief This function is to get the fare of the route
     * @retval BDUInt32 The actual fare, 0 if the fare is not needed
     */
    BDUInt32 getFare() const;

    /**
     * @brief This function is to get the total distance of the route
     * @retval BDUInt32 The distance of the route in meters
     */
    BDUInt32 getTotalDistance() const;

    /**
     * @brief This function is to get the total time that will take
     * @retval BDUInt32 total time that will take in minutes
     */
    BDUInt32 getTotalTime() const;

    /**
     * @brief This function is to get the total traffic light that will take
     * @retval BDUInt32 total traffic light that will take in nums
     */
    BDUInt32 getTrafficLight() const;
    /**
     * @brief This function is to get the list of Inter-changes
     * @param[out] listOfICs The container that will holds the list of ICs
     */
    void getInterChanges(std::vector<std::string>& listOfICs) const;

    BDBool getIsUsingLocalCarInfo() const;
private:
    class BDRouteInfoImpl;
    BDRouteInfoImpl* m_pImpl;
};

} // namespcae route
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEINFO_H
