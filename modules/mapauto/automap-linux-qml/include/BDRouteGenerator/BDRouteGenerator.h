/**
 * @file BDRouteGenerator.h
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

#ifndef BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEGENERATOR_H
#define BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEGENERATOR_H

#include <BDRouteTypes.h>
#include <BDRouteOption.h>
#include <BDRoutePosition.h>
#include <BDRouteInfo.h>
#include <BNAObserver/IAdapterMsgObserver.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace route {

typedef enum _BDCarTypeEnum
{
    BD_CAR_TYPE_06L = 1,
    BD_CAR_TYPE_08L = 2,
    BD_CAR_TYPE_09L = 3,
    BD_CAR_TYPE_DEFAULT= 0 ,
    BD_CAR_TYPE_10L = 4,
    BD_CAR_TYPE_10T = 5,
    BD_CAR_TYPE_11L = 6,
    BD_CAR_TYPE_12L = 7,
    BD_CAR_TYPE_12T = 8,
    BD_CAR_TYPE_13L = 9,
    BD_CAR_TYPE_13T = 10,
    BD_CAR_TYPE_14L = 11,
    BD_CAR_TYPE_14T = 12,
    BD_CAR_TYPE_15L = 13,
    BD_CAR_TYPE_15T = 14,
    BD_CAR_TYPE_16L = 15,
    BD_CAR_TYPE_16T = 16,
    BD_CAR_TYPE_18L = 17,
    BD_CAR_TYPE_18T = 18,
    BD_CAR_TYPE_19L = 19,
    BD_CAR_TYPE_19T = 20,
    BD_CAR_TYPE_20L = 21,
    BD_CAR_TYPE_20T = 22,
    BD_CAR_TYPE_21L = 23,
    BD_CAR_TYPE_21T = 24,
    BD_CAR_TYPE_22L = 25,
    BD_CAR_TYPE_22T = 26,
    BD_CAR_TYPE_23L = 27,
    BD_CAR_TYPE_23T = 28,
    BD_CAR_TYPE_24L = 29,
    BD_CAR_TYPE_24T = 30,
    BD_CAR_TYPE_25L = 31,
    BD_CAR_TYPE_25T = 32,
    BD_CAR_TYPE_26L = 33,
    BD_CAR_TYPE_27L = 34,
    BD_CAR_TYPE_27T = 35,
    BD_CAR_TYPE_28L = 36,
    BD_CAR_TYPE_28T = 37,
    BD_CAR_TYPE_29L = 38,
    BD_CAR_TYPE_30L = 39,
    BD_CAR_TYPE_30T = 40,
    BD_CAR_TYPE_32L = 41,
    BD_CAR_TYPE_32T = 42,
    BD_CAR_TYPE_33L = 43,
    BD_CAR_TYPE_34L = 44,
    BD_CAR_TYPE_35L = 45,
    BD_CAR_TYPE_35T = 46,
    BD_CAR_TYPE_36L = 47,
    BD_CAR_TYPE_36T = 48,
    BD_CAR_TYPE_37L = 49,
    BD_CAR_TYPE_38L = 50,
    BD_CAR_TYPE_38T = 51,
    BD_CAR_TYPE_39L = 52,
    BD_CAR_TYPE_40L = 53,
    BD_CAR_TYPE_40T = 54,
    BD_CAR_TYPE_42L = 55,
    BD_CAR_TYPE_43L = 56,
    BD_CAR_TYPE_44L = 57,
    BD_CAR_TYPE_44T = 58,
    BD_CAR_TYPE_45L = 59,
    BD_CAR_TYPE_46L = 60,
    BD_CAR_TYPE_47L = 61,
    BD_CAR_TYPE_47T = 62,
    BD_CAR_TYPE_48L = 63,
    BD_CAR_TYPE_48T = 64,
    BD_CAR_TYPE_50L = 65,
    BD_CAR_TYPE_50T = 66,
    BD_CAR_TYPE_52L = 67,
    BD_CAR_TYPE_53L = 68,
    BD_CAR_TYPE_54L = 69,
    BD_CAR_TYPE_55L = 70,
    BD_CAR_TYPE_56L = 71,
    BD_CAR_TYPE_57L = 72,
    BD_CAR_TYPE_58L = 73,
    BD_CAR_TYPE_60L = 74,
    BD_CAR_TYPE_60T = 75,
    BD_CAR_TYPE_62L = 76,
    BD_CAR_TYPE_62T = 77,
    BD_CAR_TYPE_63L = 78,
    BD_CAR_TYPE_64L = 79,
    BD_CAR_TYPE_65L = 80,
    BD_CAR_TYPE_65T = 81,
    BD_CAR_TYPE_66L = 82,
    BD_CAR_TYPE_67L = 83,
    BD_CAR_TYPE_68L = 84,
    BD_CAR_TYPE_71T = 85,
    BD_CAR_TYPE_73L = 86,
    BD_CAR_TYPE_78L = 87,
    BD_CAR_TYPE_78T = 88,
    BD_CAR_TYPE_80T = 89,
    BD_CAR_TYPE_97L = 90,
    BD_CAR_TYPE_98T = 91,
    BD_CAR_TYPE_120L = 92,
    BD_CAR_TYPE_127L = 93,
    BD_CAR_TYPE_COUNT = 94
}BDCarTypeEnum;


class BDCarInfo {
public:
    BDCarInfo();
    BDCarInfo(const std::string carNum,
                     const std::string carProvinceName,
                     const BDCarTypeEnum carType);
    ~BDCarInfo();
    BDCarInfo(const BDCarInfo& object);
    BDCarInfo& operator=(const BDCarInfo& object);
    BDBool operator==(const BDCarInfo& object)const;
    std::string getCarNumber( ) const;
    std::string getCarProviceName( ) const;
    BDCarTypeEnum getCarType( ) const;
private:
    class BDCarInfoImpl;
    BDCarInfoImpl* m_pImpl;
};

//Forward class since IBDRouteGeneratorListener uses BDRouteGenerator
class BDRouteGenerator;

//namespace baidu {
//namespace navi_adapter {
//        class IAdapterMsgObserver;
//}
//}

/**
 * @brief This interface class must be implemented if callbacks are needed for BDRouteGenerator class.
 * @details If user wants to receive the status of route generator callbacks, this class must be implemented.
 * @author Hansen(bianheshan@baidu.com)
 */
class IBDRouteGeneratorListener {
public:
    /**
     * @brief Destructor must be implemented
     */
    virtual ~IBDRouteGeneratorListener();

    /**
     * @brief Route Generator status callback
     * @details This function will be invoked when route search status is updated.
     * @pre BDRouteSearch.setEventListener must be set
     * @param[in] status the route generating status
     * @param[in] numOfRoutes the number routes that are generated
     * @param[in] generator the instance of route generator that are performed
     */
    virtual void onGenerateStatusUpdated(const BDRouteGenerator& generator, const BDRouteGeneratingStatus& status, const BDByte& numOfRoutes = 0) = 0;
};

/**
 * @brief This class is used for generating routes.
 * @details Each BDRouteGenerator can generate multiple routes at a time. The number of routes that are generated
 *          depends on the navi engines and possible routes.
 * @author Hansen (bianheshan@baidu.com)
 */
class BDRouteGenerator: public baidu::navi_adapter::IAdapterMsgObserver {
public:
    /**
     * @brief The constructor which should be initiated in order to perform route generating
     */
    BDRouteGenerator();
    /**
     * @brief When the route generator do not need anymore, this should be called
     */
    ~BDRouteGenerator();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDRouteGenerator(const BDRouteGenerator& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDRouteGenerator& operator=(const BDRouteGenerator& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDRouteGenerator& object) const;

    /**
     * @brief This function is to register listener in order to deal with the callback function.
     * @details This must be set if user wants to receive the callback events from route generator class.
     * @pre BDRouteGenerator must be declared
     * @param[in] listener IBDRouteGeneratorLisenter object to be registered
     * @retval NONE
     * @note Example code
     * @code
        class Listener: public IBDRouteGeneratorLisener
        {
        public:
            Listener()
            {
                ...
            }
            ~Listener()
            {
                ...
            }
            void onGenerateStatusUpdated(const BDRouteGeneratingStatus& status, const BDByte& numOfRoutes, const BDRouteGenerator& generator)
            {
                ...
            }
        };

        BDRouteGenerator* routeGenerator = new BDRouteGenerator();
        IBDRouteGeneratorListener* listener = new IBDRouteGeneratorListener();

        routeGenerator->setEventListener(listener);
     * @endcode
     */
    void setEventListener(IBDRouteGeneratorListener* listener);

    /**
     * @brief This function is to generate the routes.
     * @pre setStartPosition, setDestination must be called before this function is called
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR The startPosition or destination is not set
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa cancel
     */
    BDResult generate();

    /**
     * @brief This function is to cancel the current generating operation.
     * @pre This function only works when the status is generating
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR Nothing to cancel
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa generate
     */
    BDResult cancel();

    /**
     * @brief This function is to select route.
     * @pre This function only works when the status is generated
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR Nothing to cancel
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa generate
     */
    BDResult selectRoute(const BDUInt64& routeId);

    /**
     * @brief This function is to get the route generating status.
     * @retval BDRouteGeneratingStatus::NONE Nothing happend before
     * @retval BDRouteGeneratingStatus::GENERATING The routes are currently generating
     * @retval BDRouteGeneratingStatus::FINISHED The routes are generated
     * @retval BDRouteGeneratingStatus::CANCELLED The route generating is cancelled
     */
    BDRouteGeneratingStatus getStatus() const;

    /**
     * @brief This function is to get the number of routes that are generated.
     * @retval BDByte The number of routes
     */
    BDByte getRoutesCount() const;

    /**
     * @brief This function is to get the generated route info
     * @details This info can be retrived by the route index
     * @param[in] index The index of generated route
     * @param[out] route The generated route info
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::OUT_OF_RANGE The index is not available
     * @retval BDResult::UNVAILABLE The navi engine is not available
     * @sa getRoutesInfo
     */
    BDResult getRouteInfo(const BDByte& index, BDRouteInfo& route) const;

    /**
     * @brief This function is to get all the generated routes' info
     * @param[out] route The generated route info
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNVAILABLE The navi engine is not available
     * @sa getRouteInfo
     */
    BDResult getRoutesInfo(std::vector<BDRouteInfo>& routes) const;

    /**
     * @brief This function is to get the currently selected route info
     * @param[out] route The currently selected route info
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::INVALID There is no selected route.
     * @sa getRouteInfo, getRoutesInfo
     */
    BDResult getSelectedRouteInfo(BDRouteInfo& route);

    /**
     * @brief setLocalCarInfo
     * @param carInfo
     * @return
     */
    BDResult setLocalCarInfo(const BDCarInfo& carInfo);
    /**
     * @brief This function is to add a route option for a route
     * @details The route option must be set in order to perform route generation
     * @param[in] option The option for route
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::UNAVAILABLE The engine is not available
     */
    BDResult addRouteOption(const BDRouteOption& option);

    /**
     * @brief This function is to clear route options
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::UNAVAILABLE The engine is not available
     */
    BDResult clearRouteOption();

    /**
     * @brief This function is to get the route option which is set
     * @param[out] option The route option that will be received
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::UNAVAILABLE The engine is not available
     */
    BDResult getRouteOption(const BDByte& index, BDRouteOption& option) const;

    /**
     * @brief This function is to get the number of route option
     * @retval BDByte The number of route option that are set
     */
    BDByte getRouteOptionCount() const;

    /**
     * @brief This function is to set the start position
     * @param[in] position The start position of route
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR The given position is wrong
     * @sa getStartPosition
     */
    BDResult setStartPosition(const BDRoutePosition& position);
    /**
     * @brief This function is to get the start position of route
     * @param[out] position The start position of route that is set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::ERROR The start position is not set
     * @sa setStartPosition
     */
    BDResult getStartPosition(BDRoutePosition& position) const;
    /**
     * @brief This function is to check whether start position set or not
     * @retval TRUE The start position is set
     * @retval FALSE The start position is not set
     * @sa setStartPosition
     */
    BDBool isExistStartPosition() const;
    /**
     * @brief This function is to erase the start position to be not set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult clearStartPosition();

    /**
     * @brief This function is to set the destination of the route
     * @param[in] position The destination to be set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa getDestination
     */
    BDResult setDestination(const BDRoutePosition& position);
    /**
     * @brief This function is to get the destination
     * @param[out] posistion The destination of the route.
     * @retval BDResult::OK The result of operation is success
     * @retval BDResult::ERROR The destination is not set
     * @sa setDestination
     */
    BDResult getDestination(BDRoutePosition& position) const;
    /**
     * @brief This function is to check whether destination is set or not
     * @retval TRUE The destination is set
     * @retval FALSE The destination is not set
     * @sa setDestination
     */
    BDBool isExistDestination() const;
    /**
     * @brief This function is to erase the destination to be not set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult clearDestination();

    /**
     * @brief This function is to add waypoint to the route
     * @details The waypoint is only added at the end of the waypoints
     * @param[in] waypoint The waypoint to be added
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa removeWaypoint
     */
    BDResult addWaypoint(const BDRoutePosition& waypoint);
    /**
     * @brief This function is to remove the waypoint by given index
     * @param[in] index The waypoint index
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::OUT_OF_RANGE The index is not exist
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa addWaypoint
     */
    BDResult removeWaypoint(const BDByte& index);
    /**
     * @brief This function is to get the waypoint that is set
     * @param[in] index The waypoint index
     * @param[out] waypoint The waypoint to be received
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::OUT_OF_RANGE The index is not available
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa getWaypointsCount
     */
    BDResult getWaypoint(const BDByte& index, BDRoutePosition& waypoint) const;
    /**
     * @brief This function is to count the number of waypoints
     * @retval BDByte the number of waypoints
     */
    BDByte getWaypointsCount() const;

    /**
     * @brief This function is to set the list of waypoints
     * @details This function removes previous waypoints and add the list of waypoints
     * @param[in] waypoints The list of waypoints with ordering
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa addWaypoint, getWaypoints
     */
    BDResult setWaypoints(const std::vector<BDRoutePosition>& waypoints);

    /**
     * @brief This function is to get the list of waypoints
     * @param[out] waypoints The list of waypoints container that will be received
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     * @sa addWaypoint, setWaypoints, getWaypointsCount
     */
    BDResult getWaypoints(std::vector<BDRoutePosition>& waypoints) const;

    /**
     * @brief This function erases all the waypoints that are set
     * @retval BDResult::OK The operation is done properly
     * @retval BDResult::UNAVAILABLE The navi engine is not available
     */
    BDResult clearWaypoints();

private:
    class BDRouteGeneratorImpl;
    BDRouteGeneratorImpl* m_pImpl;
    bool Update(int unMsgID, int unArg1, void *nArg2);
};

} // namespcae route
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEGENERATOR_H
