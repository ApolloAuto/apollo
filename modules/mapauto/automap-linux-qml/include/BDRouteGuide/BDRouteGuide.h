/**
 * @file BDRouteGuide.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDE_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDE_H

#include <BDNaviTypes.h>
#include <BDRouteGuideTypes.h>
#include <IBDRouteGuideListener.h>
#include <BDRouteInfo.h>

//baidumapengine
#include "BNAInterface/navi_auto_adapter_if.h"
#include "BNAObserver/IAdapterMsgObserver.h"

#define BDNAVI_ROUTE_GUIDE_DESTIATION_MAX_BOUNDARY 200
#define BDNAVI_ROUTE_GUIDE_DESTIATION_MIN_BOUNDARY 30


namespace baidu {
namespace mapauto {
namespace navi {
namespace guide {

#define  LOCATION_POINT_RATIO 100000.0

/**
 * @brief BDRouteGuide This is controller of route guide operation. \n
 * There is only one guidance in a car, so this class is singletone
 * @details Target route is set by calling setRoute(HRouteSearch*)
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @sa BDRouteInfo
 * @todo  
 */
class BDRouteGuide : public IAdapterMsgObserver
{
public:
    /** 
     * @brief Get singletone instance of BDRouteGuide
     * @details It is intendef not for users, buf for engines
     */
    static BDRouteGuide* getInstance();

    /**
     * @brief set callback listener
     * @details Set listener that handles all of guide event
     * @param[in] pListener instance of IHRouteGuideListener class
     * @retval NONE
     * @sa IBDPOISearchListener
     */

    void setBasicEventListener(IBDRouteGuideBasicListener* pListener);

    /**
     * @brief set callback listener
     * @details Set listener that handles all of guide event
     * @param[in] pListener instance of IHRouteGuideListener class
     * @retval NONE
     * @sa IBDPOISearchListener
     */
    void setViewEventListener(IBDRouteGuideViewListener* pListener);

    /**
     * @brief set callback listener
     * @details Set listener that handles all of guide event
     * @param[in] pListener instance of IHRouteGuideListener class
     * @retval NONE
     * @sa IBDPOISearchListener
     */
    void setAlertEventListener(IBDRouteGuideAlertListener* pListener);

    /**
     * @brief Set which route to be guide
     * @details It is possible to create HRouteSearch multiple instance,\n
     * but only one of them can be target of guidance. \n
     * Calling this function, start new guidance session using HRouteSearch(1st parameter) \n
     * simulation(if running) or existing guidance would be stopped. 
     * @param[in] routeSearch 
     * @retval H_ERROR invalid HRouteSearch return H_ERROR
     * @retval H_OK Changing target route succeeds
     * @sa HRouteSearch
     * @sa startRouteGuidance
    */
    BDResult setRoute(const route::BDRouteInfo& routeSearch);

    /**
     * @brief Check route set or not
     * @retval true route is not set yet.
     * @retval false route is set already.
     */
    BDBool isRouteSet() const;

    /**
     * @brief Refresh route to avoid congestion
     */
    int refreshRoute();

    /**
     * @brief Start route guidance refering to set HRouteSearch
     * @details This function is available after setRoute called with valid route search instance
     * @retval BDResult::OK succeed
     * @retval otherwise error
     * @sa setRoute
     */
    BDResult startRouteGuidance();

    /**
     * @brief Stop started, paused or resumed guidance
     * @details When route guidance is stopped, set route will be cleared and IBDRouteGuideBasicListener.onGuideChanged \
     * will be called accroding to virtual route with current heading
     * @retval BDResult::OK succeed
     * @retval otherwise error
     */    
    BDResult stopRouteGuidance();

    /** 
     * @brief Get current route guide status
     * @retval BDRouteGuideStatus::NOT_AVAILABLE guide is not available
     * @retval BDRouteGuideStatus::GENERAL_GUIDE general guiding only is running
     * @retval BDRouteGuideStatus::ROUTE_GUIDING general guiding and route guiding are running
     */
    BDRouteGuideStatus getRouteGuideStatus();


    /**
     * @brief get Remain distance of destination or waypoint
     * @param[in] nIndex index, when index is more than the number of (waypoint + destination), get distance of destination
     * @param[out] pDistance metric unit
     * @retval H_ERROR no set Route, or wrong index
     * @retval H_OK succeed
     */
    BDResult getRemainDistance(const BDUInt32& nIndex, BDUInt32& distance) const;

    /**
     * @brief get Remain time of destination or waypoint
     * @param[in] nIndex index, when index is more than the number of (waypoint + destination), get remain time of destination
     * @param[out] pTime second
     * @retval H_ERROR no set Route, or wrong index
     * @retval H_OK succeed
     */
    BDResult getRemainTime(const BDUInt32& nIndex, BDUInt32& remainTime) const;

    /**
     * @brief get Name of the current road where car located 
     * @param[out] roadName name of road where the car is on
     * @retval H_ERROR no set Route
     * @retval H_OK succeed
     */
    BDResult getRoadName(std::string& roadName) const;

    /**
     * @brief Get type of the current road where car located
     * @param[out] roadType type of road as BDRoadType
     * @retval BDResult::OK succeed
     * @retval otherwise Can't get the current road type
     */
    BDResult getRoadType(BDRoadType& roadType) const;

    /**
     * @brief Get address where car located
     * @param[out] address address as BDAddress
     * @retval BDResult::OK succeed
     * @retval otherwise Can't get the current road type
     */
    BDResult getAddress(baidu::mapauto::common::BDAddress& address) const;

    /**
     * @brief Get mapMatch result
     * @param[out] longitude
     * @param[out] latitude
     * @param[out] angle
     * @param[out] altitude
     * @param[out] speed
     */
    BDResult getMapMatchInfo(BDUInt32& longitude, BDUInt32& latitude, BDFloat& angle, BDUInt32& altitude, BDUInt32& speed);

    /**
     * @brief Set status of map auto level
     * @param[in] status
     */
    BDResult setMapAutoLevelStatus(const bool status);

    /**
     * @brief Set memory scale of map
     * @param[in] memoryScale
     */
    BDResult setMapMemoryScale(const int  memoryScale);

    /**
     * @brief Set stopRouteGuide Speak
     * @param[in] status
     */
    BDResult setStopRouteGuideSpeak(bool status);

    /**
     * @brief set config of route cruise
     * @param[in] bCloseSpeedLimit
     * @param[in] bCloseSpeedCamera
     * @param[in] bCloseTrafficLightCameraSpeak
     * @param[in] bClosePeccanryCameraSpeak
     * @param[in] bCloseTrafficSign
     */
    BDResult setCruiseConfig(bool bCloseSpeedLimit, bool bCloseSpeedCamera, bool bCloseTrafficLightCameraSpeak, bool bClosePeccanryCameraSpeak, bool bCloseTrafficSign);

    /**
     * @brief Start route cruise
     */
    BDResult startRouteCruise();

    /**
     * @brief Stop route cruise
     */
    BDResult stopRouteCruise();

    /**
     * @brief Start simulation
     */
    BDResult startSimulation();

    /**
     * @brief Pause simulation
     */
    BDResult pauseSimulation();

    /**
     * @brief Stop simulation
     */
    BDResult stopSimulation();

    /**
     * @brief Set simulation speed
     * @param[in] speed
     */
    BDResult setSimulationSpeed(const BDFloat speed) const;

    /**
     * @brief Set simulation type
     * @param[in] simulationType
     */
    BDResult setSimulationType(BDRouteGuideSimulationType simulationType);

    /**
     * @brief load simulation log file
     * @param[in] logfile
     */
    BDResult loadSimulationLogFile(const std::string logfile) const;


    /**
     * @brief get raster expand map image by path of image
     * @param[in] pBGMap
     * @param[in] pArrowMap
     * @param[in] unFlag
     * @param[in] pByteBuf
     * @param[in] unLength
     */
    BDResult getRasterExpandMapImage(const char* pImagePath, const unsigned int imageType, unsigned char** pByteBuf, unsigned int& length) const;


    /**
     * @brief Set status of enable roadCondition AutoUpdate
     * @param[in] bEnable
     */
    BDResult setEnableRoadConditionAutoUpdate(bool bEnable);

    /**
     * @brief update road condition
     * @param[in] updateType
     */
    BDResult updateRoadCondtion(BDRGUpdateRoadConditionEnum updateType = BDRGUpdateRoadConditionEnum::ONLYRC);

    /**
     * @brief get car progress of road condition
     * @param[in] fProgress
     */
    BDResult getCarProgress(float& fProgress);

    /**
     * @brief get road condition
     * @param[in] arrRoadCondition
     */
    BDResult getRoadCondition(std::vector<BDRoadCondition>& arrRoadCondition);

    /**
     * @brief enter viewALL
     * @param[in] left
     * @param[in] top
     * @param[in] right
     * @param[in] bottom
     * @param[in] height
     * @param[in] width
     */
    BDResult enterViewAll(BDRouteGuideViewAllType type);

    /**
     * @brief exit viewAL
     */
    BDResult exitViewAll();

    /**
     * @brief switch to avoid Route
     * @param[in] bSwitch
     * @param[in] routeIdx
     */
    BDResult switchRoute(const bool bSwitch = true, const unsigned int& routeIdx = 0);

        /**
     * @brief change route: main to slave, slave to main, etc
     */
    BDResult onlineChangeRoute();

    /**
     * @brief set speech function status: open or close
     */
    BDResult setSpeechFunctionStatus(const bool& status);
	
    /**
     * @brief get curRoadNameByPos everytime
     * @param[in]longitue  gcj02*100000
     * @param[in]latitude  gcj02*100000 
     * @retval curRoadName
     */
    std::string getCurRoadNameByPos(int longitude, int latitude);

private:
    ///////////////////////////////update function///////////////////////////////
    bool Update (int unMsgID, int unArg1, void* nArg2);

    ///////////////////////////////handle message function///////////////////////////////
    void onHandleRouteGuideStatusChangeMsg();

    void onHandleSimpleInfoMsg();

    void onHandleRasterExpandMapMsg();

    void onHandleRasterExpandMapHideMsg();

    void onHandleAssistInfoMsg();

    void onHandleRemainInfoMsg();

    void onHandleRoadNameChangeMsg();

    void onHandleHighwayInfoMsg();

    void onHandleParkingMsg();

    void onHandleAvoidRouteMsg();

    void onHandleUpdateRoadConditionMsg();

    void onHandleYawMsg();

    void onHandleReRoutePlanMsg(int rpStatus);

    void onHandleChangeRouteMsg(int resChangeRoute);

    void onHandleLaneUpdateMsg();

    void onHandleSpeechText();

    void onHandleCruiseSignal();

    void onHandleRefreshRouteUpdate(int32_t result);

    void attachAdapterObserver();

    void attachSingleAdapterObserver(BNAGuideMessageTypeEnum msgType);

    void detachAdapterObserver();

    void detachSingleAdapterObserver(BNAGuideMessageTypeEnum msgType);

    //deal function
    int getClientSafetyByAssitType(BNAGuideAssistantType assitType); //get client safety type

    std::string getTurnNameByType(int turnType);

private:
    BDRouteGuide();
    ~BDRouteGuide();
    
    /**
     * @brief Disable copy constructor
     */
    BDRouteGuide(const BDRouteGuide &);
     /**
     * @brief Disable assign operator
     */
    const BDRouteGuide &operator = (const BDRouteGuide &);

    class BDRouteGuideImpl;
    BDRouteGuideImpl* m_pImpl;


private:
    BDRouteGuideStatus m_guideStatus;
};

} // namespcae guide
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDE_H
