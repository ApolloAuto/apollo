/**
 * @file IHRouteGuideListener.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_IBDROUTEGUIDELISTENER_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_IBDROUTEGUIDELISTENER_H

#include <BDRouteGuideTypes.h>
#include <BDPOIInfo.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief The IBDRouteGuideBasicListener interface class
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class IBDRouteGuideBasicListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDRouteGuideBasicListener();

    /**
     * @brief Route guide status is changed
     * @param[in] status BDRouteGuideStatus
     */
    virtual void onRouteGuideStatusUpdated(const BDRouteGuideStatus& status) = 0;

    /**
     * @brief Route guide can enter routeCruise
     * @param[in] status
     */
    virtual void onRouteCruiseAvailableUpdated(bool& status) = 0;

    /**
     * @brief Turn Info callback
     * @details Client needs to provide proper guidance in this function such as to show TBT icon on screen or to play.
     * This function will be called every 1 second.
     * @param[in] turnInfo basic information including 2 upcoming turn info
     * @retval NONE
     * @sa BDTurnInfo
     */
    virtual void onTurnInfoUpdated(const std::vector<BDTurnInfo>& turnInfoList) = 0;

    /**
     * @brief Route deviated callback
     * @details After getting this callback, client should make a decision to re-route or not
     * @retval NONE
     */
    virtual void onRouteDeviated() = 0;

    /**
     * @brief Road name updated callback
     * @param[in] roadName updated road name
     * @retval NONE
     */
    virtual void onRoadNameUpdated(const std::string& roadName) = 0;

    /**
     * @brief Highway Info updated callback
     * @param[in] highwayInfo
     * @retval NONE
     */
    virtual void onHighwayInfoUpdated(const BDHighwayInfo& highwayInfo) = 0;

    /**
     * @brief Address updated callback
     * @param[in] address updated address
     * @retval NONE
     */
    virtual void onAddressUpdated(const baidu::mapauto::common::BDAddress& address) = 0;

    /**
     * @brief Road type updated callback
     * @param[in] roadType the type of road
     * @retval NONE
     */
    virtual void onRoadTypeUpdated(const BDRoadType& roadType) = 0;

    /**
     * @brief Speech updated callback
     * @details Every speeches regarding route guidance and alert
     * @param[in] type Importance type of the speech
     * @param[in] speech Text to be spoken
     * @retval NONE
     */
    virtual void onSpeechUpdated(const int32_t& type, const std::string& speech) = 0;

    /**
     * @brief Arrived at waypoints
     * @param[out] pointIdx index of waypoints from 0 to n-1 of waypoints
     * @retval NONE
     */
    virtual void onWaypointArrived(const BDUInt32& pointIdx) = 0;

    /**
     * @brief Arrived at destination
     * @retval NONE
     */
    virtual void onDestinationArrived() = 0;

    /**
     * @brief Route guide can post dest park
     * @param[in] status
     */
    virtual void onDestParkAvailableUpdated(bool& status) = 0;

    /**
     * @brief Route guide can post avoid route msg
     * @param[in] status
     */
    virtual void onAvoidRouteMsgUpdated(bool& status) = 0;


    /**
     * @brief Route guide can post road condition updated msg
     * @param[in] status
     */
    virtual void onRoadConditionUpdated(bool& status) = 0;

     /**
     * @brief change route updated callback
     * @param[in] status the status of changing route
     * @retval NONE
     */
    virtual void onChangeRouteUpdated(const BDRouteGuideChangeRouteType& status) = 0;

    /**
     * @brief lane info updated callback
     * @param[in] laneInfo the detail of lane
     * @retval NONE
     */
    virtual void onLaneInfoUpdated(const BDLaneInfo& laneInfo) = 0;

    /**
     * @brief Route is regenerated
     * @param[in] finished true : route re-generation is finished
     * false : route re-generation is started
     */
    virtual void onRouteReGenerated(const BDRouteReGeneratingStatus& status) = 0;

    virtual void onRefreshRouteUpdated(const BDRefreshRouteStatusType& result) = 0;
};

/**
 * @brief The IHRouteGuideViewListener interface class
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class IBDRouteGuideViewListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDRouteGuideViewListener();

    /**
     * @brief ILS image view updated callback
     * @param[in] hInfo HILSInfo which should be displayed on map
     * hInfo which has remainDistance as 0 will be called always to hide ILS image view on the app.
     * @retval NONE
     */
    virtual void onILSImageViewUpdated(const BDILSImageViewInfo& hInfo) = 0;

    /**
     * @brief ILS image view hide callback
     * @param[in] status
     */
    virtual void onHideILSImageViewUpdated(bool& status) = 0;

};

/**
 * @brief The IBDRouteGuideAlertListener
 * @details Client can get alert event such as camera, safety, traffic and user poi.
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class IBDRouteGuideAlertListener {
public:
    /**
     * @brief The user might implement the destructor
     */
    virtual ~IBDRouteGuideAlertListener();

    /**
     * @brief Camera callback
     * @details Client needs to provide proper guidance in this function such as to show Camera icon on screen or to alert speed limitation
     * @param[in] cInfo List of BDCameraInfo, Each item means one Camera, handle it in order.
     * @retval NONE
     * @sa BDCameraInfo
     */
    virtual void onCameraInfoUpdated(const BDCameraInfo& cInfo) = 0;

    /**
     * @brief Safety alert callback
     * @param[in] sInfo List of BDSafetyInfo. Each item means one Safety information.
     * @retval NONE
     */
    virtual void onSafetyInfoUpdated(const BDSafetyInfo& sInfo) = 0;
};

/**
 * @brief The IBDRouteGuideOptionalListener
 * @details This class provides callback for optional guide event
 * @todo To be discussed with actual use-cases
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class IBDRouteGuideOptionalListener {
public:
    virtual ~IBDRouteGuideOptionalListener();

    /**
     * @brief Time Restict Zone alert callback
     * @retval NONE
     */
    virtual void onTimeRestrictedRoadEntered() = 0;    
};

} // namespcae guide
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDROUTEGUIDELISTENER_H
