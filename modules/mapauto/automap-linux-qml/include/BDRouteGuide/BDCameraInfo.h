/**
 * @file BDCameraInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDCAMERAINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDCAMERAINFO_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief BDCameraType enum
 * @detail supet set of type
 */
enum class BDCameraType {
    NONE = 0,
    SPEED_STATIC,                   /**< [BAIDU] Static Speed Camera */
    SPEED_SECTION_START,            /**< [BAIDU] Speed Section Camera Start */
    SPEED_SECTION_END,              /**< Speed Section Camera End */
    SPEED_KEEP_LANE_SECTION_START,  /**< Speed + Keeping Lane Section Camera Start */
    SPEED_KEEP_LANE_SECTION_END,    /**< Speed + Keeping Lane Section Camera Start */
    SPEED_MOVE,                     /**< Moving Speed Camera */
    SPEED_SIGNAL,                   /**< Traffic Signal + Static Speed Camera */
    TRAFFIC_SIGNAL,                 /**< [Baidu Only] Traffic Signal */ 
    LAMP_METER,                     /**< Lamp Metering (Entering Expressway) Camera */
    SPEED_BUS,                      /**< Lane for bus only + Static Speed Camera*/
    LANE_BUS,                       /**< Lane for Bus Only (No Speed) */
    CUT_IN,                         /**< Cut in Camera */
    TAILGATING,                     /**< Tailgate at crossway Camera */
    SHOULDER,                       /**< A Shoulder Camera */
    SHOULDER_SERIES,                /**< Shoulder in series Camera */ 
    LOAD_ILLEGAL,                   /**< Illegal Load (for truck) Camera */
    LOAD_OVER,                      /**< Over load (for truck) Camera */
    TRAFFIC_INFO,                   /**< Correcting Traffic Info Camera */
    USER_REGISTER,                  /**< User Registered Speed/Signal Camera */
    OTHER,                          /**< [Baidu only] Bus Only Lane, Drive Against and many things */
    MAX
};

/**
 * @brief This is the data class that holds the camera information
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @details In conventional navigation, engine and application are tightly coupled because navigation development company prefer to full-stack
 * So there are legacy such as iconId(it's UX spec and should be managed by a application not a engine)
 * Traffic law is different in each country and each region.
 * @todo just few functions are declared now, iconId -> enum, type -> enum, use pimpl and hide private member variables
 */
class BDCameraInfo {
public:
	/**
     * @brief The constructor which should be initiated in order to use this class
     */
    BDCameraInfo();


    /**
     * @brief The constructor which has every parameters
     */
//    BDCameraInfo(const BDCameraType& type, const baidu::mapauto::common::HGeoCoord& geoCoord, const BDUInt32& remainDistance,
//                const BDUInt32& speedLimit, const BDUInt32& startTime, const BDUInt32& endTime,
//                const BDUInt16& averageSpeed, const BDBool& isOverSpeed);
    BDCameraInfo(const BDCameraType& type, const BDUInt32& remainDistance,
                const BDUInt32& speedLimit, const BDUInt32& startTime, const BDUInt32& endTime,
                const BDUInt16& averageSpeed, const BDBool& isOverSpeed);

    /**
     * @brief The destructor must be called when the camera information is not needed anymore
     */
    ~BDCameraInfo();

    /**
     * @brief copy constructor
     */
    BDCameraInfo (const BDCameraInfo& other);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] other The source object to be assigned
     */
    BDCameraInfo& operator=(const BDCameraInfo& other);

    /**
     * @brief This function is used to get type of camera
     * @retval BDCameraType type of camera
     * @sa BDCameraType
     */
    BDCameraType getType() const;

    /**
     * @brief This function is used to set type of camera
     * @param[in] eType type of camera
     * @sa BDCameraType
     */
    void setType(BDCameraType eType);

//    /**
//     * @brief This function is used to get position of camera
//     * @param[out] pLonlat postion of camera
//     */
//    baidu::mapauto::common::BDGeoCoord& getGeoCoord() const;

//    /**
//     * @brief This function is used to set position of camera (not for client)
//     * @param[in] lonlat postion of camera
//     */
//    void setGeoCoord(const baidu::mapauto::common::BDGeoCoord& geoCoord);

    /**
     * @brief This function is used to get remain distance to camera from position of the car
     * @return distance to camera
     * @todo If user can calculate remain driving distance from HRouteInfo. this api should be gone. 
     */
    BDUInt32 getRemainDistance() const;

    /**
     * @brief This function is used to set remain distance to camera (not for client)
     * @param[in] remainDistance distance to camera
     * @todo If user can calculate remain driving distance from HRouteInfo. this api should be gone. 
     */
    void setRemainDistance(const BDUInt32& distance);

    /**
     * @brief This function is used to get speed limit of the camera
     * @return Speed limitation
     */
    BDUInt32 getSpeedLimit() const;

    /**
     * @brief This function is used to set speed limit of camera (not for client)
     * @param[in] limit speed limit of camera
     */
    void setSpeedLimit(const BDUInt32& limit);

    /**
     * @brief This function is used to get start time of the bus only lane
     * @return BDUInt32 0 : No time restriction
     * otherwise : HHMM (eg. 1500 -> PM 03:00)
     * @todo Need to dicuss to move to option
     */
    BDUInt32 getStartTime() const;

    /**
     * @brief This function is used to set start time of bus only lane (not for client)
     * @param[in] time 0 : No time restriction
     * otherwise : HHMM (eg. 1500 -> PM 03:00)
     * @todo Need to dicuss to move to option
     */
    void setStartTime(const BDUInt32& time);

    /**
     * @brief This function is used to get end time of the bus only lane
     * @return BDUInt32 0 : No time restriction
     * otherwise : HHMM (eg. 1500 -> PM 03:00)
     * @todo Need to dicuss to move to option
     */
    BDUInt32 getEndTime() const;

    /**
     * @brief This function is used to set end time of bus only lane (not for client)
     * @param[in] time 0 : No time restriction
     * otherwise : HHMM (eg. 1500 -> PM 03:00)
     * @todo Need to dicuss to move to option
     */
    void setEndTime(const BDUInt32& time);

    /**
     * @brief This function is get average speed when SPEED_SECTION_START/END and SPEED_KEEP_LANE_SECTION_START/END is activated
     * @retval BDUInt16 average speed
     */
    BDUInt16 getAverageSpeed() const;

    /**
     * @brief This function is set average speed
     * @param[in] speed average speed
     */
    void setAverageSpeed(const BDUInt16& speed);

    /**
     * @brief This function is check weather speed is over or not
     * @retval true speed is over
     * @retval false speed is not over
     */
    BDBool isOverSpeed() const;

    /**
     * @brief This function is to enable or disable over speed
     * @param[in] enable enablement of over speed
     */
    void enableOverSpeed(const BDBool& enable);

private:
	class Impl;
	Impl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDCAMERAINFO_H
