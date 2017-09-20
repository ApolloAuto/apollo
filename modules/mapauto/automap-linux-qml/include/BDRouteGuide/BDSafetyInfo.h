/**
 * @file BDSafetyInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDSAFETYINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDSAFETYINFO_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief BDSafetyType enum
 * @note super set for all region
 */
enum class BDSafetyType {
    NONE = 0, 
    SPEED_BUMP,           /**< Speed Bump */
    ACCIDENT,             /**< Frequent Accident */ 
    ROCKS,                /**< Falling Rocks */
    FOG,                  /**< Dense Fog */
    ROADKILL,             /**< Road kill (to avoid wild animal) */ 
    RAIL,                 /**< Cross on train rail */
    CURVE,                /**< Curve */ 
    CURVE_SERIES,         /**< Curve in series */
    LEFT_CURVE,           /**< Left Curve */
    RIGHT_CURVE,          /**< Right Curve */
    LEFT_RIGHT_CURVE,     /**< Left and Right Curve */
    RIGHT_LEFT_CURVE,     /**< Right and Left Curve */
    SCHOOL_ZONE,          /**< School Zone */
    NARROW,               /**< Narrow Road */ 
    DOWNHILL,             /**< Down Hill */ 
    EXPRESS_CONSTRUCTION, /**< Expressway under Construction */
    JOINT,                /**< [Baidu Only] Joint roads (MAIN + small connection road */
    BLIND_SLOPE,          /**< [Baidu Only] Blind slope */
    DAMAGED_ROAD,         /**< [Baidu Only-Uneven] Damaged road */
    VILIAGE,              /**< [Baidu Only] Viliage */
    SLIP,                 /**< [Baidu Only] Slip by iced road or water */
    NO_OVERTAKE,          /**< [Baidu Only] No over taking */
    HONK,                 /**< [Baidu Only] Honk for against vehicles */
    NARROW_BRIDGE,        /**< [Baidu Only] Narrow bridge */
    CROSSWIND,            /**< [Baidu Only] Cross wind */
    UNDER_WATER,          /**< [Baidu Only] Under water roads */
    LOW_SPEED,            /**< [Baidu Only] Low Speed */
    JOINT_SAMESIZE_ROADS, /**< [Baidu Only] Joint same size roads */     
    MAX
};

/**
 * @brief This is the data class that holds the safety information
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @details when safety info guide is occured, it means that there is a something which driver needs to be careful of.
 */
class BDSafetyInfo {
public:
	/**
     * @brief The constructor which should be initiated in order to use this class
     */
    BDSafetyInfo();

    /**
     * @brief The destructor must be called when the safety information is not needed anymore
     */
    ~BDSafetyInfo();

    /**
     * @brief copy constructor
     */
    BDSafetyInfo (const BDSafetyInfo& other);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] other The source object to be assigned
     */
    BDSafetyInfo& operator=(const BDSafetyInfo& other);

    /**
     * @brief This function is used to get type of safety
     * @retval BDSafetyType type of safety
     * @sa BDSafetyType
     */
    BDSafetyType getType() const;

    /**
     * @brief This function is used to set type of safety
     * @param[in] eType type of safety
     * @sa BDSafetyType
     */
    void setType(BDSafetyType eType);

    /**
     * @brief This function is used to get position of safety
     * @return geoCoord postion of safety
     */
    baidu::mapauto::common::BDGeoCoord& getGeoCoord() const;

    /**
     * @brief This function is used to set position of camera (not for client)
     * @param[in] geoCoord postion of safety
     */
    void setGeoCoord(const baidu::mapauto::common::BDGeoCoord& geoCoord);

    /**
     * @brief This function is used to get remain distance to safety warning from position of the car
     * @todo If user can calculate remain driving distance from BDRouteInfo. this api should be gone.
     * @return distance to Safety
     */
    BDUInt32 getRemainDistance() const;

    /**
     * @brief This function is used to set remain distance to safety warning (not for client)
     * @param[in] distance distance to Safety
     * @todo If user can calculate remain driving distance from BDRouteInfo. this api should be gone.
     */
    void setRemainDistance(const BDUInt32& distance);


private:
    class BDSafetyInfoImpl;
    BDSafetyInfoImpl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDSAFETYINFO_H
