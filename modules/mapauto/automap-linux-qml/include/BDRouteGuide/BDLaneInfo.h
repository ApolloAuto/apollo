/**
 * @file BDLaneInfo.h
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
 
#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDLANEINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDLANEINFO_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief The HLaneVehicleType enum
 * @details Dedicated type for vehicle or special objective
 */
enum class BDLaneType {
    NORMAL,
    BUS,
    HIGHPASS,
    HOV,
    MAX,
};

/**
 * @brief The BDLaneLevelType enum
 * @details grade level of lane
 */
enum class BDLaneLevelType {
    NORMAL,
    UNDERPASS,
    OVERPASS,
    MAX,
};

/**
 * @brief The BDLaneVariationType enum
 */
enum class BDLaneVariationType {
    NONE,
    INCREASE,
    DECREASE,
    MAX,
};

/**
 * @brief The BDLaneValidType enum
 */
enum class BDLaneValidType {
    NONE,
    POSSIBLE,
    RECOMMEND,
    IMPOSSIBLE,
    MAX,
};

/*
 * @brief This is the data class that holds the lane direction information of BDLaneItem
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class BDLaneDirection {
public:
    BDLaneDirection();
    ~BDLaneDirection();

    /**
     * @brief The constructor which should be initiated in order to use this class
     * @param[in] clockDirection clock based direction
     * @param[in] validType lane valid type
     */      
    BDLaneDirection(const BDByte& clockDirection, const BDLaneValidType& validType);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneDirection(const BDLaneDirection& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneDirection& operator=(const BDLaneDirection& object);

    /**
     * @brief Get clock based direction
     * @retval HUInt8 clock based direction : 0 ~ 11
     */
    BDByte getDirection() const;

    /**
     * @brief Set clock based direction
     * @param[in] direction clock based direction
     */
    void setDirection(const BDByte& direction);

    /**
     * @brief Get valid type of direction
     * @retval BDLaneValidType valid type of direction
     */    
    BDLaneValidType getValidType() const;

    /**
     * @brief Set lane valid type
     * @param[in] validType lane valid type
     */
    void setValidType(const BDLaneValidType& validType);

private:
    class BDLaneDirectionImpl;
    BDLaneDirectionImpl* m_pImpl;
};

/**
 * @brief This is the data class that holds the lane item information for BDLaneInfo
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class BDLaneItem {
public:
    BDLaneItem();
    ~BDLaneItem();

    /**
     * @brief The constructor which should be initiated in order to use this class
     * @param[in] laneType lane type
     * @param[in] levelType level type of lane
     * @param[in] variationType variation type of lane
     * @param[in] directions lane directions
     */
    BDLaneItem(const BDLaneType& laneType, const BDLaneLevelType& levelType,
              const BDLaneVariationType& variationType, const std::vector<BDLaneDirection>& directions);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneItem(const BDLaneItem& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneItem& operator=(const BDLaneItem& object);

    /**
     * @brief Get dedicated lane type
     * @retval BDLaneType The dedicated lane type of BDLaneItem
     */
    BDLaneType getLaneType() const;

    /**
     * @brief Set lane type
     * @param[in] laneType lane type
     */
    void setLaneType(const BDLaneType& laneType);

    /**
     * @brief Get lane position type
     * @retval HLaneFacilityType The lane facility type of BDLaneItem
     */
    BDLaneLevelType getLevelType() const;

    /**
     * @brief Set lane level type
     * @param[in] levelType lane level type
     */
    void setLevelType(const BDLaneLevelType& levelType);

    /**
     * @brief Get land variation type
     * @retval BDLaneVariationType the lane variation(increse/decrease) type of BDLaneItem
     */
    BDLaneVariationType getVariationType() const;

    /**
     * @brief Set variation type
     * @param[in] variationType lane variation type
     */
    void setVariationType(const BDLaneVariationType& variationType);

    /**
     * @brief Get directions of lane
     * @retval std::vector<BDLaneDirection> The directions of BDLaneItem
     */
    std::vector<BDLaneDirection>& getDirections() const;

    /**
     * @brief Set directions of lane
     * @param[in] directions vector of direction
     */
    void setDirections(const std::vector<BDLaneDirection>& directions);

private:
    class BDLaneItemImpl;
    BDLaneItemImpl* m_pImpl;
};

/**
 * @brief This is the data class that holds the lane information for IBDRouteGuideBasicListener.onLaneUpdated.
 * @author Sun Fengyan (sunfengyan@baidu.com)
 */
class BDLaneInfo {
public:
    BDLaneInfo();
    ~BDLaneInfo();

    /**
     * @brief The constructor which should be initiated in order to use this class
     * @param[in] geoCoord geo-coord
     * @param[in] remainDistance remain distance
     * @param[in] items vector of lane item
     */
    BDLaneInfo(const common::BDGeoCoord& geoCoord, const BDUInt32& remainDistance, const std::vector<BDLaneItem>& items);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneInfo(const BDLaneInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDLaneInfo& operator=(const BDLaneInfo& object);

    /**
     * @brief Get directions of lane
     * @retval std::vector<BDLaneDirection> The directions of BDLaneItem
     */
    common::BDGeoCoord& getGeoCoord() const;

    /**
     * @brief Set geo-coord
     * @param[in] geoCoord geo-coord
     */
    void setGeoCoord(const common::BDGeoCoord& geoCoord);

    /**
     * @brief Get remain distance of lane
     * @retval std::vector<BDLaneDirection> The directions of BDLaneItem
     */
    BDUInt32 getRemainDistance() const;

    /**
     * @brief Set remain distance
     * @param[in] remainDistance remain distance
     */
    void setRemainDistance(const BDUInt32& remainDistance);

    /**
     * @brief Get lane items
     * @retval std::vector<BDLaneItem> lane items of BDLaneInfo
     */
    std::vector<BDLaneItem>& getLaneItems() const;

    /**
     * @brief Set lane items
     * @param[in] items vector of lane items
     */
    void setLaneItems(const std::vector<BDLaneItem>& items);

private:
    class BDLaneInfoImpl;
    BDLaneInfoImpl* m_pImpl;
};

} // namespace guide
} // namespace navi
} // namespace mapauto
} // baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDLANEINFO_H
