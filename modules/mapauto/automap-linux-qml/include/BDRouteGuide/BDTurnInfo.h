/**
 * @file BDTurnInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_GUIDE_BDTURNINFO_H
#define BAIDU_MAPAUTO_NAVI_GUIDE_BDTURNINFO_H

#include <BDNaviTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace guide {

/**
 * @brief BDTurnInfo. This is a kind of data class that has property about basic turn information.
 * @details In conventional navigation, engine and HMI application are tightly coupled because navigation development company prefer full-stack development.
 * So there are legacy such as iconId(it's UX spec and should be managed by a application not a engine)
 * @author Sun Fengyan (sunfengyan@baidu.com)
 * @todo just few functions are declared now, iconId -> enum, check tollfare necessasary?, use pimpl and hide private member variables
 */
class BDTurnInfo {
public:
    BDTurnInfo();
    ~BDTurnInfo();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDTurnInfo(const BDTurnInfo& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDTurnInfo& operator=(const BDTurnInfo& object);

    /**
     * @brief Get GeoCoord of node
     * @retval BDGeoCoord location of node
     */
    common::BDGeoCoord& getGeoCoord() const;

    /**
     * @brief Get remain distance from current car location to node
     * @retval BDUInt32 distance (meter or feet in accordance with HRouteGuideSettings.setDistanceUnit())
     */
    BDUInt32 getRemainDistance() const;

    /**
     * @brief Get direction name
     * @retval std::wstring direction name such as Seoul or First St
     */
    std::string& getDirectionName() const;

    /**
     * @brief Get node name
     * @retval std::wstring node name such as Seoul IC or Junction.
     */
    std::string& getNodeName() const;

    /**
     * @brief Get toll fare
     * @retval BDUInt32 toll fare
     */
    BDUInt32 getTollFare() const;

    /**
     * @brief Get icon id
     * @retval BDUInt32 icon id
     */
    BDUInt32 getIconId() const;

    /**
     * @brief Get icon file name
     * @param[out] iconFileName icon file name
     */
    std::string getIconFileName() const;

    /**
     * @brief Get next road name
     * @param[out] nextNoadName
     */
    std::string getNextRoadName() const;
    /**
     * @brief Set GeoCoord of node
     * @param[in] geoCoord location of node
     */
    void setGeoCoord(const common::BDGeoCoord& geoCoord);

    /**
     * @brief Set remain distance from current car location to node
     * @param[in] distance distance (meter or feet in accordance with HRouteGuideSettings.setDistanceUnit())
     */
    void setRemainDistance(const BDUInt32& distance);

    /**
     * @brief Set direction name
     * @param[in] directionName direction name such as Seoul or First St
     */
    void setDirectionName(const std::string& directionName);

    /**
     * @brief Set node name
     * @param[in] nodeName node name such as Seoul IC or Junction.
     */
    void setNodeName(const std::string& nodeName);

    /**
     * @brief Set toll fare
     * @param[in] tollFare toll fare
     */
    void setTollFare(const BDUInt32& tollFare);

    /**
     * @brief Set icon id
     * @param[in] iconId icon id
     */
    void setIconId(const BDUInt32& iconId);

    /**
     * @brief Set icon file name
     * @param[in] iconFileName icon file name
     */
    void setIconFileName(const std::string& iconFileName);

    /**
     * @brief Set next road name
     * @param[in] nextRoadName
     */
    void setNextRoadName(const std::string& nextRoadName);
private:
    class BDTurnInfoImpl;
    BDTurnInfoImpl *m_pImpl;
};

} // namespcae guide
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_GUIDE_BDTURNINFO_H
