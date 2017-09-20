/**
 * @file BDRoutePosition.h
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

#ifndef BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEPOSITION_H
#define BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEPOSITION_H

#include <BDRouteTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace route {

/**
 * @brief This is the data class that handles Route Position.
 * @author wuwei08 (wuwei08@baidu.com)
 */
class BDRoutePosition {
public:
    /**
     * @brief The constructor of BDRoutePosition
     */
    BDRoutePosition();

    /**
     * @brief The destructor of BDRoutePosition
     */
    ~BDRoutePosition();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDRoutePosition(const BDRoutePosition& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDRoutePosition& operator=(const BDRoutePosition& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDRoutePosition& object) const;

    /**
     * @brief This function returns the route position based on HLonLat format.
     * @retval BDGeoCoord The geo coordination
     * //@param[out] position The geo position
     * //@retval BDResult::INVALID the position is not set
     * //@retval BDResult::OK the position is properly returned
     * @sa setPosition
     */
    common::BDGeoCoord& getPosition() const;

    /**
     * @brief This function sets the route position with the HLonLat format.
     * @param[in] position BDGeoCoord type's route position data to be set
     * @retval NONE
     * @sa getPosition
     */
    void setPosition(const common::BDGeoCoord& position);

    /**
     * @brief This function is to get the id of the place if exist
     * @retval std::string the id of place (it might be different in other country)
     * @sa setPlaceId
     */
    std::string getPlaceId() const;

    /**
     * @brief This function is to set the place id if exist
     * @param[in] placeId The id of place (KOR: LinkID, CHN: POI ID)
     * @sa getPlaceId
     */
    void setPlaceId(const std::string& placeId);

    /**
     * @brief This function is to check the geo coordination is valid or not
     * @retval TRUE The geo coordination is valid
     * @retval FALSE The geo coordination is not valid
     */
    BDBool isValid() const;

    /**
     * @deprecated No need, it can be linkID or placeID
     * @brief This function is to get placeInfo key list
     * @details The information key is the information categories for the position
     * @param[out] placeInfoKeys The list of information types
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::UNAVAILABLE The engine is not available
     * @todo List of Keys must be defined
     */
    BDResult getPlaceInfoKeys(std::vector<std::string>& placeInfoKeys) const;

    /**
    * @deprecated No need, it can be linkID or placeID
     * @brief This function is to set the information of the geo coordination if required
     * @param[in] placeInfo The attribute name which is used as key
     * @param[in] value The value of information
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::INVALID The attribute is not exist
     */
    BDResult setPlaceInfo(const std::string& placeInfo, const std::string& value);

    /**
    * @deprecated No need, it can be linkID or placeID
     * @brief This function is to get the placeInfo value
     * @details The information indicates the property of the street at the position
     * @param[in] placeInfo The information name
     * @param[out] value The value of information
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::INVALID The attribute is not exist
     */
    BDResult getPlaceInfo(const std::string& placeInfo, std::string& value) const;

private:
    class BDRoutePositionImpl;
    BDRoutePositionImpl* m_pImpl;
};

} // namespcae route
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEPOSITION_H
