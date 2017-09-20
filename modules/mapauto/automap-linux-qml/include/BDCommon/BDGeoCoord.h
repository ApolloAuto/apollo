/**
 * @file BDGeoCoord.h
 * @author MapAuto Linux Team
 *
 * Copyright (c) 2017  Baidu MapAuto Company,,
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

#ifndef BAIDU_MAPAUTO_COMMON_BDGEOCOORD_H
#define BAIDU_MAPAUTO_COMMON_BDGEOCOORD_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace common {

class BDGeoCoord {
public:
    /**
     * @brief The constructor of geo coord
     */
    BDGeoCoord();

    BDGeoCoord(BDInt32 lon, BDInt32 lat, BDInt32 alt);

    /**
     * @brief The destructor of geo coord
     */
    ~BDGeoCoord();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDGeoCoord(const BDGeoCoord& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDGeoCoord& operator=(const BDGeoCoord& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDGeoCoord& object) const;

    /**
     * @brief This function is to get the longitude
     * @retval BDInt32 Longitude of this geo coordination
     */
    BDInt32 getLongitude() const;

    /**
     * @brief This function is to set the longitude of this geo coordination
     * @param[in] longitude The longitude of the geo coordination
     */
    void setLongitude(const BDInt32& longitude);

    /**
     * @brief This function is to get the latitude
     * @retval BDInt32 latitude of this geo coordination
     */
    BDInt32 getLatitude() const;

    /**
     * @brief This function is to set the latitude of this geo coordination
     * @param[in] latitude The latitude of the geo coordination
     */
    void setLatitude(const BDInt32& latitude);

    /**
     * @brief This function is to get the altitude
     * @retval BDInt32 altitude in centimeter
     */
    BDInt32 getAltitude() const;

    /**
     * @brief This function is to set the altitude
     * @param[in] altitude The altitude in centimeter
     */
    void setAltitude(const BDInt32& altitude);

    static BDFloat getAngle(const BDGeoCoord& one, const BDGeoCoord& other);

private:
    class BDGeoCoordImpl;
    BDGeoCoordImpl* m_pImpl;
};

} // namespace common
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_COMMON_BDGEOCOORD_H
