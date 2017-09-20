/**
 * @file BDGeoArea.h
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

#ifndef BAIDU_MAPAUTO_NAVI_BDGEOAREA_H
#define BAIDU_MAPAUTO_NAVI_BDGEOAREA_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace common {

/**
 * @brief This class is the data class that handles the geo area
 * @details BDGeoArea is usually used with route generation
 * @author Hansen (bianheshan@baidu.com)
 */
class BDGeoArea {
public:
    /**
     * @brief The constructor of HGeo Area
     */
    BDGeoArea();

    /**
     * @brief The destructor of HGeo Area
     */
    ~BDGeoArea();

    /**
     * @brief Ths constructor of BDGeoArea with BDGeoCoord
     * @param[in] topLeft The GeoCoord of Top Left
     * @param[in] bottomRight The GeoCoord of Bottom Right
     */
    BDGeoArea(const common::BDGeoCoord& topLeft, const common::BDGeoCoord& bottomRight);

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDGeoArea(const BDGeoArea& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDGeoArea& operator=(const BDGeoArea& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDGeoArea& object) const;

    /**
     * @brief This function is to get the geo coord of top left position
     * @retval BDGeoCoord The geo coord of top left position
     */
    common::BDGeoCoord& getTopLeft() const;

    /**
     * @brief This function is to set the geo coord of top left position
     * @param[in] coord The geo coord of top left position
     */
    void setTopLeft(const common::BDGeoCoord& coord);

    /**
     * @brief This function is to get the geo coord of bottom right position
     * @retval BDGeoCoord The geo coord of bottom right position
     */
    common::BDGeoCoord& getBottomRight() const;

    /**
     * @brief This function is to set the geo coord of bottom right position
     * @param[in] coord The geo coord of bottom right position
     */
    void setBottomRight(const common::BDGeoCoord& coord);

private:
    class BDGeoAreaImpl;
    BDGeoAreaImpl* m_pImpl;
};

} // namespace common
} // namespace mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_BDGEOAREA_H
