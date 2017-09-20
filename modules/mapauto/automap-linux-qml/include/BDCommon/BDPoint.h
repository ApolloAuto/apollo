/**
 * @file BDPoint.h
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

#ifndef BAIDU_MAPAUTO_COMMON_BDPOINT_H
#define BAIDU_MAPAUTO_COMMON_BDPOINT_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace common {

/**
 * @brief The BDPoint class\n\n
 * @details Support for x, y coordination
 * @author Baidu MapAuto Team
 */
class BDPoint {
public:
    /**
     * @brief The constructor of data class
     */
    BDPoint();

    /**
     * @brief The constructor of data class
     * @details This constructor set x, y coordination
     * @param[in] x x coordination
     * @param[in] y y coordination
     */
    BDPoint(const BDUInt32& x, const BDUInt32& y);

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] point The source object to be copied
     */
    BDPoint(const BDPoint& point);

    /**
     * @brief The destructor of data class
     */
    virtual ~BDPoint();

    /**
     * @brief This function is to override the assignment operator
     * @param[in] point The source object to be assigned
     * @retval BDPoint Reference of this class
     */
    BDPoint& operator = (const BDPoint& point);

    /**
     * @brief This function is to override == operator
     * @param[in] point comparable object
     * @retval TRUE if two objects are same
     * @retval FALSE if two objects are different
     */
    BDBool operator==(const BDPoint& point) const;

    /**
     * @brief This function set x, y coordination
     * @details This function set x, y coordination
     * @param[in] x  x coordination
     * @param[in] y  y coordination
     */
    void set(const BDUInt32& x, const BDUInt32& y);

    /**
     * @brief This function set x coordination
     * @details This function set x coordination
     * @param[in] x  x coordination
     */
    void setX(const BDUInt32& x);

    /**
     * @brief This function set y coordination
     * @details This function set y coordination
     * @param[in] y  y coordination
     */
    void setY(const BDUInt32& y);

    /**
     * @brief This function get x, y coordination
     * @details This function get x, y coordination
     * @param[out] x  x coordination
     * @param[out] y  y coordination
     */
    void get(BDUInt32* x, BDUInt32* y) const;

    /**
     * @brief This function get x coordination
     * @details This function get x coordination
     * @retval BDUInt32 x coordination
     */
    BDUInt32 getX() const;

    /**
     * @brief This function get y coordination
     * @details This function get y coordination
     * @retval BDUInt32 y coordination
     */
    BDUInt32 getY() const;

private:
    class BDPointImpl;
    BDPointImpl* m_pImpl;
};

} // common
} // mapauto
} //baidu

#endif // BAIDU_MAPAUTO_COMMON_BDPOINT_H
