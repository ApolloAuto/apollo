/**
  * @file BDRectangle.h
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

#ifndef BAIDU_MAPAUTO_COMMON_BDRECTANGLE_H
#define BAIDU_MAPAUTO_COMMON_BDRECTANGLE_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace common {

/**
 * @brief The BDRectangle class\n\n
 * @details Support for rectangle coordination
 * @author Baidu MapAuto Team
 */
class BDRectangle {
public:
    /**
     * @brief The constructor of data class
     */
    BDRectangle();

    /**
     * @brief The constructor of data class
     * @details This constructor set x, y, width, height value for rectangle
     * @param[in] x x coordination
     * @param[in] y y coordination
     * @param[in] w width
     * @param[in] h height
     */
    BDRectangle(const BDUInt32& x, const BDUInt32& y, const BDUInt32& w, const BDUInt32& h);

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] rect The source object to be copied
     */
    BDRectangle(const BDRectangle& rect);

    /**
     * @brief The destructor of data class
     */
    virtual ~BDRectangle();

    /**
     * @brief This function is to override the assignment operator
     * @param[in] rect The source object to be assigned
     * @retval BDRectangle Reference of this class
     */
    BDRectangle& operator = (const BDRectangle& rect);

    /**
     * @brief This function is to override == operator
     * @param[in] rect comparable object
     * @retval TRUE if two objects are same
     * @retval FALSE if two objects are different
     */
    BDBool operator==(const BDRectangle& rect) const;

    /**
     * @brief This function set rectangle coordination
     * @details This constructor set x, y, width, height value of rectangle
     * @param[in] x x coordination
     * @param[in] y y coordination
     * @param[in] w width
     * @param[in] h height
     */
    void set(const BDUInt32& x, const BDUInt32& y, const BDUInt32& w, const BDUInt32& h);

    /**
     * @brief This function set x coordination
     * @details This constructor set x value of rectangle
     * @param[in] x x coordination
     */
    void setX(const BDUInt32& x);

    /**
     * @brief This function set y coordination
     * @details This constructor set y value of rectangle
     * @param[in] y y coordination
     */
    void setY(const BDUInt32& y);

    /**
     * @brief This function set width
     * @details This constructor set width value of rectangle
     * @param[in] w width
     */
    void setWidth(const BDUInt32& w);

    /**
     * @brief This function set height
     * @details This constructor set height value of rectangle
     * @param[in] h height
     */
    void setHeight(const BDUInt32& h);

    /**
     * @brief This function get rectangle coordination
     * @details This constructor get x, y, width, height value of rectangle
     * @param[out] x  x coordination
     * @param[out] y  y coordination
     */
    void get(BDUInt32* x, BDUInt32* y, BDUInt32* w, BDUInt32* h) const;

    /**
     * @brief This function get x coordination
     * @details This constructor get x value of rectangle
     * @retval BDUInt32 x coordination
     */
    BDUInt32 getX() const;

    /**
     * @brief This function get y coordination
     * @details This constructor get y value of rectangle
     * @retval BDUInt32 y coordination
     */
    BDUInt32 getY() const;

    /**
     * @brief This function get width
     * @details This constructor get width value of rectangle
     * @retval BDUInt32 width
     */
    BDUInt32 getWidth() const;

    /**
     * @brief This function get height
     * @details This constructor get height value of rectangle
     * @retval BDUInt32 height
     */
    BDUInt32 getHeight() const;

private:
    class BDRectangleImpl;
    BDRectangleImpl* m_pImpl;
};

} // common
} // mapauto
} // baidu
#endif // BAIDU_MAPAUTO_COMMON_BDRECTANGLE_H
