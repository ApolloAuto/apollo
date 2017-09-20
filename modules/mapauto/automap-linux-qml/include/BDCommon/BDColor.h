/**
 * @file BDColor.h
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

#ifndef BAIDU_MAPAUTO_COMMON_BDCOLOR_H
#define BAIDU_MAPAUTO_COMMON_BDCOLOR_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace common {

/**
 * @brief This is the color data class.
 * @details This class is used to control the color of surface(Background).
 * @author wuwei08 (wuwei08@baidu.com)
 */
class BDColor {
public:
    /**
     * @brief The constructor of data class
     */
    BDColor();

    /**
     * @brief The constructor of data class
     * @details The override copy constructor
     * @param[in] r red value (range: 0~255)
     * @param[in] g green value (range: 0~255)
     * @param[in] b blue value (range: 0~255)
     * @param[in] a alpha value (range: 0~255)
     */
    BDColor(const BDByte& r, const BDByte& g, const BDByte& b, const BDByte& a);

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] color The source object to be copied
     */
    BDColor(const BDColor& color);

    /**
     * @brief The destructor of data class
     */
    virtual ~BDColor();

    /**
     * @brief This function is to override the assignment operator
     * @param[in] color The source object to be assigned
     * @retval BDColor Reference of this class
     */
    BDColor& operator = (const BDColor& color);

    /**
     * @brief This function is to override == operator
     * @param[in] color comparable object
     * @retval TRUE if two objects are same
     * @retval FALSE if two objects are different
     */
    BDBool operator==(const BDColor& color) const;

    /**
     * @brief This function set the color value
     * @details This function set the color with red, green, blue, alpha (range: 0~255)
     * @param[in] r red value (range: 0~255)
     * @param[in] g green value (range: 0~255)
     * @param[in] b blue value (range: 0~255)
     * @param[in] a alpha value (range: 0~255)
     */
    void set(const BDByte& r, const BDByte& g, const BDByte& b, const BDByte& a);

    /**
     * @brief This function set the red value of color
     * @details This function set the red value of color (range: 0~255)
     * @param[in] r red value
     */
    void setRed(const BDByte& r);

    /**
     * @brief This function set the green value of color
     * @details This function set the green value of color (range: 0~255)
     * @param[in] g green value
     */
    void setGreen(const BDByte& g);

    /**
     * @brief This function set the blue value of color
     * @details This function set the blue value of color (range: 0~255)
     * @param[in] b blue value
     */
    void setBlue(const BDByte& b);

    /*** @retval BDPoint Reference of this class
     * @brief This function set the alpha value of color
     * @details TThis function set the alpha value of color (range: 0~255)
     * @param[in] a alpha value
     */
    void setAlpha(const BDByte& a);

    /**
     * @brief This function get the color value
     * @details This function set the red value of color (range: 0~255)
     * @param[in] r set red value* @retval BDPoint Reference of this class
     * @param[in] g set green value
     * @param[in] b set blue value
     * @param[in] a set alpha value
     */
    void get(BDByte* r, BDByte* g, BDByte* b, BDByte* a) const;

    /**
     * @brief This function get the red value of color
     * @details TThis function get the red value of color
     * @retval BDByte red value (range: 0~255)
     */
    BDByte getRed() const;

    /**
     * @brief This function get the alpha value of color
     * @details TThis function get the green value of color
     * @retval BDByte green value (range: 0~255)
     */
    BDByte getGreen() const;

    /**
     * @brief This function get the alpha value of color
     * @details TThis function get the blue value of color
     * @retval BDByte blue value (range: 0~255)
     */
    BDByte getBlue() const;

    /**
     * @brief This function get the alpha value of color
     * @details TThis function get the alpha value of color
     * @retval BDByte alpha value (range: 0~255)
     */
    BDByte getAlpha() const;

private:
    class BDColorImpl;
    BDColorImpl* m_pImpl;
};

} // common
} // mapauto
} // baidu

#endif // BAIDU_MAPAUTO_COMMON_BDCOLOR_H
