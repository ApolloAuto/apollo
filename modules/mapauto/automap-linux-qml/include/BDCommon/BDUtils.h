/**
 * @file BDUtils.h
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

#ifndef BAIDU_MAPAUTO_BDUTILS_H
#define BAIDU_MAPAUTO_BDUTILS_H
namespace baidu {
namespace mapauto {
namespace common {
/**
* @brief The BDMapUtil class
* @details This class provides util tools
* @author panyu(panyu02@baidu.com)
*/
class BDUtils
{
public:
    /**
     * @brief Transformation String  to Integer
     * @param[in] String
     * @retval  Integer value
     */
    static int stringToInt(std::string str);

    /**
     * @brief Locate current position
     * @param[out] longitude latitude
     */
    static void locate(double& longitude, double& latitude);

    /**
     * @brief Transformation Coordinate wgs84 to gcj02
     * @param[in&out] longitude, latitude
     */
    static void wgs84ToGcj02(double& longitude, double& latitude);

    /**
     * @brief Transformation angle to Arc
     * @param[in] angle
     * @retval Arc
     */
    static double angle2Arc(double angle);

    /**
     * @brief Measuring distance between wo points
     * @param[in] longitude1 latitude1: point1 ; longitude2 latitude2:point2
     * @retval distance
     */
    static double ll2Distance(double longitude1, double latitude1, double longitude2, double latitude2);

    /**
     * @brief Measuring bearing between wo points
     * @param[in] longitude1 latitude1: point1 ; longitude2 latitude2:point2
     * @retval bearing of two points
     */
    static double ll2Bearing(double longitude1, double latitude1, double longitude2, double latitude2);

    /**
     * @brief Measuring distance between point and current position
     * @param[in] longitude latitude: point1
     * @retval distance
     */
    static double distanceFromHere(double longitude, double latitude);

};


} // common
} // mapauto
} // baidu
#endif
