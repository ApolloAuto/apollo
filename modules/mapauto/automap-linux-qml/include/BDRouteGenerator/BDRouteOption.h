/**
 * @file BDRouteOption.h
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

#ifndef BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEOPTION_H
#define BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEOPTION_H

#include <BDRouteTypes.h>

namespace baidu{
namespace mapauto {
namespace navi {
namespace route {

/**
 * @brief This class is the data class that handles the option when route generating
 * @details This option must be set in order to perform route generation
 * @author Hansen (bianheshan@baidu.com)
 */
class BDRouteOption {
public:
    /**
     * @brif This is the constructor
     * //@param[in] routeType BDRouteType must be set when create BDRouteOption
     */
    BDRouteOption();

    /**
     * @brief The destructor must be called when the route option is not needed anymore
     */
    ~BDRouteOption();

    /**
     * @brief This function is to probihit shallow copy
     * @details The override copy constructor
     * @param[in] The source object to be copied
     */
    BDRouteOption(const BDRouteOption& object);

    /**
     * @brief This function is to override the assignment operator
     * @param[in] The source object to be assigned
     */
    BDRouteOption& operator=(const BDRouteOption& object);

    /**
     * @brief This function is to override == operator
     * @param[in] object right side handler
     * @retval TRUE if two objects's are same
     * @retval FALSE if two objects' are different
     */
    BDBool operator==(const BDRouteOption& object) const;

    /**
     * @brief This function is to change the route type when was set in constructor
     * @param[in] routeType The route type that will be used for generating route
     */
    void setRouteType(const BDRouteType& routeType);

    /**
     * @brief This function is to get the route type which is set
     * @retval BDRouteType The type for generating route such as fastest, shortest, etc
     */
    BDRouteType getRouteType() const;

    /**
     * @brief This function is to set online mode
     * @details The default setting for onlineMode is the ON
     * @param[in] mode TRUE if online mode needed
                       FALSE embedded mode
     */
    void enableOnlineMode(const BDBool& mode);

    /**
     * @brief This function is to check online mode is set or not
     * @retval TRUE online mode for generating route
     * @retval FALSE embedded mode for generating route
     */
    BDBool isOnlineMode() const;

    /**
     * @brief This function is to get the list of additional options
     * @param[out] attributeList The available option list for current platform
     * @retval BDResult::OK The operation is done successfully
     * @retval BDResult::UNAVAILABLE The engine is not available
     * @sa getOption, setOption
     * @code
        DoorToDoor       : consider direction of destination (Korea, Europe - default 1: Consider)
        Highway          : use highway (Korea, NA, China, Europe - default 1: use highway)
        TollRoad         : use toll road (Korea, NA, China, Europe, Middle East - default 1: use toll road)
        TollPass         : use toll pass such as High-pass in korea (Korea, NA - default 1: use toll pass)
        RealtimeTraffic  : use realtime traffic information (Korea, NA, Europe - default 1: use real time traffic)
        TimeRestricted   : consider restricted time for area (Korea, NA, Europe, Middle East - default 1: consider)
        Ferry            : consider ferry route (Korea, NA, China, Europe, Middle East - default 0: do not care)
        PrivateZone      : avoid private zone (NA - default 1: avoid private zone)
        RTM              : consider accident information (Korea - default 1: consider)
        Tunnel           : include tunnel (Europe, Middle East - default 1: include tunnel)
     * @endcode
     */
    BDResult getOptionList(std::vector<std::string>& optionList) const;

    /**
     * @brief This function is to set the additional option for route generation
     * @details The possible options are differnt depending on the country.
     * @param[in] optionType The type of option which can be obtained by optionList
     * @param[out] value The value of the attribute that will be set
     * @sa getOptionList, getOption
     */
    void setOption(const std::string& optionType, const BDUInt32& value);

    /**
     * @brief This function is to get the value of option
     * @param[in] optionType The type of the option
     * @retval BDInt32 The value of option
     * @sa setOption getOptionList
     */
    BDUInt32 getOption(const std::string& optionType) const;

private:
    class BDRouteOptionImpl;
    BDRouteOptionImpl* m_pImpl;
};

} // namespcae route
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_ROUTE_BDROUTEOPTION_H
