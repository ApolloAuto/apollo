/**
 * @file BDPOIInfo.h
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

#ifndef BAIDU_MAPAUTO_NAVI_SEARCH_BDPOIINFO_H
#define BAIDU_MAPAUTO_NAVI_SEARCH_BDPOIINFO_H

#include <BDNaviTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace search {

class BDSubPOIInfo {
public:
    std::string						        uid;	                /* POI点的服务端uid */
    common::BDGeoCoord			point;		        /**< 显示坐标 */
    std::string		                        name;             /**< POI名称 */
    std::string		                        alisename;		/**< POI别名 */
};

/**
 * @brief BDPOIInfo. One instance means one POI
 * @details It has only getter/setter to get/set property(name, position)
 * @author Baidu MapAuto Team
 * @todo just few functions are declared now, use pimpl and hide private member variables
 */
class BDPOIInfo {
public:
    BDPOIInfo();
    ~BDPOIInfo();

    BDPOIInfo(const BDPOIInfo& other);

    BDPOIInfo& operator= (const BDPOIInfo& other);

    /**
     * @brief get uid property
     * @details get unique uid of POI
     * @retval unique uid of POI.
     */
    std::string getUid() const;

    /**
     * @brief set uid property
     * @details get unique uid of POI
     * @param[in] id unique uid of POI.
     */
    void setUid(std::string uid);

    /**
     * @brief get id property
     * @details get unique id of POI
     * @retval unique id of POI.
     * @sa setId
     */
    BDUInt64 getId() const;

    /**
     * @brief set id property
     * @details get unique id of POI
     * @param[in] id unique id of POI.
     * @sa getId
     */
    void setId(const BDUInt64 id);

    /**
     * @brief get Name property
     * @details get name of POI
     * @retval Name of POI.
     * @sa setName
     */
    std::string& getName() const;

    /**
     * @brief set Name property
     * @details It is intendef not for users, buf for engines
     * @param[in] strName Name of POI
     * @retval NONE
     * @sa getName
     */
    void setName(const std::string& strName);

    /**
     * @brief get Position property
     * @details get position of POI
     * @retval position of POI. HLonLat class
     * @sa setPosition
     */
    common::BDGeoCoord& getPosition() const;

    /**
     * @brief set Position property
     * @details It is intendef not for users, buf for engines
     * @param[in] coord position of POI. BDGeoCoord class
     * @retval NONE
     * @sa getPosition
     */
    void setPosition(const common::BDGeoCoord& coord);

    /**
     * @brief get phonenumber property
     */
    std::string& getPhoneNumber() const;

    /**
     * @brief set phonenumber property
     */
    BDResult setPhoneNumber(const std::string& phonenumber);

    /**
     * @brief get Address
     * @retval BDAddress
     * @sa setAddress
     */
    common::BDAddress& getAddress() const;

    /**
     * @brief set Address
     * @retval BDResult
     * @sa getAddress
     */
    BDResult setAddress(const common::BDAddress& address);

    /**
     * @brief set full option, it is only for engine
     */
    BDResult setFullOption(const std::string& fullOptionJsonString);

    /**
     * @brief get available option key list.
     * @param[out] optionKeyList Class, BusinessHour, Theme, Bus, Station
     */
    BDResult getOptionKeyList(std::vector<std::string>& keyList) const;

    /**
     * @brief get option paired with the key
     * @param[out] property, property is also variant type(consist of key-value pairs). even if value is not set, specify key-empty value for user to know key list.
     */
    BDResult getOption(const std::string& key, common::BDVariant& property) const;

    /**
     * @brief set option with key.
     * @param[out] property only value with specified key is update. default value and valid range must be provided with document.
     */
    BDResult setOption(const std::string& key, const common::BDVariant& property);

    /**
     * @brief get sub-pois.
     * @param[out] get sub-pois.
     */
    std::vector<BDSubPOIInfo> getSubPoiList() const;

    /**
     * @brief set sub-pois.
     * @param[in] set sub-pois.
     */
    BDResult setSubPoiList(BDSubPOIInfo subPoi);

private:
    class Impl;
    Impl* m_pImpl;

};

} // namespace search
} // namespace navi
} // namespace mapauto
}

#endif // BAIDU_MAPAUTO_NAVI_SEARCH_BDPOIINFO_H
