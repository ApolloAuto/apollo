/**
 * @file BDPOISearchFilter.h
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

#ifndef BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCHFILTER_H
#define BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCHFILTER_H

#include <BDNaviTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {
namespace search {

enum class BDPOISearchType
{
    INTEGRATE,
    POI,
    ADDRESS,
};

/**
 * @brief Filter of POISearch
 * @details Every options have to be set before call search(). After search finished, it doesn't influence on result.
 * @author Hansen (bianheshan@baidu.com)
 * @todo just few functions are declared now
 */
class BDPOISearchFilter {
public:
    BDPOISearchFilter();
    BDPOISearchFilter(const BDPOISearchFilter &filter);
    BDPOISearchFilter& operator= (const BDPOISearchFilter &filter);
    ~BDPOISearchFilter();


    /**
     * @brief set SearchType
     * @details Unknown/Address/Theme
     * @param[in] type it determines how to interpret keyword.
     * @retval NONE
     * @sa BDSearchType, setKeyword
     * @todo use enum class BDSearchType instead of HUInt
     */
    void setSearchType(const BDPOISearchType& type);

    /**
     * @brief get SearchType
     * @details Get current searchType of searchFilter
     * @retval HPOISearchType it determines how to interpret keyword.
     * @sa setSearchType
     * @todo use enum class BDSearchType instead of HUInt
     */
    BDPOISearchType getSearchType() const;

    /**
     * @brief set keyword[target] of search filter
     * @details setter
     * @param[in] strKeyword Keyword is wide string type, it can be sequence of words, consonants or just sequence of numbers. The keyword will be interpreted as set SearchType
     * @retval NONE
     * @sa getKeyword
     * @todo Regular expression feature
     */
    void setKeyword(const std::string& strKeyword);

    /**
     * @brief get keyword[target] of search filter
     * @details getter
     * @retval keyword default value is L""
     * @sa setKeyword
     * @todo Regular expression feature
     */
    std::string& getKeyword() const;

    /**
     * @brief set Exact match mode TRUE/FALSE
     * @details Determine whether search only exactly matched keywords or include similar keywords
     * @param[in] bOn, true : only search keyword exactly matched, default value is true
     * @retval NONE
     * @sa getExactMatchMode
     * @todo Not implemented yet
     */
    void setExactMatchMode(const BDBool& bOn);

    /**
     * @brief get Exact match mode
     * @details when searching, Determine whether search only exactly matched keywords or include similar keywords
     * @retval TRUE exact match mode is on
     * @retval FALSE exact match mode is off
     * @sa setExactMatchMode
     * @todo Not implemented yet
     */
    BDBool isExactMatchMode() const;

    /**
     * @brief set Address code, only RegionCode is OK
     * @details set search bound as the address
     * @param[in] address address class
     * @retval NONE
     * @sa getAddressCode
     * @todo Not implemented yet
     */
    void setAddress(const baidu::mapauto::common::BDAddress& address);

    /**
     * @brief get Address code
     * @details get address boundary
     * @retval address
     * @sa setAddressCode
     * @todo Not implemented yet
     */
    baidu::mapauto::common::BDAddress& getAddress() const;

    /**
     * @brief Set sort type, only support sort by distance, and you must setBoundary first
     * @details Results are sorted by NONE,DISTANCE in ascending order.
     * @param[in] eType results are sorted by this type
     * @retval NONE
     * @sa getSortType
     * @todo Not implemented yet
     */
    void setSortType(const BDSortType& eType);

    /**
     * @brief Get sort type
     * @details Results are sorted by NONE,DISTANCE in ascending order.
     * @retval BDSortType see BDSortType
     * @sa setSortType
     * @todo Not implemented yet
     */
    BDSortType getSortType() const;

    /**
     * @brief Set Datum Position
     * @details when sortType is DISTANCE, distance will be measured between pos and position of POI
     * @param[in] coord default value is position of the car
     * @retval NONE
     * @sa setSortType, getSortDatumPositon
     */
    void setSortDatumPosition(const common::BDGeoCoord& coord);

    /**
     * @brief Get Datum Position
     * @details when sortType is DISTANCE, distance will be measured between pos and position of POI
     * @retval reference GeoCoord
     * @sa setSortType, setSortDatumPositon
     */
    common::BDGeoCoord& getSortDatumPosition() const;

    /**
     * @brief set boundary circle
     * @details only search POI located in boundary
     * @param[in] coord center of circle, position
     * @param[in] radius radius of circle
     * @retval NONE
     */
    void setBoundary(const common::BDGeoCoord& coord, const BDUInt32& radius);

    /**
     * @brief get boundary circle
     * @details only search POI located in boundary
     * @param[out] pCoord center of circle, position
     * @param[out] pRadius radius of circle
     * @retval NONE
     */
    void getBoundary(common::BDGeoCoord* pCoord, BDUInt32* pRadius) const;

    /**
     * @brief get available option key list.
     * @param[out] optionKeyList Theme/Class/?
     */
    BDResult getOptionKeyList(const std::vector<std::string> keyList) const;

    /**
     * @brief get option paired with the key
     * @param[out] property, property is also variant type(consist of key-value pairs). even if value is not set, specify key-empty value for user to know key list.
     */
    BDResult getOption(const std::string key, common::BDVariant& property) const;

    /**
     * @brief set option with key.
     * @param[out] property only value with specified key is update. default value and valid range must be provided with document.
     */
    BDResult setOption(const std::string& key, const common::BDVariant& property);

    BDResult getPageOption(BDUInt32& curPage, BDUInt32& pageCount) const;

    BDResult setPageOption(BDUInt32 curPage, BDUInt32 pageCount);

private:
    class Impl;
    Impl* m_pImpl;
};

} // namespcae search
} // namespcae navi
} // namespcae mapauto
} // namespace baidu

#endif // BAIDU_MAPAUTO_NAVI_SEARCH_BDPOISEARCHFILTER_H
