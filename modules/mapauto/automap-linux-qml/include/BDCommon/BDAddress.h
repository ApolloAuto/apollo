/**
 * @file BDAddress.h
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

#ifndef BAIDU_MAPAUTO_NAVI_BDADDRESS_H
#define BAIDU_MAPAUTO_NAVI_BDADDRESS_H

#include <BDNaviTypes.h>
namespace baidu {
namespace mapauto {
namespace common {

/**
 * @brief Address in navigation is diffrent with address in website. it's just has enough information to navigate.
 */
class BDAddress {
public:
    BDAddress();
    ~BDAddress();

    /**
     * @brief copy constructor
     */
    BDAddress (const BDAddress& other);

    /**
     * @brief assign operator
     */
    BDAddress& operator= (const BDAddress& other);

    /**
     * @brief get Region, region is consist of levels. must specify proper keyword in each level.
     * @sa HAddressSearch::getRegionLeveList
     */
    std::vector<std::string>& getRegion() const;

    /**
     * @brief set Region. 
     */
    void setRegion(const std::vector<std::string>& region);

    /**
     * @brief get region code.
     * @TODO check all of code is bit flags 123(Large)/456(Middle)/789(Small)
     */
    BDUInt32 getRegionCode() const;

    /**
     * @brief set region code
     */
    void setRegionCode(const BDUInt32& code);

    /**
     * @brief Get zipcode. zipCode is matched with specific region. it is optional because it doesn't matter in navigation.
     */
    std::string& getZipCode() const;

    /**
     * @brief set Zipcode.
     */
    void setZipCode(const std::string& zipCode);

    /**
     * @brief Building number is unique in street level.
     */
    std::string& getBuildingNumber() const;

    /**
     * @brief set Building number 19-1
     */
    void setBuildingNumber(const std::string& number);

    std::string getFullAddress() const;

private:
    class Impl;
    Impl* m_pImpl;
};

} // namespace common
} // namespace mapauto
} // namespace baidu
#endif // BAIDU_MAPAUTO_NAVI_BDADDRESS_H
