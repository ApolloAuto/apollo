/**
 * @file BDVariant.h
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


#ifndef BAIDU_MAPAUTO_COMMON_BDVARIANT_H
#define BAIDU_MAPAUTO_COMMON_BDVARIANT_H

#include <BDCommonTypes.h>

namespace baidu{
namespace mapauto {
namespace common {

/**
 * @brief The BDVariant class\n
 * @details The actual form of this class is the JSON
 * @author Hansen (bianheshan@baidu.com)
 */
class BDVariant {
public:

    /**
     * @brief default constructor
     */
    BDVariant();

    /**
     * @brief default destructor
     */
    virtual ~BDVariant();

    /**
     * @brief copy constructor
     */
    BDVariant(const BDVariant &);

    /**
     * @brief assignment operation
     * @param[in] reference of an BDVariant
     * @retval BDVariant reference of BDVariant
     */
    const BDVariant &operator = (const BDVariant &);

    /**
     * @brief This function stores a copy of std::string value. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the value to be setted
     * @param[in] value The value to be setted
     */
    BDResult setValue(const std::string& key, const std::string& value);

    /**
     * @brief This function stores a copy of BDInt32 value. A compile error will occur if Variant doesn't handle the type
     * @param[in] key   The key of the value to be setted
     * @param[in] value The value to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setValue(const std::string& key, const BDInt32& value);

    /**
     * @brief This function stores a copy of BDFloat value. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the value to be setted
     * @param[in] value The value to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setValue(const std::string& key, const BDFloat& value);

    /**
     * @brief This function stores a copy of vector containing std::string type values. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the vector to be setted
     * @param[in] vector The vector to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setVector(const std::string& key, const std::vector<std::string>& vector);

    /**
     * @brief This function stores a copy of vector containing BDInt32 type values. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the vector to be setted
     * @param[in] vector The vector to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setVector(const std::string& key, const std::vector<BDInt32>& vector);

    /**
     * @brief This function stores a copy of vector containing BDFloat type values. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the vector to be setted
     * @param[in] vector The vector to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setVector(const std::string& key, const std::vector<BDFloat>& vector);

    /**
     * @brief This function stores a copy of vector containing BDFloat type values. A compile error will occur if BDVariant doesn't handle the type
     * @param[in] key   The key of the variant to be setted
     * @param[in] variant The variant to be setted
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult setHVariant(const std::string& key, const BDVariant& variant);

    /**
     * @brief Returns std::string value related to the key.
     * @param[in] key   The key of the value to be returned
     * @param[out] value The value to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getValue(const std::string& key, std::string& value) const;

    /**
     * @brief Returns BDInt32 value related to the key.
     * @param[in] key   The key of the value to be returned
     * @param[out] value The value to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getValue(const std::string& key, BDInt32& value) const;

    /**
     * @brief Returns BDFloat value related to the key.
     * @param[in] key   The key of the value to be returned
     * @param[out] value The value to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getValue(const std::string& key, BDFloat& value) const;

    /**
     * @brief Returns std::vector<std::string> related to the key.
     * @param[in] key The key of the vector to be returned
     * @param[out] vector The vector to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getVector(const std::string& key, std::vector<std::string>& vector) const;

    /**
     * @brief Returns std::vector<BDInt32> related to the key.
     * @param[in] key The key of the vector to be returned
     * @param[out] vector The vector to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getVector(const std::string& key, std::vector<BDInt32>& vector) const;

    /**
     * @brief Returns std::vector<BDFloat> related to the key.
     * @param[in] key The key of the vector to be returned
     * @param[out] vector The vector to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getVector(const std::string& key, std::vector<BDFloat>& vector) const;

    /**
     * @brief Returns BDVariant related to the key.
     * @param[in] key The key of the variant to be returned
     * @param[out] variant The variant to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getHVariant(const std::string& key, BDVariant& variant) const;

    /**
     * @brief Returns serialized Json type std::string
     * @param[out] serializedJson The BDFloat value to be returned
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult toJson(std::string& serializedJson) const;

    /**
     * @brief Returns BDVariant containing all of the values and keys
     * @param[in] serializedJson The serialized json file that can be converted into BDVariant
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult toHVariant(const std::string& serializedJson) const;

    /**
     * @brief This function destroy all of key and value
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult clear();

    /**
     * @brief Returns existing keys in BDVariant
     * @param[out] keys existing keys in BDVariant
     * @retval BDResult::OK success
     * @retval BDResult::INVALID otherwise failed
     */
    BDResult getKeys(std::vector<std::string>& keys) const;

    /**
     * @brief Returns type related to key
     * @param[in] key The key which wants to know its type
     * @retval BDVarTypes The type related to the key
     */
    BDVarTypes getType(const std::string& key) const;

private:
    class BDVariantImpl;
    BDVariantImpl* m_pImpl;
};

} // common
} // mapauto
} // baidu

#endif // BAIDU_MAPAUTO_COMMON_BDVARIANT_H
