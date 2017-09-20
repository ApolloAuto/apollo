/**
 * @file BDNaviTypes.h
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

#ifndef BAIDU_MAPAUTO_NAVI_BDNAVITYPES_H
#define BAIDU_MAPAUTO_NAVI_BDNAVITYPES_H

#include <BDCommonTypes.h>

namespace baidu {
namespace mapauto {
namespace navi {

enum class BDSearchType {
    NAME = 0x1,
    TEL = 0x2,
    ADDRESS = 0x4,
    CLASS = 0x8,
    MAX
};

enum class BDAddressType {
    KR_OLD,
    KR_NEW,
    US,
    CN,
    MAX
};

enum class BDSortType {
    NONE,
    DISTANCE,
//    DOCID,
//    ADDRESSCODE,
//    TIME,
//    PRIORITY,
//    OILPRICE,
//    NAME,
//    TITLE,
//    CLASS,
    MAX
};

enum class BDRoadType {
    HIGHWAY,
    CITYFASTWAY,
    NATIONWAY,
    PROVINCEWAY,
    COUNTYWAY,
    TOWNWAY,
    OTHERWAY,
    LEVEL9WAY,
    FERRYWAY,
    WALKWAY,
    INVALID,
    MAX,
};

} // namespace navi
} // namespace mapauto
} //baidu

#include <BDGeoArea.h>
#include <BDAddress.h>

#endif // BAIDU_MAPAUTO_NAVI_BDNAVITYPES_H
