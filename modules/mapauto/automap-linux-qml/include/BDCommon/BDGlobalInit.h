/**
 * @file HGlobalInit.h
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

#ifndef BAIDU_MAPAUTO_GLOBAL_INIT_H
#define BAIDU_MAPAUTO_GLOBAL_INIT_H
namespace baidu {
namespace mapauto {
namespace common {
/**
 * @brief BDMapAuto global init interface.
 * @param cfg_path[in] Resource path
 * @param uuid[in]  Device unique identification
 */
void BDGlobalInit(const std::string &cfg_path, const std::string &uuid);
} // common
} // mapauto
} // baidu
#endif
