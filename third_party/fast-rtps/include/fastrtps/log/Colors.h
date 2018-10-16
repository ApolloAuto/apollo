// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file Colors.h
 */

#ifndef _FASTRTPS_LOG_COLORS_H_
#define _FASTRTPS_LOG_COLORS_H_


#if defined(_WIN32)
	#define C_DEF ""
	#define C_RED ""
	#define C_B_RED ""
	#define C_GREEN ""
	#define C_B_GREEN ""
	#define C_YELLOW ""
	#define C_B_YELLOW ""
	#define C_BLUE ""
	#define C_B_BLUE ""
	#define C_MAGENTA ""
	#define C_B_MAGENTA ""
	#define C_CYAN ""
	#define C_B_CYAN ""
	#define C_WHITE ""
	#define C_B_WHITE ""
	#define C_BRIGHT ""
#else
	#define C_DEF "\033[m"
	#define C_RED "\033[31m"
	#define C_B_RED "\033[31;1m"
	#define C_GREEN "\033[32m"
	#define C_B_GREEN "\033[32;1m"
	#define C_YELLOW "\033[33m"
	#define C_B_YELLOW "\033[33;1m"
	#define C_BLUE "\033[34m"
	#define C_B_BLUE "\033[34;1m"
	#define C_MAGENTA "\033[35m"
	#define C_B_MAGENTA "\033[35;1m"
	#define C_CYAN "\033[36m"
	#define C_B_CYAN "\033[36;1m"
	#define C_WHITE "\033[37m"
	#define C_B_WHITE "\033[37;1m"
	#define C_BRIGHT "\033[1m"
#endif


#endif /* _FASTRTPS_LOG_COLORS_H_ */
