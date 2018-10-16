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

/*!
 * @file StringMatching.h
 *
 */

#ifndef STRINGMATCHING_H_
#define STRINGMATCHING_H_
#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC
namespace eprosima {
namespace fastrtps{
namespace rtps {
/**
 * Class StringMatching used to match different strings against each other as defined by the POSIX fnmatch API (1003.2-1992
section B.6).
 @ingroup UTILITIES_MODULE
 */
class StringMatching {
public:
	StringMatching();
	virtual ~StringMatching();
	/** Static method to match two strings.
	* It checks the string specified by the input argument to see if it matches the pattern specified by the pattern argument.
	*/
	static bool matchString(const char* pattern,const char* input);
	//FIXME: 	CONVERTIR EN INLINE
};
}
} /* namespace rtps */
} /* namespace eprosima */
#endif
#endif /* STRINGMATCHING_H_ */
