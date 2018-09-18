/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TF2_IMPL_CONVERT_H
#define TF2_IMPL_CONVERT_H

namespace tf2 {
namespace impl {

template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter {
public:
  template<typename A, typename B>
  static void convert(const A& a, B& b);
};

// The case where both A and B are messages should not happen: if you have two
// messages that are interchangeable, well, that's against the ROS purpose:
// only use one type. Worst comes to worst, specialize the original convert
// function for your types.
// if B == A, the templated version of convert with only one argument will be
// used.
//
//template <>
//template <typename A, typename B>
//inline void Converter<true, true>::convert(const A& a, B& b);

template <>
template <typename A, typename B>
inline void Converter<true, false>::convert(const A& a, B& b)
{
  fromMsg(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, true>::convert(const A& a, B& b)
{
  b = toMsg(a);
}

template <>
template <typename A, typename B>
inline void Converter<false, false>::convert(const A& a, B& b)
{
  fromMsg(toMsg(a), b);
}

}
}

#endif //TF2_IMPL_CONVERT_H
