// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// INTEL CONFIDENTIAL
//
// Copyright (c) 2018 Intel Corporation
//
// This software and the related documents are Intel copyrighted materials, and
// your use of them is governed by the express license under which they were
// provided to you (License). Unless the License provides otherwise, you may not
// use, modify, copy, publish, distribute, disclose or transmit this software or
// the related documents without Intel's prior written permission.
//
// This software and the related documents are provided as is, with no express or
// implied warranties, other than those that are expressly stated in the License.
//
// ----------------- END LICENSE BLOCK -----------------------------------
/**
 * @file
 */
#pragma once

// The helper function "std::make_unique()" is defined since C++14.
// The definition of "std::make_unique()" borrowed from C++14 is given here
// so that it can be used in C++11.
#if __cplusplus == 201103L
namespace std {

template <typename _Tp>
struct _MakeUniq {
  typedef unique_ptr<_Tp> __single_object;
};

template <typename _Tp>
struct _MakeUniq<_Tp[]> {
  typedef unique_ptr<_Tp[]> __array;
};

template <typename _Tp, size_t _Bound>
struct _MakeUniq<_Tp[_Bound]> {
  struct __invalid_type {}; 
};

// std::make_unique for single objects
template <typename _Tp, typename... _Args>
inline typename _MakeUniq<_Tp>::__single_object make_unique(_Args&&... __args) {
  return unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...));
}

// Alias template for remove_extent
template <typename _Tp>
using remove_extent_t = typename remove_extent<_Tp>::type;

// std::make_unique for arrays of unknown bound
template <typename _Tp>
inline typename _MakeUniq<_Tp>::__array make_unique(size_t __num) {
  return unique_ptr<_Tp>(new remove_extent_t<_Tp>[__num]());
}

// Disable std::make_unique for arrays of known bound
template <typename _Tp, typename... _Args>
inline typename _MakeUniq<_Tp>::__invalid_type make_unique(_Args&&...) = delete;
}  // namespace std
#endif

