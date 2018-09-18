/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Tully Foote */

#ifndef TF2_TRANSFORM_DATATYPES_H
#define TF2_TRANSFORM_DATATYPES_H

#include <string>
#include "ros/time.h"

namespace tf2
{

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf2 datatype equivilant of a MessageStamped */
template <typename T>
class Stamped : public T{
 public:
  ros::Time stamp_; ///< The timestamp associated with this data
  std::string frame_id_; ///< The frame_id associated this data

  /** Default constructor */
  Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){}; //Default constructor used only for preallocation

  /** Full constructor */
  Stamped(const T& input, const ros::Time& timestamp, const std::string & frame_id) :
    T (input), stamp_ ( timestamp ), frame_id_ (frame_id){ } ;
  
  /** Copy Constructor */
  Stamped(const Stamped<T>& s):
    T (s),
    stamp_(s.stamp_),
    frame_id_(s.frame_id_) {}
  
  /** Set the data element */
  void setData(const T& input){*static_cast<T*>(this) = input;};
};

/** \brief Comparison Operator for Stamped datatypes */
template <typename T> 
bool operator==(const Stamped<T> &a, const Stamped<T> &b) {
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ && static_cast<const T&>(a) == static_cast<const T&>(b);
};


}
#endif //TF2_TRANSFORM_DATATYPES_H
