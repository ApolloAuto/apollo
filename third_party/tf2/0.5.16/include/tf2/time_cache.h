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

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include "transform_storage.h"

#include <list>

#include <sstream>

#include <ros/message_forward.h>
#include <ros/time.h>

#include <boost/shared_ptr.hpp>

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(TransformStamped);
}

namespace tf2
{

typedef std::pair<ros::Time, CompactFrameID> P_TimeAndFrameID;

class TimeCacheInterface
{
public:
  /** \brief Access data from the cache */
  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0)=0; //returns false if data unavailable (should be thrown as lookup exception

  /** \brief Insert data into the cache */
  virtual bool insertData(const TransformStorage& new_data)=0;

  /** @brief Clear the list of stored values */
  virtual void clearList()=0;

  /** \brief Retrieve the parent at a specific time */
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str) = 0;

  /**
   * \brief Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
   */
  virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;


  /// Debugging information methods
  /** @brief Get the length of the stored list */
  virtual unsigned int getListLength()=0;

  /** @brief Get the latest timestamp cached */
  virtual ros::Time getLatestTimestamp()=0;

  /** @brief Get the oldest timestamp cached */
  virtual ros::Time getOldestTimestamp()=0;
};

typedef boost::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache : public TimeCacheInterface
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache(ros::Duration  max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));


  /// Virtual methods

  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0);
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual ros::Time getLatestTimestamp();
  virtual ros::Time getOldestTimestamp();
  

private:
  typedef std::list<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  ros::Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two, ros::Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, ros::Time time, TransformStorage& output);


  void pruneList();



};

class StaticCache : public TimeCacheInterface
{
 public:
  /// Virtual methods

  virtual bool getData(ros::Time time, TransformStorage & data_out, std::string* error_str = 0); //returns false if data unavailable (should be thrown as lookup exception
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(ros::Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();


  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual ros::Time getLatestTimestamp();
  virtual ros::Time getOldestTimestamp();
  

private:
  TransformStorage  storage_;
};

}

#endif // TF2_TIME_CACHE_H
