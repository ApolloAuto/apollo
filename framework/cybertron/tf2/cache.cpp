
#include <assert.h>
#include <iostream>

#include "cybertron/common/log.h"
#include "cybertron/tf2/time_cache.h"
#include "cybertron/tf2/exceptions.h"
#include "cybertron/tf2/LinearMath/Vector3.h"
#include "cybertron/tf2/LinearMath/Quaternion.h"
#include "cybertron/tf2/LinearMath/Transform.h"
#include "cybertron/proto/common_geometry.pb.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

TransformStorage::TransformStorage() {}

TransformStorage::TransformStorage(const adu::common::TransformStamped& data,
                                   CompactFrameID frame_id,
                                   CompactFrameID child_frame_id)
    : stamp_(data.header().stamp()),
      frame_id_(frame_id),
      child_frame_id_(child_frame_id) {
  const adu::common::Quaternion& o = data.transform().rotation();
  rotation_ = tf2::Quaternion(o.qx(), o.qy(), o.qz(), o.qw());
  const adu::common::Vector3& v = data.transform().translation();
  translation_ = tf2::Vector3(v.x(), v.y(), v.z());
}

TimeCache::TimeCache(cybertron::Duration max_storage_time)
    : max_storage_time_(max_storage_time) {}

// hoisting these into separate functions causes an ~8% speedup.  Removing
// calling them altogether adds another ~10%
void createExtrapolationException1(cybertron::Time t0, cybertron::Time t1,
                                   std::string* error_str) {
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation at time " << t0.ToNanosecond()
       << ", but only time " << t1.ToNanosecond() << " is in the buffer";
    *error_str = ss.str();
  }
}

void createExtrapolationException2(cybertron::Time t0, cybertron::Time t1,
                                   std::string* error_str) {
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the future.  Requested time "
       << t0.ToNanosecond() << " but the latest data is at time "
       << t1.ToNanosecond();
    *error_str = ss.str();
  }
}

void createExtrapolationException3(cybertron::Time t0, cybertron::Time t1,
                                   std::string* error_str) {
  if (error_str) {
    std::stringstream ss;
    ss << "Lookup would require extrapolation into the past.  Requested time "
       << t0.ToNanosecond() << " but the earliest data is at time "
       << t1.ToNanosecond();
    *error_str = ss.str();
  }
}

uint8_t TimeCache::findClosest(TransformStorage*& one, TransformStorage*& two,
                               cybertron::Time target_time,
                               std::string* error_str) {

  // No values stored
  if (storage_.empty()) {
    return 0;
  }

  // If time == 0 return the latest
  if (target_time.IsZero()) {
    one = &storage_.front();
    return 1;
  }
  // One value stored
  if (++storage_.begin() == storage_.end()) {
    TransformStorage& ts = *storage_.begin();
    if (ts.stamp_ == target_time) {
      one = &ts;
      return 1;
    } else {
      createExtrapolationException1(target_time, ts.stamp_, error_str);
      AWARN << " error:" << *error_str;
      return 0;
    }
  }
  cybertron::Time latest_time = (*storage_.begin()).stamp_;
  cybertron::Time earliest_time = (*(storage_.rbegin())).stamp_;

  if (target_time == latest_time) {
    one = &(*storage_.begin());
    return 1;
  } else if (target_time == earliest_time) {
    one = &(*storage_.rbegin());
    return 1;
  }
  // Catch cases that would require extrapolation
  else if (target_time > latest_time) {
    createExtrapolationException2(target_time, latest_time, error_str);
    AWARN << " error:" << *error_str;
    return 0;
  } else if (target_time < earliest_time) {
    createExtrapolationException3(target_time, earliest_time, error_str);
    AWARN << " error:" << *error_str;
    return 0;
  }

  // At least 2 values stored
  // Find the first value less than the target value
  L_TransformStorage::iterator storage_it = storage_.begin();
  while (storage_it != storage_.end()) {
    if (storage_it->stamp_ <= target_time) break;
    ++storage_it;
  }

  // Finally the case were somewhere in the middle  Guarenteed no extrapolation
  // :-)
  one = &*(storage_it);    // Older
  two = &*(--storage_it);  // Newer
  return 2;
}

void TimeCache::Interpolate(const TransformStorage& one,
                            const TransformStorage& two, cybertron::Time time,
                            TransformStorage& output) {
  // Check for zero distance case
  if (two.stamp_ == one.stamp_) {
    output = two;
    return;
  }
  // Calculate the ratio
  tf2Scalar ratio = (time.ToSecond() - one.stamp_.ToSecond()) /
                    (two.stamp_.ToSecond() - one.stamp_.ToSecond());

  // Interpolate translation
  output.translation_.setInterpolate3(one.translation_, two.translation_,
                                      ratio);

  // Interpolate rotation
  output.rotation_ = slerp(one.rotation_, two.rotation_, ratio);

  output.stamp_ = one.stamp_;
  output.frame_id_ = one.frame_id_;
  output.child_frame_id_ = one.child_frame_id_;
}

bool TimeCache::getData(
    cybertron::Time time, TransformStorage& data_out,
    std::string* error_str)  // returns false if data not available
{
  TransformStorage* p_temp_1;
  TransformStorage* p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  // if(num_nodes == 2){
  //   LOG_DEBUG_FORMAT("p_temp_1 time:%f,
  // x:%f,y:%f,z:%f,qx:%f,qy:%f,qz:%f,qw:%f",
  //     p_temp_1->stamp_.ToSecond(),
  //     p_temp_1->translation_.x(),
  //     p_temp_1->translation_.y(),
  //     p_temp_1->translation_.z(),
  //     p_temp_1->rotation_.x(),
  //     p_temp_1->rotation_.y(),
  //     p_temp_1->rotation_.z(),
  //     p_temp_1->rotation_.w());
  //   LOG_DEBUG_FORMAT("p_temp_2
  // time:%f,x:%f,y:%f,z:%f,qx:%f,qy:%f,qz:%f,qw:%f",
  //     p_temp_2->stamp_.ToSecond(),
  //     p_temp_2->translation_.x(),
  //     p_temp_2->translation_.y(),
  //     p_temp_2->translation_.z(),
  //     p_temp_2->rotation_.x(),
  //     p_temp_2->rotation_.y(),
  //     p_temp_2->rotation_.z(),
  //     p_temp_2->rotation_.w());
  // }
  if (num_nodes == 0) {
    return false;
  } else if (num_nodes == 1) {
    data_out = *p_temp_1;
  } else if (num_nodes == 2) {
    if (p_temp_1->frame_id_ == p_temp_2->frame_id_) {
      Interpolate(*p_temp_1, *p_temp_2, time, data_out);
    } else {
      data_out = *p_temp_1;
    }
  } else {
    assert(0);
  }

  return true;
}

CompactFrameID TimeCache::getParent(cybertron::Time time,
                                    std::string* error_str) {
  TransformStorage* p_temp_1;
  TransformStorage* p_temp_2;

  int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0) {
    return 0;
  }

  return p_temp_1->frame_id_;
}

bool TimeCache::insertData(const TransformStorage& new_data) {
//  LOG_DEBUG_FORMAT("tf2 cache list size: %d", storage_.size());
//  LOG_DEBUG_FORMAT("p_temp_new time:%f, x:%f,y:%f,z:%f,qx:%f,qy:%f,qz:%f,qw:%f,frm:%d,child_frm:%d",
//                   new_data.stamp_.ToSecond(), new_data.translation_.x(),
//                   new_data.translation_.y(), new_data.translation_.z(),
//                   new_data.rotation_.x(), new_data.rotation_.y(),
//                   new_data.rotation_.z(), new_data.rotation_.w(),
//                   new_data.frame_id_, new_data.child_frame_id_);

  L_TransformStorage::iterator storage_it = storage_.begin();

  if (storage_it != storage_.end()) {
    if (storage_it->stamp_ > new_data.stamp_ + max_storage_time_) {
      return false;
    }
  }

  while (storage_it != storage_.end()) {
    if (storage_it->stamp_ < new_data.stamp_) {
      break;
    } else if ( storage_it->stamp_ == new_data.stamp_
        && storage_it->frame_id_ == new_data.frame_id_
        && storage_it->child_frame_id_ == new_data.child_frame_id_) {
      // both the stamp and frame_ids are the same, so just update the tansform data
      // and there is no need to pruneList because the stamps set in the list doesn't change
      storage_it = storage_.erase(storage_it);
      storage_.insert(storage_it, new_data);
      return true;
    }
    ++storage_it;
  }
  storage_.insert(storage_it, new_data);

  pruneList();
  return true;
}

void TimeCache::clearList() { storage_.clear(); }

unsigned int TimeCache::getListLength() { return storage_.size(); }

P_TimeAndFrameID TimeCache::getLatestTimeAndParent() {
  if (storage_.empty()) {
    return std::make_pair(cybertron::Time(), 0);
  }

  const TransformStorage& ts = storage_.front();
  return std::make_pair(ts.stamp_, ts.frame_id_);
}

cybertron::Time TimeCache::getLatestTimestamp() {
  if (storage_.empty()) return cybertron::Time();  // empty list case
  return storage_.front().stamp_;
}

cybertron::Time TimeCache::getOldestTimestamp() {
  if (storage_.empty()) return cybertron::Time();  // empty list case
  return storage_.back().stamp_;
}

void TimeCache::pruneList() {
  cybertron::Time latest_time = storage_.begin()->stamp_;

  while (!storage_.empty() &&
         storage_.back().stamp_ + max_storage_time_ < latest_time) {
    storage_.pop_back();
  }
}

}
}
}
