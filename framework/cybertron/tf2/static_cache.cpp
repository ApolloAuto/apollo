
#include "cybertron/tf2/time_cache.h"
#include "cybertron/tf2/exceptions.h"

#include "cybertron/tf2/LinearMath/Transform.h"

namespace apollo {
namespace cybertron {
namespace tf2 {

bool StaticCache::getData(
    cybertron::Time time, TransformStorage& data_out,
    std::string* error_str)  // returns false if data not available
{
  (void)error_str;
  data_out = storage_;
  data_out.stamp_ = time;
  return true;
};

bool StaticCache::insertData(const TransformStorage& new_data) {
  storage_ = new_data;
  return true;
};

void StaticCache::clearList() {
  return;
};

unsigned int StaticCache::getListLength() {
  return 1;
};

CompactFrameID StaticCache::getParent(cybertron::Time time,
                                      std::string* error_str) {
  (void)time;
  (void)error_str;
  return storage_.frame_id_;
}

P_TimeAndFrameID StaticCache::getLatestTimeAndParent() {
  return std::make_pair(cybertron::Time(), storage_.frame_id_);
}

cybertron::Time StaticCache::getLatestTimestamp() {
  return cybertron::Time();
};

cybertron::Time StaticCache::getOldestTimestamp() {
  return cybertron::Time();
};

}
}
}
