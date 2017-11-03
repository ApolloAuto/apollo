#ifndef MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_H_
#define MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_H_

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"

namespace apollo {
namespace perception {

class SharedData {
 public:
  SharedData() {}
  virtual ~SharedData() {}

  virtual bool Init() = 0;

  // this api should clear all the memory used,
  // and would be called by SharedDataManager when reset DAGStreaming.
  virtual void Reset() {
    CHECK(false) << "reset() not implemented.";
  }

  virtual void RemoveStaleData() {
    CHECK(false) << "remove_stale_data() not implemented.";
  }

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(SharedData);
};

REGISTER_REGISTERER(SharedData);

#define REGISTER_SHAREDDATA(name) REGISTER_CLASS(SharedData, name)

}  // namespace perception
}  // namespace apollo
#endif  // MODEULES_PERCEPTION_ONBOARD_SHARED_DATA_H_
