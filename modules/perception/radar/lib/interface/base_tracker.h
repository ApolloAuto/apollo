// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui (huiyujiang@baidu.com)
// @file: base_tracker.h
// @brief: radar tracker interface
#ifndef RADAR_LIB_INTERFACE_BASE_TRACKER_H_
#define RADAR_LIB_INTERFACE_BASE_TRACKER_H_
// SAMPLE CODE:
//
// class MyTracker : public BaseTracker {
// public:
//     MyTracker() : BaseTracker() {}
//     virtual ~MyTracker() {}
//
//     virtual bool Init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool Track(
//             const base::Frame& detected_frame,
//              const TrackerOptions& options,
//              base::FramePtr tracked_frame) override {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string Name() const override {
//          return "MyTracker";
//      }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_TRACKER(MyTracker);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseTracker* tracker =
//          BaseTrackerRegisterer::get_instance_by_name("MyTracker");
// using tracker to do somethings.
// ////////////////////////////////////////////////////
#include <string>
#include <vector>
#include "Eigen/Core"
#include "modules/perception/base/frame.h"
#include "cybertron/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace radar {
struct TrackerOptions {};
class BaseTracker {
 public:
  BaseTracker() : name_("BaseTracker") {}
  virtual ~BaseTracker() = default;
  virtual bool Init() = 0;
  // @brief: tracking objects.
  // @param [in]: current object frame.
  // @param [in]: options.
  // @param [out]: current tracked objects frame.
  virtual bool Track(const base::Frame &detected_frame,
                     const TrackerOptions &options,
                     base::FramePtr tracked_frame) = 0;
  virtual std::string Name() { return name_; }

 protected:
  std::string name_;

 private:
  BaseTracker(const BaseTracker &) = delete;
  BaseTracker &operator=(const BaseTracker &) = delete;
};
PERCEPTION_REGISTER_REGISTERER(BaseTracker);
#define PERCEPTION_REGISTER_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTracker, name)
}  // namespace radar
}  // namespace perception
}  // namespace apollo
#endif  // RADAR_LIB_INTERFACE_BASE_TRACKER_H_
