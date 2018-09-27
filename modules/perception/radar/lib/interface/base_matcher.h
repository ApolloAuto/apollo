// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Yujiang Hui (huiyujiang@baidu.com)
// @file: base_matcher.h
// @brief: base match interface
#ifndef RADAR_LIB_INTERFACE_BASE_MATCHER_H_
#define RADAR_LIB_INTERFACE_BASE_MATCHER_H_
#include <gtest/gtest_prod.h>
#include <string>
#include <utility>
#include <vector>
#include "modules/perception/base/frame.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/radar/lib/tracker/common/radar_track.h"

namespace apollo {
namespace perception {
namespace radar {
typedef std::pair<size_t, size_t> TrackObjectPair;

struct TrackObjectMatcherOptions {
  Eigen::Vector3d *ref_point = nullptr;
};

class BaseMatcher {
 public:
  BaseMatcher() : name_("BaseMatcher") {}
  virtual ~BaseMatcher() {}
  virtual bool Init() { return true; }
  // @brief match radar objects to tracks
  // @params[IN] radar_tracks: global tracks
  // @params[IN] radar_frame: current radar frame
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @return nothing
  virtual bool Match(const std::vector<RadarTrackPtr> &radar_tracks,
                     const base::Frame &radar_frame,
                     const TrackObjectMatcherOptions &options,
                     std::vector<TrackObjectPair> *assignments,
                     std::vector<size_t> *unassigned_tracks,
                     std::vector<size_t> *unassigned_objects) {
    return true;
  }
  virtual void IDMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                       const base::Frame &radar_frame,
                       std::vector<TrackObjectPair> *assignments,
                       std::vector<size_t> *unassigned_tracks,
                       std::vector<size_t> *unassigned_objects);
  static void SetMaxMatchDistance(double dist);
  static double GetMaxMatchDistance();
  static void SetBoundMatchDistance(double dist);
  static double GetBoundMatchDistance();
  virtual std::string Name() { return name_; }

 protected:
  std::string name_;
  static double s_max_match_distance_;
  static double s_bound_match_distance_;
  virtual bool RefinedTrack(const base::ObjectPtr &track_object,
                            double track_timestamp,
                            const base::ObjectPtr &radar_object,
                            double radar_timestamp);
  FRIEND_TEST(BaseMatcherTest, base_matcher_test);

 private:
  BaseMatcher(const BaseMatcher &) = delete;
  BaseMatcher &operator=(const BaseMatcher &) = delete;
};

PERCEPTION_REGISTER_REGISTERER(BaseMatcher);
#define PERCEPTION_REGISTER_MATCHER(name) \
  PERCEPTION_REGISTER_CLASS(BaseMatcher, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo
#endif  // RADAR_LIB_INTERFACE_BASE_MATCHER_H_
