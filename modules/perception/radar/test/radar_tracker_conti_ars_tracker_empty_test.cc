// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: hui yujiang (huiyujiang@baidu.com)
// @file: radar_tracker_conti_ars_tracker_test.cc
// @brief: conti ars tracker test

#include <gtest/gtest.h>
#include "cybertron/common/log.h"
#include "modules/perception/radar/lib/tracker/conti_ars_tracker/conti_ars_tracker.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}  // namespace lib
namespace radar {
TEST(ContiArsTrackerTest, conti_ars_tracker_empty_init_test) {
  BaseTracker* tracker = new ContiArsTracker();
  lib::FLAGS_work_root = "./radar_test_data/conti_ars_tracker/empty";
  EXPECT_EQ(tracker->Init(), false);
  delete tracker;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
