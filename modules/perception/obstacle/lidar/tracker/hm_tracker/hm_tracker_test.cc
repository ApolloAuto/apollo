#include <gtest/gtest.h>
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"

namespace apollo {
namespace perception {

class HmObjectTrackerTest : public testing::Test {
 protected:
  HmObjectTrackerTest() : config_manager_(NULL) {}
  virtual ~HmObjectTrackerTest() {}
  void SetUp() {

    FLAGS_work_root = "modules/perception";
    FLAGS_config_manager_path = "conf/config_manager.config";

    config_manager_ = Singleton<ConfigManager>::Get();
    if (config_manager_ == NULL) {
      AERROR << "failed to get ConfigManager instance.";
      return ;
    }
    if (!config_manager_->Init()) {
      AERROR << "failed to init ConfigManager";
      return ;
    }
    hm_tracker_ = new HmObjectTracker();
  }
  void TearDown() {
    delete hm_tracker_;
    hm_tracker_ = NULL;
  }

 protected:
  ConfigManager*    config_manager_;
  HmObjectTracker*  hm_tracker_;
};

TEST_F(HmObjectTrackerTest, segment) {
  // test init()
  EXPECT_TRUE(hm_tracker_->Init()); 
}

}  // namespace perception
}  // namespace apollo
