// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
// @date 2016/09/19 21:17:39

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include "module/perception/traffic_light/onboard/test/subnode_test.h"
#include "module/perception/traffic_light/onboard/preprocessor_subnode.h"

namespace adu {
namespace perception {
namespace config_manager {
DECLARE_string(config_manager_path);
};
namespace traffic_light {

class TLPreprocessorSubnodeTest : public SubnodeTest {
 public:
  TLPreprocessorSubnodeTest() = default;
  virtual ~TLPreprocessorSubnodeTest() = default;
  virtual void SetUp();
  virtual void TearDown();
 protected:
  TLPreprocessorSubnode subnode;
  std::string config_manager_path;
};

void TLPreprocessorSubnodeTest::SetUp() {
  //clear the config manager
  if (base::Singleton<config_manager::ConfigManager>::_instance) {
    delete base::Singleton<config_manager::ConfigManager>::_instance;
    base::Singleton<config_manager::ConfigManager>::_instance = NULL;
  }
  base::Singleton<config_manager::ConfigManager>::_instance =
      new config_manager::ConfigManager;
  //store the path.
  config_manager_path = config_manager::fLS::FLAGS_config_manager_path;
  //init event_manager
  std::vector<std::string> event_names;
  event_names.push_back("synced-images");
  ASSERT_TRUE(init_event_manager(event_names, &subnode._sub_meta_events,
                                 &event_manager));
  subnode._event_manager = &event_manager;
  //init share_data
  std::vector<std::string> share_data_names;
  share_data_names.push_back("TLPreprocessingData");
  ASSERT_TRUE(init_shared_data_manager(share_data_names, &shared_data_manager));
  subnode._shared_data_manager = &shared_data_manager;
}

void TLPreprocessorSubnodeTest::TearDown() {
  //restore the path.
  config_manager::fLS::FLAGS_config_manager_path = config_manager_path;
  delete base::Singleton<config_manager::ConfigManager>::_instance;
  base::Singleton<config_manager::ConfigManager>::_instance = NULL;
}

TEST_F(TLPreprocessorSubnodeTest, init_shared_data_failed) {
  std::vector<std::string> share_data_names;
  share_data_names.push_back("Not-Exist-Shared-Data");
  onboard::SharedDataManager local_shared_data_manager;
  ASSERT_FALSE(init_shared_data_manager(share_data_names, &local_shared_data_manager));
  TLPreprocessorSubnode local_subnode;
  local_subnode._shared_data_manager = &local_shared_data_manager;
  ASSERT_FALSE(local_subnode.init_shared_data());
}

TEST_F(TLPreprocessorSubnodeTest, init_get_module_config_failed) {
  ASSERT_TRUE(subnode.init_shared_data());
  config_manager::fLS::FLAGS_config_manager_path = "somewhere-not-exist.";
  ASSERT_FALSE(subnode.init_internal());
}

TEST_F(TLPreprocessorSubnodeTest, init_synchronizer_lack_sync_failed) {
  config_manager::fLS::FLAGS_config_manager_path =
      "conf/preprocessor_config_manager_lack_sync.config";
  if (base::Singleton<config_manager::ConfigManager>::_instance) {
    delete base::Singleton<config_manager::ConfigManager>::_instance;
    base::Singleton<config_manager::ConfigManager>::_instance = NULL;
  }
  base::Singleton<config_manager::ConfigManager>::_instance =
      new config_manager::ConfigManager;
  ASSERT_TRUE(subnode.init_shared_data());
  ASSERT_FALSE(subnode.init_internal());
}

TEST_F(TLPreprocessorSubnodeTest, init_synchronizer_bad_sync_failed) {
  config_manager::fLS::FLAGS_config_manager_path =
      "conf/preprocessor_config_manager_bad_sync.config";
  if (base::Singleton<config_manager::ConfigManager>::_instance) {
    delete base::Singleton<config_manager::ConfigManager>::_instance;
    base::Singleton<config_manager::ConfigManager>::_instance = NULL;
  }
  base::Singleton<config_manager::ConfigManager>::_instance =
      new config_manager::ConfigManager;
  ASSERT_TRUE(subnode.init_shared_data());
  ASSERT_FALSE(subnode.init_internal());
}

TEST_F(TLPreprocessorSubnodeTest, init_synchronizer_lack_revserv_field) {
  if (base::Singleton<config_manager::ConfigManager>::_instance) {
    delete base::Singleton<config_manager::ConfigManager>::_instance;
    base::Singleton<config_manager::ConfigManager>::_instance = NULL;
  }
  base::Singleton<config_manager::ConfigManager>::_instance =
      new config_manager::ConfigManager;
  config_manager::fLS::FLAGS_config_manager_path = "conf/preprocessor_config_manager.config";
  ASSERT_TRUE(subnode.init_shared_data());

  subnode._reserve = "bad-revserv";
  ASSERT_FALSE(subnode.init_internal());
}

TEST_F(TLPreprocessorSubnodeTest, init_succ) {

  if (base::Singleton<config_manager::ConfigManager>::_instance) {
    delete base::Singleton<config_manager::ConfigManager>::_instance;
    base::Singleton<config_manager::ConfigManager>::_instance = NULL;
  }
  base::Singleton<config_manager::ConfigManager>::_instance =
      new config_manager::ConfigManager;
  config_manager::fLS::FLAGS_config_manager_path = "conf/preprocessor_config_manager.config";
  ASSERT_TRUE(subnode.init_shared_data());

  subnode._reserve = "long_focus_camera_source_type:1;long_focus_camera_source_name:long_focus;"
      "short_focus_camera_source_type:1;short_focus_camera_source_name:short_focus";
  ASSERT_FALSE(subnode.init_internal());
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
