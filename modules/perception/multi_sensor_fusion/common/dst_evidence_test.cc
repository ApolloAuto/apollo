/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/multi_sensor_fusion/common/dst_evidence.h"

#include <boost/format.hpp>

#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {
// elementary hypotheses
enum { F111 = (1 << 0), FA18 = (1 << 1), P3C = (1 << 2) };
// compound hypotheses
enum { FAST = (F111 | FA18), UNKNOWN = (F111 | FA18 | P3C) };
static const std::vector<uint64_t> fod_subsets = {F111, FA18, P3C, FAST,
                                                  UNKNOWN};
static const std::map<uint64_t, std::string> hypo_names = {
    {F111, "F111"},
    {FA18, "FA18"},
    {P3C, "P3C"},
    {FAST, "FAST"},
    {UNKNOWN, "UNKNOWN"}};

class DSTEvidenceTest : public ::testing::Test {
 public:
  DSTEvidenceTest()
      : sensor1_dst_("test"), sensor2_dst_("test"), fused_dst_("test") {
    DstManager *dst_manager = DstManager::Instance();
    std::vector<std::string> fod_subset_names = {"F111", "FA18", "P3C", "FAST",
                                                 "UNKNOWN"};
    dst_manager->AddApp("test", fod_subsets, fod_subset_names);
    vec_equal_ = [](const std::vector<double> &vec,
                    const std::vector<double> &gt) {
      CHECK_EQ(vec.size(), gt.size());
      for (size_t i = 0; i < vec.size(); ++i) {
        EXPECT_NEAR(vec[i], gt[i], 1e-6);
      }
    };
  }
  ~DSTEvidenceTest() {}

 protected:
  void run(const std::vector<double> &sensor1_data,
           const std::vector<double> &sensor2_data) {
    ASSERT_TRUE(sensor1_dst_.SetBbaVec(sensor1_data));
    ASSERT_TRUE(sensor2_dst_.SetBbaVec(sensor2_data));
    fused_dst_ = sensor1_dst_ + sensor2_dst_;
    fused_dst_.ComputeSptPlsUct();
    fused_dst_.ComputeProbability();
    fused_dst_vec_ = fused_dst_.GetBbaVec();
    fused_spt_vec_ = fused_dst_.GetSupportVec();
    fused_pls_vec_ = fused_dst_.GetPlausibilityVec();
    fused_uct_vec_ = fused_dst_.GetUncertaintyVec();
    fused_prob_vec_ = fused_dst_.GetProbabilityVec();
  }
  void assign_dst_test() {
    std::map<uint64_t, double> dst_map = {
        {F111, 0.3}, {FA18, 0.18}, {FAST, 0.42}, {UNKNOWN, 0.1}};
    std::vector<double> dst_vec_gt = {0.3, 0.18, 0.00, 0.42, 0.1};
    Dst dst("test");
    dst.SetBba(dst_map);
    const std::vector<double> dst_vec = dst.GetBbaVec();
    for (size_t i = 0; i < dst_vec_gt.size(); ++i) {
      EXPECT_DOUBLE_EQ(dst_vec[i], dst_vec_gt[i]);
    }
  }
  void add_dst_test1() {
    std::vector<double> sensor1_data = {0.3, 0.15, 0.03, 0.42, 0.1};
    std::vector<double> sensor2_data = {0.4, 0.1, 0.02, 0.45, 0.03};
    run(sensor1_data, sensor2_data);
    std::vector<double> fused_dst_gt = {0.546233074876, 0.160861011457,
                                        0.00405045712302, 0.285383636153,
                                        0.00347182039116};
    std::vector<double> fused_spt_gt = {0.5462330748755931, 0.1608610114570073,
                                        0.004050457123018168,
                                        0.9924777224858234, 1.0};
    std::vector<double> fused_pls_gt = {0.8350885314199745, 0.4497164680013887,
                                        0.007522277514176598,
                                        0.9959495428769819, 1.0};
    std::vector<double> fused_prob_gt = {0.69008217, 0.30471010, 0.00520773,
                                         0.99479227, 1.0000000};
    std::vector<double> fused_uct_gt(5);
    for (size_t i = 0; i < fused_spt_gt.size(); ++i) {
      fused_uct_gt[i] = fused_pls_gt[i] - fused_spt_gt[i];
    }
    AINFO << fused_dst_.PrintBba();
    // double sum = std::accumulate(fused_dst__vec.begin(),
    // fused_dst__vec.end(), 0.0);
    // AINFO << boost::format("fused dst sum: %lf") % sum;
    AINFO << "check dst";
    vec_equal_(fused_dst_vec_, fused_dst_gt);
    AINFO << "check spt";
    vec_equal_(fused_spt_vec_, fused_spt_gt);
    AINFO << "check pls";
    vec_equal_(fused_pls_vec_, fused_pls_gt);
    AINFO << "check uct";
    vec_equal_(fused_uct_vec_, fused_uct_gt);
    AINFO << "check prob";
    vec_equal_(fused_prob_vec_, fused_prob_gt);
  }
  void add_dst_test2() {
    std::vector<double> sensor1_data = {0.3, 0.15, 0.03, 0.42, 0.1};
    std::vector<double> sensor2_data = {0.5, 0.3, 0.17, 0.00, 0.03};
    run(sensor1_data, sensor2_data);
    std::vector<double> fused_dst_gt = {
        0.6318805610013573, 0.30990800784195444, 0.034685567787664004,
        0.019001658874981148, 0.004524204494043131};
    std::vector<double> fused_spt_gt = {0.6318805610013573, 0.30990800784195444,
                                        0.034685567787664004,
                                        0.9607902277182928, 1.0};
    std::vector<double> fused_pls_gt = {0.6554064243703815, 0.3334338712109787,
                                        0.039209772281707134,
                                        0.9653144322123359, 1.0};
    std::vector<double> fused_prob_gt = {0.64288946, 0.32091691, 0.03619364,
                                         0.96380636, 1.00000000};
    std::vector<double> fused_uct_gt(5);
    for (size_t i = 0; i < fused_spt_gt.size(); ++i) {
      fused_uct_gt[i] = fused_pls_gt[i] - fused_spt_gt[i];
    }
    AINFO << fused_dst_.PrintBba();
    AINFO << "check dst";
    vec_equal_(fused_dst_vec_, fused_dst_gt);
    AINFO << "check spt";
    vec_equal_(fused_spt_vec_, fused_spt_gt);
    AINFO << "check pls";
    vec_equal_(fused_pls_vec_, fused_pls_gt);
    AINFO << "check uct";
    vec_equal_(fused_uct_vec_, fused_uct_gt);
    AINFO << "check prob";
    vec_equal_(fused_prob_vec_, fused_prob_gt);
  }
  void multipy_dst_test() {
    std::vector<double> dst_data = {0.13891389, 0.57305731, 0.02580258,
                                    0.22472247, 0.03750375};
    std::vector<double> gt = {0.11113111, 0.45844584, 0.02064206, 0.17977798,
                              0.230003};
    Dst dst("test");
    dst.SetBbaVec(dst_data);
    Dst res("test");
    res = dst * 0.8;
    AINFO << res.PrintBba();
    vec_equal_(res.GetBbaVec(), gt);
  }

 private:
  std::function<void(const std::vector<double> &vec,
                     const std::vector<double> &gt)>
      vec_equal_;
  // InitApp _init_app;
  Dst sensor1_dst_;
  Dst sensor2_dst_;
  Dst fused_dst_;
  std::vector<double> fused_dst_vec_;
  std::vector<double> fused_spt_vec_;
  std::vector<double> fused_pls_vec_;
  std::vector<double> fused_uct_vec_;
  std::vector<double> fused_prob_vec_;
};
TEST_F(DSTEvidenceTest, assign_dst_test) { DSTEvidenceTest::assign_dst_test(); }
TEST_F(DSTEvidenceTest, add_dst_test1) { DSTEvidenceTest::add_dst_test1(); }
TEST_F(DSTEvidenceTest, add_dst_test2) { DSTEvidenceTest::add_dst_test2(); }
TEST_F(DSTEvidenceTest, multipy_dst_test) {
  DSTEvidenceTest::multipy_dst_test();
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
