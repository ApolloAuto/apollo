#include "modules/control/common/interpolation_1d.h"

#include <string>
#include <utility>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/log.h"

#include "modules/common/util/util.h"
#include "modules/control/proto/control_conf.pb.h"

using ::testing::ElementsAre;

namespace apollo {
namespace control {

class Interpolation1DTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string control_conf_file = "modules/control/testdata/lincoln.pb.txt";
    CHECK(
        ::apollo::util::load_file_to_proto(control_conf_file, &control_conf_));
  }

 protected:
  ControlConf control_conf_;
};

TEST_F(Interpolation1DTest, normal) {
  Interpolation1D::DataType xy{{0, 0}, {15, 12}, {30, 17}};

  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, estimator.Interpolate(xy[i].first));
  }

  EXPECT_DOUBLE_EQ(4.7777777777777777, estimator.Interpolate(5));
  EXPECT_DOUBLE_EQ(8.7777777777777786, estimator.Interpolate(10));
  EXPECT_DOUBLE_EQ(14.444444444444445, estimator.Interpolate(20));

  // out of x range
  EXPECT_DOUBLE_EQ(0, estimator.Interpolate(-1));
  EXPECT_DOUBLE_EQ(17, estimator.Interpolate(30));
}

TEST_F(Interpolation1DTest, unordered) {
  Interpolation1D::DataType xy{{15, 12}, {5, 5}, {40, 25}, {30, 17}};

  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, estimator.Interpolate(xy[i].first));
  }
}

TEST_F(Interpolation1DTest, brake_table) {
  const auto& brake_table =
      control_conf_.simple_longitudinal_conf().brake_table();
  AINFO << "Brake calibration table:" << brake_table.DebugString();

  Interpolation1D::DataType xy;

  for (int i = 0; i < brake_table.calibration_size(); ++i) {
    xy.push_back(std::make_pair(brake_table.calibration(i).acceleration(),
                                brake_table.calibration(i).brake()));
  }
  Interpolation1D estimator;
  EXPECT_TRUE(estimator.Init(xy));

  for (unsigned i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, estimator.Interpolate(xy[i].first));
  }
}

}  // namespace control
}  // namespace apollo
