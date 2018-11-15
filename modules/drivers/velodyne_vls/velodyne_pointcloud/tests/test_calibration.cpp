//
// C++ unit tests for calibration interface.
//

#include <gtest/gtest.h>

#include <ros/package.h>
#include <velodyne_pointcloud/calibration.h>
using namespace velodyne_pointcloud;

// global test data
std::string g_package_name("velodyne_pointcloud");
std::string g_package_path;

void init_global_data(void)
{
  g_package_path = ros::package::getPath(g_package_name);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

TEST(Calibration, missing_file)
{
  Calibration calibration(false);
  calibration.read("./no_such_file.yaml");
  EXPECT_FALSE(calibration.initialized);
}

TEST(Calibration, vlp16)
{
  Calibration calibration(g_package_path + "/params/VLP16db.yaml", false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 16);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.2617993877991494);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[15];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, 0.2617993877991494);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

TEST(Calibration, hdl32e)
{
  Calibration calibration(g_package_path + "/params/32db.yaml", false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 32);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.5352924815866609);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[31];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, 0.18622663118779495);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

TEST(Calibration, hdl64e)
{
  Calibration calibration(g_package_path + "/params/64e_utexas.yaml", false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 64);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.124932751059532);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[63];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.209881335496902);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

TEST(Calibration, hdl64e_s21)
{
  Calibration calibration(g_package_path + "/params/64e_s2.1-sztaki.yaml",
                          false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 64);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.15304134919741974);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.025999999);
  EXPECT_EQ(laser.max_intensity, 235);
  EXPECT_EQ(laser.min_intensity, 30);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[63];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.2106649408137298);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, -0.025999999);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

TEST(Calibration, hdl64e_s2_float_intensities)
{
  Calibration calibration(g_package_path +
                          "/tests/issue_84_float_intensities.yaml",
                          false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 64);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.12118950050089745);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.025999999);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 40);

  // check similar values for laser 26:
  laser = calibration.laser_corrections[26];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.014916840599137901);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.025999999);
  EXPECT_EQ(laser.max_intensity, 245);
  EXPECT_EQ(laser.min_intensity, 0);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[63];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.20683046990039078);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, -0.025999999);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  init_global_data();
  return RUN_ALL_TESTS();
}

