#include "radar_driver/conti_radar.h"
#include <gtest/gtest.h>
#include <string>
#include "ros/ros.h"

namespace adu {
namespace sensor {
namespace conti_radar {
TEST(ContiRadarCan, constructor) {
  ros::Publisher pub;
  ::adu::sensor::conti_radar::ContiRadarCan radar(pub, 0, "sensor_radar", 2);
  EXPECT_EQ(radar.get_can_port(), 2) << "CAN port is wrong";
  EXPECT_STREQ(radar.module_name().c_str(), "sensor_radar")
      << "module name is wrong";
  EXPECT_EQ(radar.scan_index(), -1) << "scan index is wrong";
  EXPECT_EQ(radar.output_type(), 0) << "output_type is wrong";
  ::adu::sensor::conti_radar::ContiRadarCan radar2(pub, 1, "sensor_radar", 3);
  EXPECT_EQ(radar2.get_can_port(), 3) << "CAN port is wrong";
  EXPECT_EQ(radar2.output_type(), 1) << "output_type is wrong";
}

TEST(ContiRadarEther, constructor) {
  ros::Publisher pub;
  ::adu::sensor::conti_radar::ContiRadarEther radar(pub, 0, "sensor_radar",
                                                    7777, "192.168.10.6");
  EXPECT_EQ(radar.get_port_num(), 7777) << "Port num is wrong";
  EXPECT_STREQ(radar.get_ip_addr().c_str(), "192.168.10.6")
      << "IP address is wrong";
  EXPECT_STREQ(radar.module_name().c_str(), "sensor_radar")
      << "module name is wrong";
  EXPECT_EQ(radar.scan_index(), -1) << "scan index is wrong";
  EXPECT_EQ(radar.output_type(), 0) << "output_type is wrong";
  ::adu::sensor::conti_radar::ContiRadarEther radar2(pub, 1, "sensor_radar",
                                                     7777, "192.168.10.6");
  EXPECT_EQ(radar2.output_type(), 1) << "output_type is wrong";
}

TEST(ContiRadarBase, communicate_cluster) {
  ros::Publisher pub;
  ::adu::sensor::conti_radar::ContiRadarCan radar(pub, 1, "sensor_radar", 2);
  CanMsg *canframe = new CanMsg;
  canframe->id = 0x600;
  canframe->len = 8;
  (canframe->data)[0] = 0x55;
  (canframe->data)[1] = 0x55;
  (canframe->data)[2] = 0xaa;
  (canframe->data)[3] = 0xaa;
  (canframe->data)[4] = 0x00;
  (canframe->data)[5] = 0x00;
  (canframe->data)[6] = 0x00;
  (canframe->data)[7] = 0x00;
  (canframe->timestamp).tv_sec = 1492071713;
  (canframe->timestamp).tv_usec = 188425;
  radar.communicate(2, canframe, true);
  EXPECT_EQ(radar.output_type(), 1) << "output_type is wrong";
  EXPECT_EQ(radar.detect_num(), 170) << "detect_num is wrong";
  ::adu::common::sensor::ContiRadar objtemparray = radar.object_msg();
  EXPECT_STREQ(objtemparray.header().module_name().c_str(), "sensor_radar");
  EXPECT_EQ(objtemparray.header().sequence_num(), 43690);
  double timesecdiff =
      fabs(1492071713.188425 - objtemparray.measurement_time());
  EXPECT_EQ(objtemparray.type(), 1);
  EXPECT_EQ(objtemparray.contiobs_size(), 0);

  canframe->id = 0x701;
  (canframe->data)[0] = 0x1F;
  (canframe->data)[1] = 0x7E;
  (canframe->data)[2] = 0x80;
  (canframe->data)[3] = 0xAA;
  (canframe->data)[4] = 0xEE;
  (canframe->data)[5] = 0x2E;
  (canframe->data)[6] = 0x00;
  (canframe->data)[7] = 0x55;
  (canframe->timestamp).tv_sec = 1492071713;
  (canframe->timestamp).tv_usec = 288425;
  radar.communicate(2, canframe, true);
  objtemparray = radar.object_msg();
  EXPECT_EQ(objtemparray.contiobs_size(), 1);
  ::adu::common::sensor::ContiRadarObs contiobs = objtemparray.contiobs(0);
  EXPECT_STREQ(contiobs.header().module_name().c_str(), "sensor_radar");
  EXPECT_EQ(contiobs.header().sequence_num(), 43690);
  timesecdiff = fabs(1492071713.288425 - contiobs.header().timestamp_sec());
  EXPECT_LT(timesecdiff, 1e-5);
  double timensecdiff =
      fabs(1492071713188425000 - objtemparray.header().radar_timestamp());
  EXPECT_LT(timensecdiff, 1e3);
  double longdistdiff = fabs(contiobs.longitude_dist() - 309.6);
  double latdistdiff = fabs(contiobs.lateral_dist() + 68.3);
  double longveldiff = fabs(contiobs.longitude_vel() - 110.0);
  double latveldiff = fabs(contiobs.lateral_vel() - 28.0);
  double rcsdiff = fabs(contiobs.rcs() + 21.5);
  EXPECT_EQ(contiobs.obstacle_id(), 31) << "obstacle_id is wrong";
  EXPECT_LT(longdistdiff, 1e-3) << "longitude distance is wrong";
  EXPECT_LT(latdistdiff, 1e-3) << "lateral distance is wrong";
  EXPECT_LT(longveldiff, 1e-3) << "longitude velocity is wrong";
  EXPECT_LT(latveldiff, 1e-3) << "lateral velocity is wrong";
  EXPECT_LT(rcsdiff, 1e-3) << "rcs is wrong";
  delete canframe;
}
TEST(ContiRadarBase, communicate_track) {
  ros::Publisher pub;
  ::adu::sensor::conti_radar::ContiRadarCan radar(pub, 0, "sensor_radar", 2);
  CanMsg *canframe = new CanMsg;
  canframe->id = 0x60A;
  canframe->len = 8;
  (canframe->data)[0] = 0x55;
  (canframe->data)[1] = 0x55;
  (canframe->data)[2] = 0xaa;
  (canframe->data)[3] = 0x00;
  (canframe->data)[4] = 0x00;
  (canframe->data)[5] = 0x00;
  (canframe->data)[6] = 0x00;
  (canframe->data)[7] = 0x00;
  (canframe->timestamp).tv_sec = 1492071713;
  (canframe->timestamp).tv_usec = 188425;
  radar.communicate(2, canframe, true);
  EXPECT_EQ(radar.output_type(), 0) << "output_type is wrong";
  EXPECT_EQ(radar.detect_num(), 85) << "detect_num is wrong";
  ::adu::common::sensor::ContiRadar objtemparray = radar.object_msg();
  EXPECT_STREQ(objtemparray.header().module_name().c_str(), "sensor_radar");
  EXPECT_EQ(objtemparray.header().sequence_num(), 21930);
  double timesecdiff =
      fabs(1492071713.188425 - objtemparray.measurement_time());
  EXPECT_EQ(objtemparray.type(), 1);
  EXPECT_EQ(objtemparray.contiobs_size(), 0);

  canframe->id = 0x60B;
  (canframe->data)[0] = 0x1F;
  (canframe->data)[1] = 0x7E;
  (canframe->data)[2] = 0x80;
  (canframe->data)[3] = 0xAA;
  (canframe->data)[4] = 0xEE;
  (canframe->data)[5] = 0x2E;
  (canframe->data)[6] = 0x00;
  (canframe->data)[7] = 0x55;
  (canframe->timestamp).tv_sec = 1492071713;
  (canframe->timestamp).tv_usec = 288425;
  radar.communicate(2, canframe, true);
  objtemparray = radar.object_msg();
  EXPECT_EQ(objtemparray.contiobs_size(), 1);
  ::adu::common::sensor::ContiRadarObs contiobs = objtemparray.contiobs(0);
  EXPECT_STREQ(contiobs.header().module_name().c_str(), "sensor_radar");
  EXPECT_EQ(contiobs.header().sequence_num(), 21930);
  timesecdiff = fabs(1492071713.288425 - contiobs.header().timestamp_sec());
  EXPECT_LT(timesecdiff, 1e-5);
  double timensecdiff =
      fabs(1492071713188425000 - objtemparray.header().radar_timestamp());
  EXPECT_LT(timensecdiff, 1e3);
  double longdistdiff = fabs(contiobs.longitude_dist() - 309.6);
  double latdistdiff = fabs(contiobs.lateral_dist() + 170.6);
  double longveldiff = fabs(contiobs.longitude_vel() - 110.0);
  double latveldiff = fabs(contiobs.lateral_vel() - 28.0);
  double rcsdiff = fabs(contiobs.rcs() + 21.5);
  EXPECT_EQ(contiobs.obstacle_id(), 31) << "obstacle_id is wrong";
  EXPECT_LT(longdistdiff, 1e-3) << "longitude distance is wrong";
  EXPECT_LT(latdistdiff, 1e-3) << "lateral distance is wrong";
  EXPECT_LT(longveldiff, 1e-3) << "longitude velocity is wrong";
  EXPECT_LT(latveldiff, 1e-3) << "lateral velocity is wrong";
  EXPECT_LT(rcsdiff, 1e-3) << "rcs is wrong";
  delete canframe;
}
}
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "conti_radar_test");
  return RUN_ALL_TESTS();
}
