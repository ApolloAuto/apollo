/*********************************************************************
 * C++ unit test for velodyne_laserscan
 * Verify correct handling of subscribe and unsubscribe events
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// Subscriber receive callback
void recv(const sensor_msgs::LaserScanConstPtr& msg) {}

// Build and publish a minimal PointCloud2 message
void publish(const ros::Publisher &pub) {
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "";
  msg.header.stamp = ros::Time::now();
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(1 * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  pub.publish(msg);
}

// Verify correct handling of subscribe and unsubscribe events
TEST(Main, subscribe_unsubscribe)
{
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 2);

  // Wait for node to startup
  ros::WallDuration(2.0).sleep();
  ros::spinOnce();
  EXPECT_EQ(0, pub.getNumSubscribers());

  // Subscribe to 'scan' and expect the node to subscribe to 'velodyne_points'
  ros::Subscriber sub = nh.subscribe("scan", 2, recv);
  for (size_t i = 10; i > 0; i--) {
    publish(pub);
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }
  EXPECT_EQ(1, sub.getNumPublishers());
  EXPECT_EQ(1, pub.getNumSubscribers());

  // Unsubscribe from 'scan' and expect the node to unsubscribe from 'velodyne_points'
  sub.shutdown();
  for (size_t i = 10; i > 0; i--) {
    publish(pub);
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }
  EXPECT_EQ(0, sub.getNumPublishers());
  EXPECT_EQ(0, pub.getNumSubscribers());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lazy_subscriber");
  return RUN_ALL_TESTS();
}
