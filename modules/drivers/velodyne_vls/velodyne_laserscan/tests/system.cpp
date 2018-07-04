/*********************************************************************
 * C++ unit test for velodyne_laserscan
 * Verify all aspects of the system
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// Define our own PointCloud type for easy use
typedef struct {
  float x; // x
  float y; // y
  float z; // z
  float i; // intensity
  uint16_t r; // ring
} Point;
typedef struct {
  std_msgs::Header header;
  std::vector<Point> points;
} PointCloud;

// Global variables
ros::Publisher g_pub;
ros::Subscriber g_sub;
sensor_msgs::LaserScan g_scan;
volatile bool g_scan_new = false;

// Convert WallTime to Time
static inline ros::Time rosTime(const ros::WallTime &stamp) {
  return ros::Time(stamp.sec, stamp.nsec);
}

// Subscriber receive callback
void recv(const sensor_msgs::LaserScanConstPtr& msg) {
  g_scan = *msg;
  g_scan_new = true;
}

// Wait for incoming LaserScan message
bool waitForScan(ros::WallDuration dur) {
  const ros::WallTime start = ros::WallTime::now();
  while (!g_scan_new) {
    if ((ros::WallTime::now() - start) > dur) {
      return false;
    }
    ros::WallDuration(0.001).sleep();
    ros::spinOnce();
  }
  return true;
}

// Build and publish PointCloud2 messages of various structures
void publishXYZIR1(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = cloud.header.frame_id;
  msg.header.stamp = cloud.header.stamp;
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
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((float*)(ptr +  0)) = cloud.points[i].x;
    *((float*)(ptr +  4)) = cloud.points[i].y;
    *((float*)(ptr +  8)) = cloud.points[i].z;
    *((float*)(ptr + 16)) = cloud.points[i].i;
    *((uint16_t*)(ptr + 20)) = cloud.points[i].r;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishXYZIR2(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 19;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = cloud.header.frame_id;
  msg.header.stamp = cloud.header.stamp;
  msg.fields.resize(5);
  msg.fields[0].name = "z";
  msg.fields[0].offset = 4;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 8;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "x";
  msg.fields[2].offset = 12;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 0;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((float*)(ptr +  0)) = cloud.points[i].i;
    *((float*)(ptr +  4)) = cloud.points[i].z;
    *((float*)(ptr +  8)) = cloud.points[i].y;
    *((float*)(ptr + 12)) = cloud.points[i].x;
    *((uint16_t*)(ptr + 16)) = cloud.points[i].r;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishXYZR(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 15;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = cloud.header.frame_id;
  msg.header.stamp = cloud.header.stamp;
  msg.fields.resize(4);
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
  msg.fields[3].name = "ring";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[3].count = 1;
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((float*)(ptr + 0)) = cloud.points[i].x;
    *((float*)(ptr + 4)) = cloud.points[i].y;
    *((float*)(ptr + 8)) = cloud.points[i].z;
    *((uint16_t*)(ptr + 12)) = cloud.points[i].r;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishR(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 2;
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = rosTime(ros::WallTime::now());
  msg.fields.resize(1);
  msg.fields[0].name = "ring";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[0].count = 1;
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((uint16_t*)(ptr + 0)) = cloud.points[i].r;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishXYZR32(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 16;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = cloud.header.frame_id;
  msg.header.stamp = cloud.header.stamp;
  msg.fields.resize(4);
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
  msg.fields[3].name = "ring";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT32;
  msg.fields[3].count = 1;
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((float*)(ptr + 0)) = cloud.points[i].x;
    *((float*)(ptr + 4)) = cloud.points[i].y;
    *((float*)(ptr + 8)) = cloud.points[i].z;
    *((uint32_t*)(ptr + 12)) = cloud.points[i].r;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishXYZ(const PointCloud &cloud) {
  g_scan_new = false;
  const uint32_t POINT_STEP = 12;
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = rosTime(ros::WallTime::now());
  msg.fields.resize(3);
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
  msg.data.resize(std::max((size_t)1, cloud.points.size()) * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < cloud.points.size(); i++) {
    *((float*)(ptr + 0)) = cloud.points[i].x;
    *((float*)(ptr + 4)) = cloud.points[i].y;
    *((float*)(ptr + 8)) = cloud.points[i].z;
    ptr += POINT_STEP;
  }
  g_pub.publish(msg);
}
void publishNone() {
  g_scan_new = false;
  const uint32_t POINT_STEP = 16;
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = rosTime(ros::WallTime::now());
  msg.data.resize(1 * POINT_STEP, 0x00);
  msg.point_step = POINT_STEP;
  msg.row_step = msg.data.size();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  g_pub.publish(msg);
}

// Find the index of the point in the PointCloud with the shortest 2d distance to the point (x,y)
static inline float SQUARE(float x) { return x * x; }
size_t findClosestIndex(const PointCloud &cloud, uint16_t ring, float x, float y) {
  size_t index = SIZE_MAX;
  float delta = INFINITY;
  for (size_t i = 0; i < cloud.points.size(); i++) {
    if (cloud.points[i].r == ring) {
      float dist = SQUARE(x - cloud.points[i].x) + SQUARE(y - cloud.points[i].y);
      if (dist < delta) {
        delta = dist;
        index = i;
      }
    }
  }
  return index;
}

// Verify that all LaserScan header values are values are passed through, and other values are default
void verifyScanEmpty(const PointCloud &cloud, bool intensity = true) {
  ASSERT_EQ(cloud.header.stamp, g_scan.header.stamp);
  EXPECT_EQ(cloud.header.frame_id, g_scan.header.frame_id);
  for (size_t i = 0; i < g_scan.ranges.size(); i++) {
    EXPECT_EQ(INFINITY, g_scan.ranges[i]);
  }
  if (!intensity) {
    EXPECT_EQ(0, g_scan.intensities.size());
  } else {
    EXPECT_EQ(g_scan.ranges.size(), g_scan.intensities.size());
    for (size_t i = 0; i < g_scan.intensities.size(); i++) {
      EXPECT_EQ(0.0, g_scan.intensities[i]);
    }
  }
}

// Verify that every PointCloud point made it to the LaserScan and other values are default
void verifyScanSparse(const PointCloud &cloud, uint16_t ring, uint16_t ring_count, bool intensity = true) {
  ASSERT_EQ(cloud.header.stamp, g_scan.header.stamp);
  EXPECT_EQ(cloud.header.frame_id, g_scan.header.frame_id);
  EXPECT_EQ(intensity ? g_scan.ranges.size() : 0, g_scan.intensities.size());
  size_t count = 0;
  for (size_t i = 0; i < g_scan.ranges.size(); i++) {
    double r = g_scan.ranges[i];
    if (std::isfinite(r)) {
      float a = g_scan.angle_min + i * g_scan.angle_increment;
      float x = g_scan.ranges[i] * cosf(a);
      float y = g_scan.ranges[i] * sinf(a);
      float e = g_scan.ranges[i] * g_scan.angle_increment + (float)1e-3; // allowable error
      size_t index = findClosestIndex(cloud, ring, x, y);
      if (index < cloud.points.size()) {
        count++;
        EXPECT_NEAR(cloud.points[index].x, x, e);
        EXPECT_NEAR(cloud.points[index].y, y, e);
        if (i < g_scan.intensities.size()) {
          EXPECT_EQ(cloud.points[index].i, g_scan.intensities[i]);
        }
      } else {
        EXPECT_TRUE(false); // LaserScan point not found in PointCloud
      }
    } else {
      EXPECT_EQ(INFINITY, r);
    }
  }
  if (ring_count > 0) {
    EXPECT_EQ(cloud.points.size() / ring_count, count); // Make sure that all points were converted to ranges
  }
}

// Verify that every LaserScan point is not default, and every point came from the PointCloud
void verifyScanDense(const PointCloud &cloud, uint16_t ring, bool intensity = true) {
  ASSERT_EQ(cloud.header.stamp, g_scan.header.stamp);
  EXPECT_EQ(cloud.header.frame_id, g_scan.header.frame_id);
  EXPECT_EQ(intensity ? g_scan.ranges.size() : 0, g_scan.intensities.size());
  for (size_t i = 0; i < g_scan.ranges.size(); i++) {
    double r = g_scan.ranges[i];
    if (std::isfinite(r)) {
      float a = g_scan.angle_min + i * g_scan.angle_increment;
      float x = g_scan.ranges[i] * cosf(a);
      float y = g_scan.ranges[i] * sinf(a);
      float e = g_scan.ranges[i] * g_scan.angle_increment + (float)1e-3; // allowable error
      size_t index = findClosestIndex(cloud, ring, x, y);
      if (index < cloud.points.size()) {
        EXPECT_NEAR(cloud.points[index].x, x, e);
        EXPECT_NEAR(cloud.points[index].y, y, e);
        ///@TODO: Test for matching intensity
      } else {
        EXPECT_TRUE(false); // LaserScan point not found in PointCloud
      }
    } else {
      EXPECT_TRUE(false); // Dense PointCloud should populate every range in LaserScan
    }
  }
}

// Verify that no LaserScan is generated when the PointCloud2 message is missing required fields
TEST(System, missing_fields)
{
  // Make sure system is connected
  ASSERT_EQ(1, g_sub.getNumPublishers());
  ASSERT_EQ(1, g_pub.getNumSubscribers());

  // Create PointCloud with 16 rings
  PointCloud cloud;
  cloud.points.resize(1);
  cloud.points[0].x = 0.0;
  cloud.points[0].y = 0.0;
  cloud.points[0].z = 0.0;
  cloud.points[0].i = 0.0;
  cloud.points[0].r = 15;

  // Verify no LaserScan when PointCloud2 fields are empty
  publishNone();
  EXPECT_FALSE(waitForScan(ros::WallDuration(0.5)));

  // Verify no LaserScan when PointCloud2 fields are missing 'ring'
  publishXYZ(cloud);
  EXPECT_FALSE(waitForScan(ros::WallDuration(0.5)));

  // Verify no LaserScan when PointCloud2 field 'ring' is the incorrect type
  publishXYZR32(cloud);
  EXPECT_FALSE(waitForScan(ros::WallDuration(0.5)));


  // Verify no LaserScan when PointCloud2 fields are missing 'x' and 'y'
  publishR(cloud);
  EXPECT_FALSE(waitForScan(ros::WallDuration(0.5)));

  // Verify that the node hasn't crashed by sending normal PointCloud2 fields
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR1(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  ASSERT_EQ(cloud.header.stamp, g_scan.header.stamp);
}

// Verify that non-point fields are passed through unmodified
TEST(System, empty_data)
{
  // Make sure system is connected
  ASSERT_EQ(1, g_sub.getNumPublishers());
  ASSERT_EQ(1, g_pub.getNumSubscribers());

  // Create PointCloud with 16 rings
  PointCloud cloud;
  cloud.header.frame_id = "abcdefghijklmnopqrstuvwxyz";
  cloud.points.resize(1);
  cloud.points[0].x = 0.0;
  cloud.points[0].y = 0.0;
  cloud.points[0].z = 0.0;
  cloud.points[0].i = 0.0;
  cloud.points[0].r = 15;

  // Verify that all three PointCloud2 types create proper default values

  // PointXYZIR (expected format)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR1(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanEmpty(cloud, true);

  // PointXYZIR (unexpected format with intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR2(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanEmpty(cloud, true);

  // PointXYZR (unexpected format without intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZR(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanEmpty(cloud, false);
}

// Verify that every piece of a small amount of random data is passed through
TEST(System, random_data_sparse)
{
  // Make sure system is connected
  ASSERT_EQ(1, g_sub.getNumPublishers());
  ASSERT_EQ(1, g_pub.getNumSubscribers());

  // Create PointCloud with sparse random data
  PointCloud cloud;
  cloud.header.frame_id = "velodyne";
  const size_t RANGE_COUNT = 100;
  const size_t RING_COUNT = 16;
  const double RANGE_MAX = 20.0;
  const double INTENSITY_MAX = 1.0;
  for (size_t i = 0; i < RANGE_COUNT; i++) {
    double angle_y = i * 1.99 * M_PI / RANGE_COUNT; // yaw
    for (size_t j = 0; j < RING_COUNT; j++) {
      double angle_p = j * 0.2 * M_PI / RING_COUNT - 0.1 * M_PI; // pitch
      double range = rand() * (RANGE_MAX / RAND_MAX);
      Point point;
      point.x = range * cos(angle_p) * cos(angle_y);
      point.y = range * cos(angle_p) * sin(angle_y);
      point.z = range * sin(angle_p);
      point.i = rand() * (INTENSITY_MAX / RAND_MAX);
      point.r = j;
      cloud.points.push_back(point);
    }
  }

  // Verify that all three PointCloud2 types are handled correctly

  // PointXYZIR (expected format)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR1(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanSparse(cloud, 8, RING_COUNT, true);

  // PointXYZIR (unexpected format with intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR2(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanSparse(cloud, 8, RING_COUNT, true);

  // PointXYZR (unexpected format without intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZR(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanSparse(cloud, 8, RING_COUNT, false);
}

// Verify that every LaserScan range is valid when given an extra large amount of random data
TEST(System, random_data_dense)
{
  // Make sure system is connected
  ASSERT_EQ(1, g_sub.getNumPublishers());
  ASSERT_EQ(1, g_pub.getNumSubscribers());

  // Create PointCloud with dense random data
  PointCloud cloud;
  cloud.header.frame_id = "velodyne";
  const size_t RANGE_COUNT = 2500;
  const size_t RING_COUNT = 16;
  const double RANGE_MAX = 20.0;
  const double INTENSITY_MAX = 1.0;
  for (size_t i = 0; i < RANGE_COUNT; i++) {
    double angle_y = i * 2.0 * M_PI / RANGE_COUNT; // yaw
    for (size_t j = 0; j < RING_COUNT; j++) {
      double angle_p = j * 0.2 * M_PI / RING_COUNT - 0.1 * M_PI; // pitch
      double range = rand() * (RANGE_MAX / RAND_MAX);
      Point point;
      point.x = range * cos(angle_p) * cos(angle_y);
      point.y = range * cos(angle_p) * sin(angle_y);
      point.z = range * sin(angle_p);
      point.i = rand() * (INTENSITY_MAX / RAND_MAX);
      point.r = j;
      cloud.points.push_back(point);
    }
  }

  // Verify that all three PointCloud2 types are handled correctly

  // PointXYZIR (expected format)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR1(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanDense(cloud, 8, true);

  // PointXYZIR (unexpected format with intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZIR2(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanDense(cloud, 8, true);

  // PointXYZR (unexpected format without intensity)
  cloud.header.stamp = rosTime(ros::WallTime::now());
  publishXYZR(cloud);
  ASSERT_TRUE(waitForScan(ros::WallDuration(1.0)));
  verifyScanDense(cloud, 8, false);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize ROS
  ros::init(argc, argv, "test_lazy_subscriber");
  ros::NodeHandle nh;

  // Setup publisher and subscriber
  g_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 2);
  g_sub = nh.subscribe("scan", 2, recv);

  // Wait for other nodes to startup
  ros::WallDuration(1.0).sleep();
  ros::spinOnce();

  // Run all the tests that were declared with TEST()
  return RUN_ALL_TESTS();
}
