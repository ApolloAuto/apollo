/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <tf2/time_cache.h>
#include <sys/time.h>
#include "tf2/LinearMath/Quaternion.h"
#include <stdexcept>

#include <geometry_msgs/TransformStamped.h>

#include <cmath>

std::vector<double> values;
unsigned int step = 0;

void seed_rand()
{
  values.clear();
  for (unsigned int i = 0; i < 1000; i++)
  {
    int pseudo_rand = std::floor(i * M_PI);
    values.push_back(( pseudo_rand % 100)/50.0 - 1.0);
    //printf("Seeding with %f\n", values.back());
  }
};


double get_rand() 
{ 
  if (values.size() == 0) throw std::runtime_error("you need to call seed_rand first");
  if (step >= values.size()) 
    step = 0;
  else
    step++;
  return values[step];
}

using namespace tf2;


void setIdentity(TransformStorage& stor)
{
  stor.translation_.setValue(0.0, 0.0, 0.0);
  stor.rotation_.setValue(0.0, 0.0, 0.0, 1.0);
}

TEST(TimeCache, Repeatability)
{
  unsigned int runs = 100;
  
  tf2::TimeCache  cache;

  TransformStorage stor;
  setIdentity(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    stor.frame_id_ = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }

  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
  }
  
}

TEST(TimeCache, RepeatabilityReverseInsertOrder)
{
  unsigned int runs = 100;
  
  tf2::TimeCache  cache;

  TransformStorage stor;
  setIdentity(stor);
  
  for ( int i = runs -1; i >= 0 ; i-- )
  {
    stor.frame_id_ = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }
  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
  }
  
}

#if 0    // jfaust: this doesn't seem to actually be testing random insertion?
TEST(TimeCache, RepeatabilityRandomInsertOrder)
{

  seed_rand();
  
  tf2::TimeCache  cache;
  double my_vals[] = {13,2,5,4,9,7,3,11,15,14,12,1,6,10,0,8};
  std::vector<double> values (my_vals, my_vals + sizeof(my_vals)/sizeof(double)); 
  unsigned int runs = values.size();

  TransformStorage stor;
  setIdentity(stor);
  for ( uint64_t i = 0; i <runs ; i++ )
  {
    values[i] = 10.0 * get_rand();
    std::stringstream ss;
    ss << values[i];
    stor.header.frame_id = ss.str();
    stor.frame_id_ = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }
  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
    std::stringstream ss;
    ss << values[i];
    EXPECT_EQ(stor.header.frame_id, ss.str());
  }
  
}
#endif

TEST(TimeCache, ZeroAtFront)
{
  uint64_t runs = 100;

  tf2::TimeCache  cache;

  TransformStorage stor;
  setIdentity(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    stor.frame_id_ = i;
    stor.stamp_ = ros::Time().fromNSec(i);
    
    cache.insertData(stor);
  }

  stor.frame_id_ = runs;
  stor.stamp_ = ros::Time().fromNSec(runs);
  cache.insertData(stor);

  for ( uint64_t i = 1; i < runs ; i++ )

  {
    cache.getData(ros::Time().fromNSec(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(i));
  }

  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs));

  stor.frame_id_ = runs;
  stor.stamp_ = ros::Time().fromNSec(runs+1);
  cache.insertData(stor);


  //Make sure we get a different value now that a new values is added at the front
  cache.getData(ros::Time(), stor);
  EXPECT_EQ(stor.frame_id_, runs);
  EXPECT_EQ(stor.stamp_, ros::Time().fromNSec(runs+1));
  
}

TEST(TimeCache, CartesianInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 2e-6;
  seed_rand();
  
  tf2::TimeCache  cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  uint64_t offset = 200;

  TransformStorage stor;
  setIdentity(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {

    for (uint64_t step = 0; step < 2 ; step++)
    {
      xvalues[step] = 10.0 * get_rand();
      yvalues[step] = 10.0 * get_rand();
      zvalues[step] = 10.0 * get_rand();
    
      stor.translation_.setValue(xvalues[step], yvalues[step], zvalues[step]);
      stor.frame_id_ = 2;
      stor.stamp_ = ros::Time().fromNSec(step * 100 + offset);
      cache.insertData(stor);
    }
    
    for (int pos = 0; pos < 100 ; pos ++)
    {
      cache.getData(ros::Time().fromNSec(offset + pos), stor);
      double x_out = stor.translation_.x();
      double y_out = stor.translation_.y();
      double z_out = stor.translation_.z();
      //      printf("pose %d, %f %f %f, expected %f %f %f\n", pos, x_out, y_out, z_out, 
      //       xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.,
      //       yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0,
      //       zvalues[0] + (xvalues[1] - zvalues[0]) * (double)pos/100.0);
      EXPECT_NEAR(xvalues[0] + (xvalues[1] - xvalues[0]) * (double)pos/100.0, x_out, epsilon);
      EXPECT_NEAR(yvalues[0] + (yvalues[1] - yvalues[0]) * (double)pos/100.0, y_out, epsilon);
      EXPECT_NEAR(zvalues[0] + (zvalues[1] - zvalues[0]) * (double)pos/100.0, z_out, epsilon);
    }
    

    cache.clearList();
  }

  
}

/** \brief Make sure we dont' interpolate across reparented data */
TEST(TimeCache, ReparentingInterpolationProtection)
{
  double epsilon = 1e-6;
  uint64_t offset = 555;

  seed_rand();

  tf2::TimeCache cache;
  std::vector<double> xvalues(2);
  std::vector<double> yvalues(2);
  std::vector<double> zvalues(2);

  TransformStorage stor;
  setIdentity(stor);

  for (uint64_t step = 0; step < 2 ; step++)
  {
    xvalues[step] = 10.0 * get_rand();
    yvalues[step] = 10.0 * get_rand();
    zvalues[step] = 10.0 * get_rand();

    stor.translation_.setValue(xvalues[step], yvalues[step], zvalues[step]);
    stor.frame_id_ = step + 4;
    stor.stamp_ = ros::Time().fromNSec(step * 100 + offset);
    cache.insertData(stor);
  }
  
  for (int pos = 0; pos < 100 ; pos ++)
  {
    EXPECT_TRUE(cache.getData(ros::Time().fromNSec(offset + pos), stor));
    double x_out = stor.translation_.x();
    double y_out = stor.translation_.y();
    double z_out = stor.translation_.z();
    EXPECT_NEAR(xvalues[0], x_out, epsilon);
    EXPECT_NEAR(yvalues[0], y_out, epsilon);
    EXPECT_NEAR(zvalues[0], z_out, epsilon);
  }
}

TEST(Bullet, Slerp)
{

  uint64_t runs = 100;
  seed_rand();

  tf2::Quaternion q1, q2;
  q1.setEuler(0,0,0);
  
  for (uint64_t i = 0 ; i < runs ; i++)
  {
    q2.setEuler(1.0 * get_rand(),
                1.0 * get_rand(),
                1.0 * get_rand());
    
    
    tf2::Quaternion q3 = slerp(q1,q2,0.5);
    
    EXPECT_NEAR(q3.angle(q1), q2.angle(q3), 1e-5);
  }

}


TEST(TimeCache, AngularInterpolation)
{
  uint64_t runs = 100;
  double epsilon = 1e-6;
  seed_rand();
  
  tf2::TimeCache  cache;
  std::vector<double> yawvalues(2);
  std::vector<double> pitchvalues(2);
  std::vector<double> rollvalues(2);
  uint64_t offset = 200;

  std::vector<tf2::Quaternion> quats(2);

  TransformStorage stor;
  setIdentity(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {

    for (uint64_t step = 0; step < 2 ; step++)
    {
      yawvalues[step] = 10.0 * get_rand() / 100.0;
      pitchvalues[step] = 0;//10.0 * get_rand();
      rollvalues[step] = 0;//10.0 * get_rand();
      quats[step].setRPY(yawvalues[step], pitchvalues[step], rollvalues[step]);
      stor.rotation_ = quats[step];
      stor.frame_id_ = 3;
      stor.stamp_ = ros::Time().fromNSec(offset + (step * 100)); //step = 0 or 1
      cache.insertData(stor);
    }
    
    for (int pos = 0; pos < 100 ; pos ++)
    {
      EXPECT_TRUE(cache.getData(ros::Time().fromNSec(offset + pos), stor)); //get the transform for the position
      tf2::Quaternion quat (stor.rotation_);

      //Generate a ground truth quaternion directly calling slerp
      tf2::Quaternion ground_truth = quats[0].slerp(quats[1], pos/100.0);
      
      //Make sure the transformed one and the direct call match
      EXPECT_NEAR(0, angle(ground_truth, quat), epsilon);
            
    }
    
    cache.clearList();
  }

  
}

TEST(TimeCache, DuplicateEntries)
{

  TimeCache cache;

  TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = 3;
  stor.stamp_ = ros::Time().fromNSec(1);

  cache.insertData(stor);

  cache.insertData(stor);


  cache.getData(ros::Time().fromNSec(1), stor);
  
  //printf(" stor is %f\n", stor.translation_.x());
  EXPECT_TRUE(!std::isnan(stor.translation_.x()));
  EXPECT_TRUE(!std::isnan(stor.translation_.y()));
  EXPECT_TRUE(!std::isnan(stor.translation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.x()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.y()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.w()));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
