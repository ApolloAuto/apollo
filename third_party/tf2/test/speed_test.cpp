/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <stdio.h>

#include <tf2/buffer_core.h>
#include "tf2/time.h"
#include <boost/lexical_cast.hpp>
#include <chrono>

using std::chrono::system_clock;
using std::chrono::steady_clock;
using std::chrono::high_resolution_clock;

int main(int argc, char** argv)
{
  int num_levels = 10;
  // if (argc > 1)
  // {
  //   num_levels = boost::lexical_cast<uint32_t>(argv[1]);
  // }

  tf2::BufferCore bc;
  geometry_msgs::TransformStamped t;
  t.header.stamp = 1;
  t.header.frame_id = "root";
  t.child_frame_id = "0";
  t.transform.translation.x = 1;
  t.transform.rotation.w = 1.0;
  bc.setTransform(t, "me");
  t.header.stamp = 2;
  bc.setTransform(t, "me");

  for (uint32_t i = 1; i < num_levels/2; ++i)
  {
    for (uint32_t j = 1; j < 3; ++j)
    {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      t.header.stamp = tf2::Time(j);
      t.header.frame_id = parent_ss.str();
      t.child_frame_id = child_ss.str();
      bc.setTransform(t, "me");
    }
  }

  t.header.frame_id = "root";
  std::stringstream ss;
  ss << num_levels/2;
  t.header.stamp = 1; 
  t.child_frame_id = ss.str();
  bc.setTransform(t, "me");
  t.header.stamp = 2;
  bc.setTransform(t, "me");

  for (uint32_t i = num_levels/2 + 1; i < num_levels; ++i)
  {
    for (uint32_t j = 1; j < 3; ++j)
    {
      std::stringstream parent_ss;
      parent_ss << (i - 1);
      std::stringstream child_ss;
      child_ss << i;

      t.header.stamp = tf2::Time(j);
      t.header.frame_id = parent_ss.str();
      t.child_frame_id = child_ss.str();
      bc.setTransform(t, "me");
    }
  }

  //logInfo_STREAM(bc.allFramesAsYAML());

  std::string v_frame0 = boost::lexical_cast<std::string>(num_levels - 1);
  std::string v_frame1 = boost::lexical_cast<std::string>(num_levels/2 - 1);
  printf("%s to %s\n", v_frame0.c_str(), v_frame1.c_str());
  geometry_msgs::TransformStamped out_t;

  const int count = 1000000;
  printf("Doing %d %d-level tests\n", count, num_levels);

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform(v_frame1, v_frame0, 0);
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("lookupTransform at Time(0) took: %f (secs) for an average of: %.9f (secs)\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::Time(1));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("lookupTransform at Time(1) took: %f for an average of: %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::Time(1.5));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("lookupTransform at Time(1.5) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      out_t = bc.lookupTransform(v_frame1, v_frame0, tf2::Time(2));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("lookupTransform at Time(2) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::Time(0));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("canTransform at Time(0) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::Time(1));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("canTransform at Time(1) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::Time(1.5 * 1e9));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("canTransform at Time(1.5) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif

#if 01
  {
    steady_clock::time_point start = steady_clock::now();
    for (uint32_t i = 0; i < count; ++i)
    {
      bc.canTransform(v_frame1, v_frame0, tf2::Time(2));
    }
    steady_clock::time_point end = steady_clock::now();
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
    printf("canTransform at Time(2) took %f for an average of %.9f\n", dur, dur / (double)count);
  }
#endif
}
