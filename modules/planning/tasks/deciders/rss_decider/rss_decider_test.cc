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

/**
 * @file
 **/

#include "TestSupport.hpp"
#include "ad_rss/core/RssCheck.hpp"

namespace ad_rss {
namespace core {

using ad_rss::world::OccupiedRegion;

class RssCheckSameDirectionTests : public testing::Test {
 protected:
  virtual void SetUp() {
    scene.situationType = ad_rss::situation::SituationType::SameDirection;
    // leadingObject = createObject(0., 0.);  // lateral speed 0
    leadingObject = createObject(0., 0.5);  // lateral speed toward right
    leadingObject.objectId = 0;

    {
      OccupiedRegion occupiedRegion;
      occupiedRegion.lonRange.minimum = ParametricValue(0.94);
      occupiedRegion.lonRange.maximum = ParametricValue(1.0);
      occupiedRegion.segmentId = 0;
      occupiedRegion.latRange.minimum = ParametricValue(0.);
      occupiedRegion.latRange.maximum = ParametricValue(0.089);  // rss safe
      // occupiedRegion.latRange.maximum = ParametricValue(0.15);  // rss unsafe
      // occupiedRegion.latRange.maximum = ParametricValue(0.3);  // rss unsafe
      leadingObject.occupiedRegions.push_back(occupiedRegion);
    }

    // followingObject = createObject(11.0, 0.);  // lateral speed 0
    followingObject = createObject(11.0, -0.5);  // lateral speed toward left
    followingObject.objectId = 1;
    {
      OccupiedRegion occupiedRegion;
      occupiedRegion.lonRange.minimum = ParametricValue(0.7);
      occupiedRegion.lonRange.maximum = ParametricValue(0.82);
      occupiedRegion.segmentId = 0;
      occupiedRegion.latRange.minimum = ParametricValue(0.3);
      occupiedRegion.latRange.maximum = ParametricValue(0.66);
      followingObject.occupiedRegions.push_back(occupiedRegion);
    }

    {
      world::RoadSegment roadSegment;
      world::LaneSegment laneSegment;

      laneSegment.id = 0;
      laneSegment.length.minimum = Distance(41.4);
      laneSegment.length.maximum = Distance(41.4);
      laneSegment.width.minimum = Distance(5.98);
      laneSegment.width.maximum = Distance(5.98);

      roadSegment.push_back(laneSegment);
      roadArea.push_back(roadSegment);
    }
  }

  virtual void TearDown() {
    followingObject.occupiedRegions.clear();
    leadingObject.occupiedRegions.clear();
    scene.egoVehicleRoad.clear();
  }
  ::ad_rss::world::Object followingObject;
  ::ad_rss::world::Object leadingObject;
  ::ad_rss::world::RoadArea roadArea;
  ::ad_rss::world::Scene scene;
};

TEST_F(RssCheckSameDirectionTests, OtherLeading_FixDistance) {
  ::ad_rss::world::WorldModel worldModel;

  worldModel.egoVehicle = ad_rss::objectAsEgo(followingObject);
  scene.object = leadingObject;

  scene.egoVehicleRoad = roadArea;
  worldModel.scenes.push_back(scene);
  worldModel.timeIndex = 1;

  ::ad_rss::world::AccelerationRestriction accelerationRestriction;
  ::ad_rss::core::RssCheck rssCheck;

  ASSERT_TRUE(rssCheck.calculateAccelerationRestriction(
      worldModel, accelerationRestriction));

  if (accelerationRestriction.longitudinalRange.maximum ==
      -1. * worldModel.egoVehicle.dynamics.alphaLon.brakeMin) {
    printf("[----------] RSS unsafe!!!\n");
    EXPECT_EQ(accelerationRestriction.longitudinalRange.maximum,
              -1. * worldModel.egoVehicle.dynamics.alphaLon.brakeMin);
  } else {
    printf("[----------] RSS safe!!!\n");
    EXPECT_EQ(accelerationRestriction.longitudinalRange.maximum,
              worldModel.egoVehicle.dynamics.alphaLon.accelMax);
  }
  // following judgement only true for leading-v v(0,0.5) and ego-v v(11,-0.5)
  EXPECT_EQ(accelerationRestriction.lateralLeftRange.maximum,
            -1 * worldModel.egoVehicle.dynamics.alphaLat.brakeMin);
  EXPECT_EQ(accelerationRestriction.lateralRightRange.maximum,
            worldModel.egoVehicle.dynamics.alphaLat.accelMax);
}

}  // namespace core
}  // namespace ad_rss
