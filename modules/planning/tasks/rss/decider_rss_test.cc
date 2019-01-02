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


#include "rss/core/RssCheck.hpp"
#include "rss/test_support/TestSupport.hpp"

namespace rss {
namespace core {

class RssCheckSameDirectionTests : public testing::Test {
 protected:
  virtual void SetUp() {
    scene.setSituationType(rss::situation::SituationType::SameDirection);
    leadingObject = createObject(0., 0.);
    leadingObject.objectId = 0;

    {
      ::rss::world::OccupiedRegion occupiedRegion;
      occupiedRegion.lonRange.minimum = 0.94;
      occupiedRegion.lonRange.maximum = 1.0;
      occupiedRegion.segmentId = 0;
      occupiedRegion.latRange.minimum = 0.;
      // occupiedRegion.latRange.maximum = 0.089;  // rss safe
      occupiedRegion.latRange.maximum = 0.15;  // rss unsafe
      leadingObject.occupiedRegions.push_back(occupiedRegion);
    }

    followingObject = createObject(11.0, 0.);
    followingObject.objectId = 1;
    {
      ::rss::world::OccupiedRegion occupiedRegion;
      occupiedRegion.lonRange.minimum = 0.7;
      occupiedRegion.lonRange.maximum = 0.82;
      occupiedRegion.segmentId = 0;
      occupiedRegion.latRange.minimum = 0.3;
      occupiedRegion.latRange.maximum = 0.66;
      followingObject.occupiedRegions.push_back(occupiedRegion);
    }

    {
      ::rss::world::RoadSegment roadSegment;
      ::rss::world::LaneSegment laneSegment;

      laneSegment.id = 0;
      laneSegment.length.minimum = 41.4;
      laneSegment.length.maximum = 41.4;
      laneSegment.width.minimum = 5.98;
      laneSegment.width.maximum = 5.98;

      roadSegment.push_back(laneSegment);
      roadArea.push_back(roadSegment);
    }
  }

  virtual void TearDown() {
    followingObject.occupiedRegions.clear();
    leadingObject.occupiedRegions.clear();
    scene.egoVehicleRoad.clear();
  }
  ::rss::world::Object followingObject;
  ::rss::world::Object leadingObject;
  ::rss::world::RoadArea roadArea;
  ::rss::world::Scene scene;
};

TEST_F(RssCheckSameDirectionTests, OtherLeading_FixDistance) {
  ::rss::world::WorldModel worldModel;

  worldModel.egoVehicle = followingObject;
  scene.object = leadingObject;

  scene.egoVehicleRoad = roadArea;
  worldModel.scenes.push_back(scene);
  worldModel.timeIndex = 1;

  ::rss::world::AccelerationRestriction accelerationRestriction;
  ::rss::core::RssCheck rssCheck;

  double dMin = calculateLongitudinalMinSafeDistance(worldModel.egoVehicle,
      worldModel.scenes[0].object);

  ASSERT_TRUE(rssCheck.calculateAccelerationRestriction(worldModel,
      accelerationRestriction));
  ASSERT_EQ(accelerationRestriction.longitudinalRange.minimum,
      -1. * worldModel.egoVehicle.dynamics.alphaLon.brakeMax);
  ASSERT_EQ(accelerationRestriction.longitudinalRange.maximum,
      -1. * worldModel.egoVehicle.dynamics.alphaLon.brakeMin);

  printf("dMin = %f\n", dMin);
  printf("accelerationRestriction.longitudinalRange.minimum=%f\n",
      accelerationRestriction.longitudinalRange.minimum);
  printf("accelerationRestriction.longitudinalRange.maximum=%f\n",
      accelerationRestriction.longitudinalRange.maximum);
}

}  // namespace core
}  // namespace rss
