# Relative Map

## Introduction
The relative map module receives navigation line, perception information, and location inforamtion from dreamview, perception, and localization module respectively. It generates realtime relative map (~10Hz), in which ADV location is always at (0,0) with heading of zero. 

The relative map is constructed in two ways: 
  * When navigation line is available: navigation line is used to generate lane central line and lane boundary is created based on lane marker information.
  * When navigation line is not available: lane markers are used to build lane boundaries and central line is calculated based on the boundaries. 

## Inputs
  * NavigationInfo from dreamview module
  * LaneMarker from perception module
  * Localization from localization module

## Outputs
  * Relative map follows map format defined in modules/map/proto/map.proto
  * NavigationInfo received from dreamview module
