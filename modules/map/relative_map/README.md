# Relative Map

## Introduction
The relative map module receives navigation line, perception information, and location inforamtion from dreamview, perception, and localization module respectively. It generates realtime relative map (~10Hz), in which ADV location is always at (0,0) with heading of zero. 

## Inputs
  * NavigationInfo from dreamview module
  * LaneMarker from perception module
  * Localization from localization module

## Outputs
  * Relative map follows map format defined in modules/map/proto/map.proto
  * NavigationInfo recieved from dreamview module
