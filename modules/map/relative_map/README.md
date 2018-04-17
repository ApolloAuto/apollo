# Relative Map

## Introduction
Relative map module is a middle layer connects HDMap/Perception module and planning module. This module generates real-time relative map in the body coordinate system (FLU) and a reference line for planning. The inputs for relative map module have two parts: offline and online. The offline part is navigation line (human driving path) and the HDMap information on and near navigation line. And the online part is the traffic sign related information from perception module, e.g., lane marker, corsswalk, traffice light and etc. The generation of relative map leverages both online and offline part. It also works with either online or offline part only.    

## Inputs
  * NavigationInfo from dreamview module
  * LaneMarker from perception module
  * Localization from localization module

## Outputs
  * Relative map follows map format defined in modules/map/proto/map.proto
  * NavigationInfo received from dreamview module
