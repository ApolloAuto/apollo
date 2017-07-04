# Decision

## Introduction
  Decision module receives obstacles information (e.g., position, speed, 
  acceleration, prediction trajectory, and etc.), master vehicle status
  (e.g, position, speed, acceleration, and etc.), traffic light status,
  and map and routing information. Based on the aforementioned
  information, decision module generates decision command for master vehicle
  and each obstacle. It also generates virtual obstacles if necessary and 
  position them based on map information. 

## Input
  * Obstacles information from perception and prediction modules
  * Traffic lights status from perception module
  * Map and routing information
  * Master vehicle status

## Output
  * Decision command for master vehicle
  * Generated virtual obstacles
  * Decision command for each obstacle