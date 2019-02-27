/******************************************************************************
 * Copyright 2019 
 * wh : add this module for analyse of the obstacle information, 
 * which detected from the ultasonic sensors.
 *
 * IN: ultrasonic information: /apollo/canbus/chassis
 * OUT: obstacles information: /apollo/ultanalyse
 * 
 *****************************************************************************/

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "ros/include/ros/ros.h"

#include "modules/drivers/ultanalyse/ultanalyse.h"

APOLLO_MAIN(apollo::ultanalyse::Ultanalyse);
