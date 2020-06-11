#!/bin/bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/apollo/modules/perception/production/internal/lib/
cyber_launch start modules/perception/production/internal/launch/perception_viz.launch

