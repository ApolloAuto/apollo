#!/bin/bash

pcd_path="./data/files/"
pose_path="./data/poses/"
output_path="./result_output/"

mkdir -p $output_path && rm -rf $output_path/*

./offline_lidar_obstacle_perception \
        --work_root=./ \
        --pcd_path=$pcd_path \
        --pose_path=$pose_path \
        --output_path=$output_path \
        --use_hdmap=false \
        --enable_tracking=true \
        --use_tracking_info=true \
        --min_life_time=-0.1 \
        --adu_data=./ \
        --log_dir=./logs \
        2>&1 | tee segment.log
