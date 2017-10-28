#!/usr/bin/env bash
CUDA_VISIBLE_DEVICES=0 python src/run_model.py \
    --type=predict \
    --steering_model_path=$1 \
    --acc_model_path=$2 \
    --input_dir=$3 \
    --output_path=$4 \
    --logtype="predict"
