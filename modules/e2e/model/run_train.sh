#!/usr/bin/env bash
CUDA_VISIBLE_DEVICES=0 python src/run_model.py \
    --type=train \
    --input_dir=./trainsets \
    --logtype=train
