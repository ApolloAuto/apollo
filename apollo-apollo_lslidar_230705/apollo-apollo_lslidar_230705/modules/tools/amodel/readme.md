## amodel
amodel is Apollo's model deployment and management tool.

## Set environment
If you are running in Apollo docker, you can skip this step. If you are running outside of docker, the following environment needs to be set up.
```shell
export APOLLO_ROOT_DIR=your_apollo_dir
```

## How to work
amodel provides the following commands.
- list. Show models installed in Apollo.
- info. Show details of a model.
- install. Install the model to Apollo.
- remove. Remove the model from Apollo.

#### List
You can get the installed models in Apollo through the `list` command.
```shell
$ % python3 modules/tools/amodel/main.py list
Name                |Task_type           |Sensor_type         |Framework           |Date
mask_pillars        |3d_detection        |lidar               |paddlepaddle        |2021-07-30
center_point        |3d_detection        |lidar               |paddlepaddle        |2022-07-22
point_pillars       |3d_detection        |lidar               |paddlepaddle        |2020-12-15
cnnseg16            |3d_segmentation     |lidar               |paddlepaddle        |2018-10-14
cnnseg128           |3d_segmentation     |lidar               |paddlepaddle        |2020-06-17
cnnseg64            |3d_segmentation     |lidar               |paddlepaddle        |2019-05-29
smoke               |3d_detection        |camera              |paddlepaddle        |2019-06-27
3d-yolo             |3d_detection        |camera              |paddlepaddle        |2019-12-08
denseline           |lane_detection      |camera              |paddlepaddle        |2019-05-29
darkSCNN            |lane_detection      |camera              |paddlepaddle        |2020-12-15
tl_detection        |tl_detection        |camera              |paddlepaddle        |2021-01-15
tl_recognition      |tl_recognition      |camera              |paddlepaddle        |2021-01-15
```

#### Info
Then you can use the `info` command to learn more about the details of the model.
```shell
$ apollo % python3 modules/tools/amodel/main.py info point_pillars
name: point_pillars
date: 2020-12-15
task_type: 3d_detection
sensor_type: lidar
framework: paddlepaddle
model_files:
- name: pfe.onnx
  size: 4125
- name: pts_backbone.zip
  size: 16945051
- name: pts_bbox_head.zip
  size: 121150
- name: pts_middle_encoder.zip
  size: 3763
- name: pts_neck.zip
  size: 2420625
- name: pts_voxel_encoder.zip
  size: 17575
- name: rpn.onnx
  size: 18300546
dataset:
- waymo
- kitti
- nusense
```

#### Install
You can deploy the model using the `install` command.
```shell
# Install from local
python3 modules/tools/amodel/main.py install xxx.zip
# Install from http
python3 modules/tools/amodel/main.py install https://xxx.zip
```

#### Remove
You can delete models installed in Apollo with the `remove` command.
```shell
python3 modules/tools/amodel/main.py remove point_pillars
```
