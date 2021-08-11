# Lidar Perception

## Introduction
In Apollo 7.0, we added a new lidar obstacle detection model named Mask-Pillars. We add the attention module and pillar-level supervision on the basis of PointPillars. Finally, the model achieves higher performance than PointPillars on Kitti validation set, and has better recall on obstacles.

## Architecture
Here we mainly focus on the modifications based on PointPillars: 
### Attention Module
Although lidar can collect high-quality point cloud data, some obstacles may have a small number of point clouds due to occlusion or distance, so we add attention on FPN encoder module refer to [Residual Attention Network for Image Classification](https://arxiv.org/abs/1704.06904). Since FPN has three feature maps with different resolutions, our attention module also acts on three feature maps at the same time。More detail about network can see figure below，S represent Sigmoid，F function is shown as Formula 1：

$$
F(x) = (1 + M(x)) * T(x)
\tag{1}
$$
$T(x)$is the output of backbone，$M(x)$is the output of attention module.

### Pillar-level supervision
In order to improve the recall of the network, we try to add obstacle information to the network. We notice that the segmentation algorithm has a high recall rate, so we use the idea of segmentation network for reference. Finally, the feature map before the input detection module is supervised with the point cloud pillar supervision data, and the obstacle distribution information is put into the feature map by learning. Here, the generation method of supervision data is based on the original point cloud data. Firstly, find the point whose point cloud is not empty, then judge whether the point belongs to an obstacle, and generate the final training data by using Gaussian distribution.

The network structure of the final FPN is shown in the figure below
<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/lidar_network.png" alt="图片名称" width="60%" />
</div>

## Results
We use the mmdetction3d framework for training. On the KITTI validation set, the results are as follows. The results of PointPillars comes from [mmdetction3d](https://github.com/open-mmlab/mmdetection3d/blob/master/configs/pointpillars/README.md)
<div align=center>


|    Method           |   3DmAP <br> Mod.|  Car <br> Easy Mod. Hard| Pedestrian <br> Easy Mod. Hard | Cyclist <br> Easy Mod. Hard|              
|---------------------|:---------:|:-----:|:----------:|:-------:|:-------------------------------------------------------------------------------------------:|
| PointPillars        | 60.11           | 85.41     73.98	 67.76 |  52.02	      46.40        42.48| 78.72	   59.95	57.25|
| Ours                | 62.07           | 86.13     76.74	 74.14 |  50.79	      45.59	       41.50| 83.91	   63.87	61.05|

|     Method          |  BEVmAP <br> Mod.|  Car <br> Easy Mod. Hard| Pedestrian <br> Easy Mod. Hard | Cyclist <br> Easy Mod. Hard|              
|---------------------|:---------:|:-----:|:----------:|:-------:|:-------------------------------------------------------------------------------------------:|
| PointPillars        | 67.76           | 89.93     86.57	 85.20 |  59.08	      53.36	       48.42| 80.93	   63.34	60.06|
| Ours                | 69.49          | 89.85     87.15	 85.55 |  58.29	      53.87	       49.98| 85.13	   67.43	63.85|
</div>



The visualize on KITTI image data as fllows. It can be seen from the figure that our model has better detection performance.
<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/lidar_detection_compare.png" alt="图片名称" width="60%" />
</div>


## Online
Here, we use libtorch for online deployment and use the torch.jit.trace function of pytorch. We divide the original model into five parts for transformation, For details, please refer to the code：
```
"modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars.cc"
```

## Launch
In order to facilitate the expansion of Apollo model, we reconstruct the Apollo lidar perception module. We can choose to start different models by modifying the corresponding configuration file. The relevant configuration files are in the following path:
```
modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipeline/
```
There are multiple directories below, corresponding to different device names. Modify the keywords "detector" in "lidar_obstacle_detection.conf" file to switch the detection model.

## Reference
- MMDetection3D: OpenMMLab next-generation platform for general 3D object detection https://github.com/open-mmlab/mmdetection3d