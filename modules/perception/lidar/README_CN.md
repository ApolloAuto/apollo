# 激光雷达感知

## 介绍
在Apollo7.0版本中我们增加了一个新的激光雷达障碍物检测模型Mask-Pillars，在Pointpillars的基础上增加了attention模块和基于pillar-level数据的监督。最终模型在KITTI数据集上取得了高于PointPillars的指标，同时对障碍物有着更好的召回效果。

## 结构
这里我们主要针对在Pointpillars基础上进行的改进：
### 注意力机制
尽管激光雷达可以采集到高质量的点云数据，但是由于遮挡或距离因素部分障碍物可能点云数量较少，因此我们参考[Residual Attention Network for Image Classification](https://arxiv.org/abs/1704.06904)在FPN encoder部分添加了attention，由于FPN有3个不同分辨率的特征图，我们的注意力模块也同时作用在3个特征图上。具体的结构可以见下图1，图中S代表Sigmoid函数，F函数形式如公式1所示：

$$
F(x) = (1 + M(x)) * T(x)
\tag{1}
$$
其中$T(x)$为主干模块的输出，$M(x)$为注意力模块的输出

### Pillar-level 监督
为了进一步提高网络的召回率，我们尝试在网络中增加障碍物的信息，在这里我们借鉴了分割网络的思路，因为分割算法通常具有较高的召回率。最终用基于点云pillar监督数据，对输入检测模块之前的特征图进行额外的监督，使用学习的方式将障碍物分布信息放入特征图中。这里监督数据的生成方式基于原始的点云数据，首先找到点云不为空的点，然后判断该点是否属于障碍物，利用高斯分布生成最终的训练数据。

最终FPN部分的网络结构如下图所示
<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/lidar_network.png" alt="图片名称" width="60%" />
</div>

## 结果
我们使用mmdetction3d框架进行训练，在KITTI验证集上结果如下,PointPillars的模型指标来自于[mmdetction3d官方](https://github.com/open-mmlab/mmdetection3d/blob/master/configs/pointpillars/README.md)

<div align=center>

|    Method           |   3DmAP <br> Mod.|  Car <br> Easy Mod. Hard| Pedestrian <br> Easy Mod. Hard | Cyclist <br> Easy Mod. Hard|              
|---------------------|:---------:|:-----:|:----------:|:-------:|
| PointPillars        | 60.11           | 85.41     73.98	 67.76 |  52.02	      46.40        42.48| 78.72	   59.95	57.25|
| Ours                | 62.07           | 86.13     76.74	 74.14 |  50.79	      45.59	       41.50| 83.91	   63.87	61.05|

|     Method          |  BEVmAP <br> Mod.|  Car <br> Easy Mod. Hard| Pedestrian <br> Easy Mod. Hard | Cyclist <br> Easy Mod. Hard|              
|---------------------|:---------:|:-----:|:----------:|:-------:|
| PointPillars        | 67.76           | 89.93     86.57	 85.20 |  59.08	      53.36	       48.42| 80.93	   63.34	60.06|
| Ours                | 69.49          | 89.85     87.15	 85.55 |  58.29	      53.87	       49.98| 85.13	   67.43	63.85|

</div>



我们将检测结果进行了可视化，如下图所示。从图中可以看出我们的模型具有更好的检出效果：
<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/lidar_detection_compare.png" alt="图片名称" width="60%" />
</div>


## 线上部署
这里我们使用libtorch进行线上部署，利用pytorch的torch.jit.trace函数。我们把原来的模型分成5个部分进行转化，具体操作可以参考代码：
```
"modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars.cc"
```

## 启动
为了方便Apollo模型的拓展，我们对Apollo激光雷达感知模块进行了重构，只需要修改对应的配置文件就可以选择启动不同的模型。相关的配置文件在路径下：
```
modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipeline/
```
下面有多个目录，对应着不同的设备名称，修改目录下的"lidar_obstacle_detection.conf"配置文件的detector关键字即可切换检测模型。

## Reference
- MMDetection3D: OpenMMLab next-generation platform for general 3D object detection https://github.com/open-mmlab/mmdetection3d