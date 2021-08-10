# Camera Perception

## Introduction 
Apollo 7.0 camera obstacle add a new model based on [SMOKE](https://github.com/lzccccc/SMOKE). SMOKE is a Single-Stage monocular 3D object detection model which made some improvements for 3D vision on the CenterNet. We did some adaptation on the SMOKE and trained on [waymo open dateset](https://waymo.com/open/).Finally, our new model is added to Apollo as a new components.


## Architecture
Here we mainly focus on the modifications based on SMOKE,  more detail about SMOKE model please reference paper.
- Deformable conv can not convert onnx or libtorch. Therefore, we modify the deformable convolution in the backbone to normal convolution, which will lead to the decline of mAP;

- Because the 3D center points of some obstacles may appear outside the image, these obstacles will be filtered out during training, resulting in missed detection. Therefore, we take the center point of 2D bounding boxes to represent the obstacle, and add a head prediction offset term to recover the 3D center point;

- We add the head to predict the width and height of the 2D bounding box, and  directly calculate the 2D bbox of the obstacle with 2D center;

- Using 2D bounding box and other 3D information, we use post-processing geometric constraints to optimize the predicted position information. Firstly, we use the 3D information predicted by the model to calculate the 3D bounding box of the obstacle, as shown in Formula 1. $\theta$ represents the rotation of obstacle，$h,w,l$ is the dimensions and $x,y,z$ represent location。

<!-- $$
B = \left[\begin{matrix} \cos(\theta) & 0 & \sin(\theta) \\ 0 & 1 & 0 \\ -\sin(\theta) & 0 & \cos(\theta) \end{matrix} \right]
\left[\begin{matrix} \pm\frac{h}{2}  \\ \pm\frac{w}{2} \\ \pm\frac{l}{2} \end{matrix} \right] + 
\left[\begin{matrix} x  \\ y \\ z \end{matrix} \right]
\tag{1}
$$ -->

<div align=center>

<img src="https://latex.codecogs.com/svg.latex?B&space;=&space;\left[\begin{matrix}&space;\cos(\theta)&space;&&space;0&space;&&space;\sin(\theta)&space;\\&space;0&space;&&space;1&space;&&space;0&space;\\&space;-\sin(\theta)&space;&&space;0&space;&&space;\cos(\theta)&space;\end{matrix}&space;\right]&space;\left[\begin{matrix}&space;\pm\frac{h}{2}&space;\\&space;\pm\frac{w}{2}&space;\\&space;\pm\frac{l}{2}&space;\end{matrix}&space;\right]&space;&plus;&space;\left[\begin{matrix}&space;x&space;\\&space;y&space;\\&space;z&space;\end{matrix}&space;\right]" />

</div>

Then, according to the corresponding relationship between the bounding boxes as the constraint condition, we optimized the position information of the obstacle as shown in formula 2.

<!-- $$
x^*, y^*, z^* = arg\,\max_{\lbrace x,y,z \rbrace}{\sum{||B - B^*||^2_{\sum}}}
\tag{2}
$$ -->

<div align=center>

<img src="https://latex.codecogs.com/svg.latex?x^*,&space;y^*,&space;z^*&space;=&space;arg\,\max_{\lbrace&space;x,y,z&space;\rbrace}{\sum{||B&space;-&space;B^*||^2_{\sum}}}" />

</div>

The final network structure is shown below
<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/camera_network.png" alt="图片名称" width="60%" />
</div>

## Training
We trained model on the waymo open source dataset. Firstly, we used the conversion tool provided by the mmdetction3d framework to convert the waymo data into Kitti format. For specific operations, please refer to the [open mmlab documentation](https://github.com/open-mmlab/mmdetection3d/blob/master/docs/datasets/waymo_det.md). We only saved the front camera (image_ 0) data. Data conversion will take up a lot of space. Please ensure that your disk has enough space. After converting waymo data into Kitti format, we only need to make a few adjustments to the code to train and test. The test results on the waymo validation set are shown in the following table:

<div align=center>

|                 |  Car  | Pedestrian | Cyclist |                     
|-----------------|:-----:|:----------:|:-------:|
| mAP             |  6.88  |    0.35     |  0.32   |  
| bev             |  11.84 |     0.41    |   0.40  |

</div>

The visualize on waymo image data as follwos：

<div align=center>
<img src="../../../docs/specs/images/3d_obstacle_perception/smoke_example.png" alt="图片名称" width="40%" />
</div>

At the same time, we also provide the paddle training code of the model with the Baidu PaddlePaddle team, and please refer to the [SMOKE-Paddle](https://github.com/PaddlePaddle/models/tree/develop/PaddleCV/3d_vision/SMOKE) for more details.

## Online
Here, we use libtorch for online deployment and use the torch.jit.trace function of pytorch. We put camera internal parameters and image scaling coefficient into the model as parameters. For details, please refer to the code:
"modules/perception/camera/lib/obstacle/detector/smoke/smoke_obstacle_detector.cc"

## Launch
We provide a separate dag file to start the smoke obstacle detection model, which can be started by the following command:
```bash
mainboard -d modules/perception/production/dag/dag_streaming_obstacle_detection.dag
``` 

## Reference
- Liu, Zechen, Zizhang Wu, and Roland Tóth. "Smoke: single-stage monocular 3d object detection via keypoint estimation." In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition Workshops, pp. 996-997. 2020.

- {MMDetection3D: OpenMMLab} next-generation platform for general 3D object detection} https://github.com/open-mmlab/mmdetection3d