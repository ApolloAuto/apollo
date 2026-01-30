目前，Apollo 中提供了以下几个 lidar 检测模型：centerpoint，pointpillars，maskpillars，cnnseg。

为了支持用户将自己训练的 lidar 检测模型部署到 apollo 中，本文将以 centerpoint 模型和 paddle3d 开源代码仓库为基础，介绍用户如何在 apollo 中新增模型，包括：paddle3d 的安装、训练配置介绍、centerpoint 的训练评测和导出，以及如何部署 centerpoint 到 apollo 中查看效果。

## CenterPoint训练

centerpoint 的完整训练流程可参考 AI Studio 项目：

- [【自动驾驶实战】基于Paddle3D&amp;Apollo的Lidar目标物检测-飞桨AI Studio星河社区](https://aistudio.baidu.com/projectdetail/6778231)

- [【感知赛事】使用ApolloScape数据集训练CenterPoint - 飞桨AI Studio星河社区](https://aistudio.baidu.com/projectdetail/7064946?contributionType=1)

### 1. 安装

- [训练代码地址](https://github.com/ApolloAuto/apollo-model-centerpoint)

- [安装教程](https://github.com/ApolloAuto/apollo-model-centerpoint/blob/master/docs/installation.md)

### 2. 训练配置介绍

接下来将以 centerpoint_pillars_016voxel_kitti.yml 为例，介绍训练配置。

#### 基础配置

| 配置名称           | 默认值 | 含义                |
| -------------- | --- | ----------------- |
| batch_size     | 4   | 训练时一个batch包含的点云帧数 |
| val_batch_size | 1   | 评测时一个batch包含的点云帧数 |
| epochs         | 160 | epoch个数           |

#### AMP配置 amp_cfg

自动混合精度（Automatic Mixed Precision，以下简称为 AMP）指的是在训练时使用fp16精度，提升计算速度并降低存储空间。详细介绍参见 [自动混合精度训练（AMP）](https://www.paddlepaddle.org.cn/documentation/docs/zh/guides/performance_improving/amp_cn.html)。

| 配置名称                    | 默认值   | 含义                     |
| ----------------------- | ----- | ---------------------- |
| use_amp                 | False | 是否开启amp训练              |
| enable                  | False | -                      |
| level                   | O1    | 混合精度训练的策略，可选项：O1，O2    |
| scalerinit_loss_scaling | 32.0  | 控制 loss 缩放比例，规避浮点数下溢问题 |

#### 数据集配置 train_dataset/val_datasets

<table>
<thead>
  <tr>
    <th colspan="3">配置名称</th>
    <th>默认值</th>
    <th>含义</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td colspan="3">type</td>
    <td>KittiPCDataset</td>
    <td>数据集类型</td>
  </tr>
  <tr>
    <td colspan="3">dataset_root</td>
    <td>datasets/KITTI</td>
    <td>数据集路径</td>
  </tr>
  <tr>
    <td rowspan="27">transforms</td>
    <td rowspan="2">LoadPointCloud</td>
    <td>dim</td>
    <td>4</td>
    <td>每个点的特征数，一般为x, y, z, intensity</td>
  </tr>
  <tr>
    <td>use_dim</td>
    <td>4</td>
    <td>使用每个点的前use_dim个特征</td>
  </tr>
  <tr>
    <td colspan="2">RemoveCameraInvisiblePointsKITTI</td>
    <td>-</td>
    <td>删除相机看不见区域的点云</td>
  </tr>
  <tr>
    <td rowspan="6">SamplingDatabase</td>
    <td>min_num_points_in_box_per_class</td>
    <td>Car: 5 Cyclist: 5 Pedestrian: 5</td>
    <td>每种类别目标的最少点数</td>
  </tr>
  <tr>
    <td>max_num_samples_per_class</td>
    <td>Car: 15 Cyclist: 10</td>
    <td>每帧点云中每种类别目标的最大个数</td>
  </tr>
  <tr>
    <td>ignored_difficulty</td>
    <td>[-1]</td>
    <td>困难程度限制</td>
  </tr>
  <tr>
    <td>database_anno_path</td>
    <td>datasets/KITTI/kitti_train_gt_database/anno_info_train.pkl</td>
    <td>anno_info_train.pkl的路径</td>
  </tr>
  <tr>
    <td>database_root</td>
    <td>datasets/KITTI/</td>
    <td>数据集的路径</td>
  </tr>
  <tr>
    <td>class_names</td>
    <td>["Car", "Cyclist", "Pedestrian"]</td>
    <td>类别信息</td>
  </tr>
  <tr>
    <td rowspan="3">RandomObjectPerturb</td>
    <td>rotation_range</td>
    <td>[-0.15707963267, 0.15707963267]</td>
    <td>gt随机旋转的角度范围</td>
  </tr>
  <tr>
    <td>translation_std</td>
    <td>[0.25, 0.25, 0.25]</td>
    <td>gt随机平移的标准差</td>
  </tr>
  <tr>
    <td>max_num_attempts</td>
    <td>100</td>
    <td>最大候选数量</td>
  </tr>
  <tr>
    <td>RandomVerticalFlip</td>
    <td></td>
    <td>-</td>
    <td>开启随机竖直翻转</td>
  </tr>
  <tr>
    <td rowspan="2">GlobalRotate</td>
    <td>min_rot</td>
    <td>-0.78539816</td>
    <td>全局旋转的最小旋转角</td>
  </tr>
  <tr>
    <td>max_rot</td>
    <td>0.78539816</td>
    <td>全局旋转的最大旋转角</td>
  </tr>
  <tr>
    <td rowspan="2">GlobalScale</td>
    <td>min_scale</td>
    <td>0.95</td>
    <td>全局缩放的最小缩放因子</td>
  </tr>
  <tr>
    <td>max_scale</td>
    <td>1.05</td>
    <td>全局缩放的最大缩放因子</td>
  </tr>
  <tr>
    <td>GlobalTranslate</td>
    <td>translation_std</td>
    <td>[0.2, 0.2, 0.2]</td>
    <td>全局平移的标准差</td>
  </tr>
  <tr>
    <td>ShufflePoint</td>
    <td></td>
    <td>-</td>
    <td>打乱点云的顺序</td>
  </tr>
  <tr>
    <td>FilterBBoxOutsideRange</td>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>过滤范围外的gt</td>
  </tr>
  <tr>
    <td rowspan="7">Gt2CenterPointTarget</td>
    <td>tasks</td>
    <td>- num_class: 1 class_names: ["Car"] - num_class: 2 class_names: ["Cyclist", "Pedestrian"]</td>
    <td>centerpoint的head设置</td>
  </tr>
  <tr>
    <td>down_ratio</td>
    <td>2</td>
    <td>下采样倍率</td>
  </tr>
  <tr>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td>voxel_size</td>
    <td>[0.16, 0.16, 4]</td>
    <td>voxel的尺寸</td>
  </tr>
  <tr>
    <td>gaussian_overlap</td>
    <td>0.1</td>
    <td>高斯重叠数值</td>
  </tr>
  <tr>
    <td>max_objs</td>
    <td>500</td>
    <td>一帧最多允许的gt数量</td>
  </tr>
  <tr>
    <td>min_radius</td>
    <td>2</td>
    <td>生成heatmap时每个gt的最小半径</td>
  </tr>
  <tr>
    <td colspan="3">mode</td>
    <td>train</td>
    <td>模式</td>
  </tr>
  <tr>
    <td colspan="3">class_balanced_sampling</td>
    <td>False</td>
    <td>开启类别采样</td>
  </tr>
  <tr>
    <td colspan="3">class_names</td>
    <td>["Car", "Cyclist", "Pedestrian"]</td>
    <td>类别信息</td>
  </tr>
</tbody>
</table>

SamplingDatabase

| 数据增强之前                                                                                                                                                   | 数据增强之后                                                                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ![数据增强之前.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%95%B0%E6%8D%AE%E5%A2%9E%E5%BC%BA%E4%B9%8B%E5%89%8D_fc166b5.png) | ![数据增强之后.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%95%B0%E6%8D%AE%E5%A2%9E%E5%BC%BA%E4%B9%8B%E5%90%8E_158f954.png) |

#### 优化器配置 optimizer

<table>
<thead>
  <tr>
    <th>配置名称</th>
    <th></th>
    <th>默认值</th>
    <th>含义</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td colspan="2">type</td>
    <td>OneCycleAdam</td>
    <td>优化器类型</td>
  </tr>
  <tr>
    <td colspan="2">beta2</td>
    <td>0.99</td>
    <td>beta2</td>
  </tr>
  <tr>
    <td colspan="2">weight_decay</td>
    <td>0.01</td>
    <td>衰减系数</td>
  </tr>
  <tr>
    <td rowspan="2">grad_clip</td>
    <td>type</td>
    <td>ClipGradByGlobalNorm</td>
    <td rowspan="2">梯度裁剪</td>
  </tr>
  <tr>
    <td>clip_norm</td>
    <td>35</td>
  </tr>
  <tr>
    <td rowspan="4">beta1</td>
    <td>type</td>
    <td>OneCycleDecayWarmupMomentum</td>
    <td rowspan="4">动量设置</td>
  </tr>
  <tr>
    <td>momentum_peak</td>
    <td>0.95</td>
  </tr>
  <tr>
    <td>momentum_trough</td>
    <td>0.85</td>
  </tr>
  <tr>
    <td>step_ratio_peak</td>
    <td>0.4</td>
  </tr>
</tbody>
</table>

#### 学习率设置 lr_scheduler

<table>
<thead>
  <tr>
    <th>配置名称</th>
    <th>默认值</th>
    <th>含义</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td>type</td>
    <td>OneCycleWarmupDecayLr</td>
    <td>学习率类型</td>
  </tr>
  <tr>
    <td>base_learning_rate</td>
    <td>0.001</td>
    <td>初始学习率</td>
  </tr>
  <tr>
    <td>lr_ratio_peak</td>
    <td>10</td>
    <td>最大学习率的比例因子</td>
  </tr>
  <tr>
    <td>lr_ratio_trough</td>
    <td>0.0001</td>
    <td>最小学习率的比例因子</td>
  </tr>
  <tr>
    <td>step_ratio_peak</td>
    <td>0.4</td>
    <td>到达最大值的step比例</td>
  </tr>
</tbody>
</table>

![learningrate.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/learningrate_5954540.png)

#### 模型配置 model

<table>
<thead>
  <tr>
    <th colspan="2">配置名称</th>
    <th>默认值</th>
    <th>含义</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td colspan="2">type</td>
    <td>CenterPoint</td>
    <td>模型名称</td>
  </tr>
  <tr>
    <td rowspan="5">voxelizer</td>
    <td>type</td>
    <td>HardVoxelizer</td>
    <td>体素化方法</td>
  </tr>
  <tr>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td>voxel_size</td>
    <td>[0.16, 0.16, 4]</td>
    <td>voxel尺寸</td>
  </tr>
  <tr>
    <td>max_num_points_in_voxel</td>
    <td>100</td>
    <td>每个voxel中最多的点数</td>
  </tr>
  <tr>
    <td>max_num_voxels</td>
    <td>[12000, 40000]</td>
    <td>训练/推理时voxel数量</td>
  </tr>
  <tr>
    <td rowspan="7">voxel_encoder</td>
    <td>type</td>
    <td>PillarFeatureNet</td>
    <td>voxel_encoder名称</td>
  </tr>
  <tr>
    <td>in_channels</td>
    <td>4</td>
    <td>输入的channel数量</td>
  </tr>
  <tr>
    <td>feat_channels</td>
    <td>[64, 64]</td>
    <td>fc层的输出channel</td>
  </tr>
  <tr>
    <td>with_distance</td>
    <td>False</td>
    <td>是否使用distance特征</td>
  </tr>
  <tr>
    <td>max_num_points_in_voxel</td>
    <td>100</td>
    <td>每个voxel中最多的点数</td>
  </tr>
  <tr>
    <td>voxel_size</td>
    <td>[0.16, 0.16, 4]</td>
    <td>voxel尺寸</td>
  </tr>
  <tr>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td rowspan="4">middle_encoder</td>
    <td>type</td>
    <td>PointPillarsScatter</td>
    <td>middle_encoder名称</td>
  </tr>
  <tr>
    <td>in_channels</td>
    <td>64</td>
    <td>输入channel数量</td>
  </tr>
  <tr>
    <td>voxel_size</td>
    <td>[0.16, 0.16, 4]</td>
    <td>voxel尺寸</td>
  </tr>
  <tr>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td rowspan="5">backbone</td>
    <td>type</td>
    <td>SecondBackbone</td>
    <td>backbone名称</td>
  </tr>
  <tr>
    <td>in_channels</td>
    <td>64</td>
    <td>输入channel</td>
  </tr>
  <tr>
    <td>out_channels</td>
    <td>[64, 128, 256]</td>
    <td>每个stage的输出channel</td>
  </tr>
  <tr>
    <td>layer_nums</td>
    <td>[3, 5, 5]</td>
    <td>每个stage的卷积层个数</td>
  </tr>
  <tr>
    <td>downsample_strides</td>
    <td>[1, 2, 2]</td>
    <td>每个stage的降采样步长</td>
  </tr>
  <tr>
    <td rowspan="5">neck</td>
    <td>type</td>
    <td>SecondFPN</td>
    <td>neck名称</td>
  </tr>
  <tr>
    <td>in_channels</td>
    <td>[64, 128, 256]</td>
    <td>输入channel</td>
  </tr>
  <tr>
    <td>out_channels</td>
    <td>[128, 128, 128]</td>
    <td>输出channel</td>
  </tr>
  <tr>
    <td>upsample_strides</td>
    <td>[0.5, 1, 2]</td>
    <td>上采样步长</td>
  </tr>
  <tr>
    <td>use_conv_for_no_stride</td>
    <td>True</td>
    <td>是否在未上采样处使用卷积层</td>
  </tr>
  <tr>
    <td rowspan="6">bbox_head</td>
    <td>type</td>
    <td>CenterHead</td>
    <td>head名称</td>
  </tr>
  <tr>
    <td>in_channels</td>
    <td>384</td>
    <td>输入channel</td>
  </tr>
  <tr>
    <td>tasks</td>
    <td>- num_class: 1class_names: ["Car"]- num_class: 2class_names: ["Cyclist", "Pedestrian"]</td>
    <td>head设置</td>
  </tr>
  <tr>
    <td>weight</td>
    <td>0.25</td>
    <td>定位损失的权重</td>
  </tr>
  <tr>
    <td>code_weights</td>
    <td>[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]</td>
    <td>8种定位属性的权重</td>
  </tr>
  <tr>
    <td>common_heads</td>
    <td>reg: [2, 2]height: [1, 2]dim: [3, 2]rot: [2, 2]</td>
    <td>4种head的设置</td>
  </tr>
  <tr>
    <td rowspan="8">test_cfg</td>
    <td>post_center_limit_range</td>
    <td>[-10., -50., -10., 80., 50., 10.]</td>
    <td>dt center的范围</td>
  </tr>
  <tr>
    <td>nms_pre_max_size</td>
    <td>1000</td>
    <td>nms之前的dt数量</td>
  </tr>
  <tr>
    <td>nms_post_max_size</td>
    <td>83</td>
    <td>nms之后的dt数量</td>
  </tr>
  <tr>
    <td>nms_iou_threshold</td>
    <td>0.1</td>
    <td>nms iou阈值</td>
  </tr>
  <tr>
    <td>score_threshold</td>
    <td>0.1</td>
    <td>置信度阈值</td>
  </tr>
  <tr>
    <td>point_cloud_range</td>
    <td>[0, -39.68, -3, 69.12, 39.68, 1]</td>
    <td>点云范围</td>
  </tr>
  <tr>
    <td>down_ratio</td>
    <td>2</td>
    <td>下采样倍率</td>
  </tr>
  <tr>
    <td>voxel_size</td>
    <td>[0.16, 0.16, 4]</td>
    <td>voxel尺寸</td>
  </tr>
</tbody>
</table>

### 3. 训练流程介绍

接下来以 KITTI 数据集为例，训练 CenterPoint 模型。

#### 数据准备

1. 首先请在 [官网](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) 进行下载相关数据
   
   - Download Velodyne point clouds, if you want to use laser information (29 GB)
   
   - training labels of object data set (5 MB)
   
   - camera calibration matrices of object data set (16 MB)

2. 下载数据集的划分文件列表：
   
   ```bash
   wget https://bj.bcebos.com/paddle3d/datasets/KITTI/ImageSets.tar.gz
   ```

3. 将数据解压后按照下方的目录结构进行组织：
   
   ```bash
   kitti_dataset_root
    |—— training
    |   |—— label_2
    |   |   |—— 000001.txt
    |   |   |—— ...
    |   |—— calib
    |   |   |—— 000001.txt
    |   |   |—— ...
    |   |—— velodyne
    |   |   |—— 000001.bin
    |   |   |—— ...
    |—— ImageSets
    │   |—— test.txt
    │   |—— train.txt
    │   |—— trainval.txt
    │   |—— val.txt
   ```

4. 在 Paddle3D 的目录下创建软链接`datasets/KITTI`，指向到上面的数据集目录：
   
   ```bash
   mkdir datasets
    ln -s /path/to/kitti_dataset_root ./datasets
    mv ./datasets/kitti_dataset_root ./datasets/KITTI
   ```

5. 生成训练时数据增强所需的真值库：
   
   ```bash
   python tools/create_det_gt_database.py --dataset_name kitti --dataset_root ./datasets/KITTI --save_dir ./datasets/KITTI
   ```
   
   `-dataset_root`指定 KITTI 数据集所在路径，`--save_dir`指定用于保存所生成的真值库的路径。该命令执行后，`save_dir`生成的目录如下：
   
   ```bash
   kitti_train_gt_database
    |—— anno_info_train.pkl
    |—— Car
    |   |—— 4371_Car_7.bin
    |   |—— ...
    |—— Cyclist
   ```

> 注意：如果想用自己的数据集进行训练，推荐将自己的数据集组织为 KITTI 标准格式。

#### 训练

KITTI 数据集上的训练使用 8 张 GPU：

```bash
python -m paddle.distributed.launch --gpus 0,1,2,3,4,5,6,7 tools/train.py --config configs/centerpoint/centerpoint_pillars_016voxel_kitti.yml --save_dir ./output_kitti --num_workers 4 --save_interval 5
```

训练启动参数介绍：

| 参数名                 | 用途                                                                                                 | 是否必选项 | 默认值          |
| ------------------- | -------------------------------------------------------------------------------------------------- | ----- | ------------ |
| iters               | 训练迭代步数                                                                                             | 否     | 配置文件中指定值     |
| epochs              | 训练迭代次数                                                                                             | 否     | 配置文件中指定值     |
| batch_size          | 单卡batch size                                                                                       | 否     | 配置文件中指定值     |
| learning_rate       | 初始学习率                                                                                              | 否     | 配置文件中指定值     |
| config              | 配置文件路径                                                                                             | 是     | -            |
| save_dir            | 检查点（模型和visualdl日志文件）的保存根路径                                                                         | 否     | output       |
| num_workers         | 用于异步读取数据的进程数量， 大于等于1时开启子进程读取数据                                                                     | 否     | 2            |
| save_interval       | 模型保存的间隔步数                                                                                          | 否     | 1000         |
| do_eval             | 是否在保存模型时启动评估                                                                                       | 否     | 否            |
| log_interval        | 打印日志的间隔步数                                                                                          | 否     | 10           |
| resume              | 是否从检查点中恢复训练状态                                                                                      | 否     | None         |
| keep_checkpoint_max | 最多保存模型的数量                                                                                          | 否     | 5            |
| quant_config        | 量化配置文件，一般放在[configs/quant](https://github.com/PaddlePaddle/Paddle3D/tree/develop/configs/quant)目录下 | 否     | None         |
| seed                | Paddle/numpy/random的全局随机种子值                                                                        | 否     | None         |
| model               | 基于预训练模型进行finetune                                                                                  | 否     | .pdparam模型文件 |

#### 评估

```bash
python tools/evaluate.py --config configs/centerpoint/centerpoint_pillars_016voxel_kitti.yml --model ./output_kitti/epoch_160/model.pdparams --batch_size 1 --num_workers 4
```

> 注意：CenterPoint 的评估目前只支持 batch_size 为 1。评估启动参数介绍如下所示：

| 参数名          | 用途                                                                                                 | 是否必选项 | 默认值      |
| ------------ | -------------------------------------------------------------------------------------------------- | ----- | -------- |
| batch_size   | 单卡batch size                                                                                       | 否     | 配置文件中指定值 |
| config       | 配置文件路径                                                                                             | 是     | -        |
| model        | 模型参数文件的路径                                                                                          | 是     | -        |
| num_workers  | 用于异步读取数据的进程数量， 大于等于1时开启子进程读取数据                                                                     | 否     | 2        |
| quant_config | 量化配置文件，一般放在[configs/quant](https://github.com/PaddlePaddle/Paddle3D/tree/develop/configs/quant)目录下 | 否     | None     |

#### 训练过程可视化

Paddle3D 使用 VisualDL 来记录训练过程中的指标和数据，我们可以在训练过程中，在命令行使用 VisualDL 启动一个 server，并在浏览器查看相应的数据

```bash
# logdir需要和训练脚本中指定的save_dir保持一致
visualdl --logdir output
```

![可视化1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%8F%AF%E8%A7%86%E5%8C%961_f5cf08e.png)

### 4. 导出模型介绍

运行以下命令，将训练时保存的动态图模型文件导出成推理引擎能够加载的静态图模型文件。

```bash
python tools/export.py --config configs/centerpoint/centerpoint_pillars_016voxel_kitti.yml --model /path/to/model.pdparams --save_dir /path/to/output
```

导出脚本参数介绍

| 参数名               | 用途                                          | 是否必选项 | 默认值            |
| ----------------- | ------------------------------------------- | ----- | -------------- |
| config            | 配置文件路径                                      | 是     | -              |
| model             | 模型参数路径                                      | 否     | -              |
| export_for_apollo | 是否用于Apollo部署，当打开该开关时，会同步生成用于Apollo部署的meta文件 | 否     | False          |
| save_dir          | 推理模型文件的保存路径                                 | 否     | exported_model |
| save_name         | 推理模型文件的保存名字                                 | 否     | None(由各模型自定决定) |
| quant_config      | 量化配置文件，如果模型使用量化训练，则在模型导出时同样需要指定量化配置文件       | 否     |                |

运行完成之后，会输出如下的文件：

```bash
|-- centerpoint_pillars_016voxel_kitti
    |-- apollo_deploy.yaml
    |-- centerpoint.pdiparams
    |-- centerpoint.pdiparams.info
    |-- centerpoint.pdmodel
```

## 部署 CenterPoint 到 Apollo 中

接下来介绍如何将导出的 centerpoint 模型部署到 apollo 中，使用 centerpoint 模型进行 3d 目标检测：

### 1. CenterPoint部署配置

首先介绍一下 centerpoint 部署的配置，配置文件路径：`/apollo/modules/perception/lidar_detection/data/center_point_param.pb.txt`。

```bash
# center point params
info {
  name: "center_point"
  version: ""
  dataset: "apollo"
  task_type: Detection3D
  sensor_type: Lidar
  framework: PaddlePaddle
  proto_file {
    file: "centerpoint.pdmodel"
  }
  weight_file {
    file: "centerpoint.pdiparams"
  }
  inputs {name: "data"}
  outputs {name: "_generated_var_9"}
  outputs {name: "_generated_var_10"}
  outputs {name: "_generated_var_11"}
}
preprocess {
  gpu_id: 0
  normalizing_factor: 255
  num_point_feature: 4
  enable_ground_removal: false
  ground_removal_height: -1.5
  enable_downsample_beams: false
  downsample_beams_factor: 1
  enable_downsample_pointcloud: true
  downsample_voxel_size_x: 0.09
  downsample_voxel_size_y: 0.09
  downsample_voxel_size_z: 0.09
  enable_fuse_frames: false
  num_fuse_frames: 5
  fuse_time_interval: 0.5
  enable_shuffle_points: false
  max_num_points: 2147483647
  reproduce_result_mode: false
  enable_roi_outside_removal: false
}
postprocess {
  score_threshold: 0.25
  num_output_box_feature: 7
  bottom_enlarge_height: 0.25
  top_enlarge_height: 0.25
  width_enlarge_value: 0
  length_enlarge_value: 0
}
paddle_settings {
  use_trt: false
  trt_precision: 1
  trt_use_static: true
  use_calibration: true
  collect_shape_info: false
  use_dynamicshape: true
  dynamic_shape_file: "modules/perception/lidar_detection/data/center_point_paddle/collect_shape_info_3lidar_20.pbtxt"
}
point2box_max_num: 5
quantize: 0.2
```

ModelInfo 配置，文件路径：`modules/perception/common/proto/model_info.proto`。

| 参数类型   | 参数名                    | 默认值 | 含义                 |
| ------ | ---------------------- | --- | ------------------ |
| string | name                   | /   | 模型名称，同models/下文件夹名 |
| string | framework              | /   | 模型推理框架             |
| string | ModelFile.proto_file   | /   | 模型网络结构             |
| string | ModelFile.weight_file  | /   | 模型权重文件             |
| string | ModelFile.anchors_file | /   | anchor size        |
| string | ModelBlob.inputs       | /   | 模型输入数据名称及维度        |
| int32  | ModelBlob.outputs      | /   | 模型输出数据名称及维度        |

PointCloudPreProcess：

| 参数类型   | 参数名                          | 默认值        | 含义                                      |
| ------ | ---------------------------- | ---------- | --------------------------------------- |
| int32  | gpu_id                       | 0          | GPU的id                                  |
| double | normalizing_factor           | 255        | 强度归一化的缩放因子                              |
| int32  | num_point_feature            | 4          | 每个点的特征数量                                |
| bool   | enable_ground_removal        | false      | 是否过滤掉地面点                                |
| double | ground_removal_height        | -1.5       | 过滤掉z值小于阈值的点                             |
| bool   | enable_downsample_beams      | false      | 是否根据beam id对点云进行过滤                      |
| int32  | downsample_beams_factor      | 4          | 保留beam id为downsample_beams_factor的倍数的点云 |
| bool   | enable_downsample_pointcloud | false      | 是否根据voxel过滤点云                           |
| double | downsample_voxel_size_x      | 0.01       | 过滤时voxel的x方向长度                          |
| double | downsample_voxel_size_y      | 0.01       | 过滤时voxel的y方向长度                          |
| double | downsample_voxel_size_z      | 0.01       | 过滤时voxel的z方向长度                          |
| bool   | enable_fuse_frames           | false      | 是否融合多帧点云                                |
| int32  | num_fuse_frames              | 5          | 融合点云的帧数                                 |
| double | fuse_time_interval           | 0.5        | 融合点云的时间间隔                               |
| bool   | enable_shuffle_points        | false      | 是否打乱点云索引                                |
| int32  | max_num_points               | 2147483647 | 允许的最大点云数量                               |
| bool   | reproduce_result_mode        | false      | 是否开启复现结果模式                              |
| bool   | enable_roi_outside_removal   | false      | 是否在输入模型之前将roi外的点云进行过滤                   |

PointCloudPostProcess

| 参数类型  | 参数名                    | 默认值  | 含义               |
| ----- | ---------------------- | ---- | ---------------- |
| float | score_threshold        | 0.5  | 置信度阈值            |
| float | nms_overlap_threshold  | 0.5  | NMS的iou阈值        |
| int32 | num_output_box_feature | 7    | 输出障碍物的属性个数       |
| float | bottom_enlarge_height  | 0.25 | 获取目标真实点云时向上扩充的范围 |
| float | top_enlarge_height     | 0.25 | 获取目标真实点云时向下扩充的范围 |
| float | width_enlarge_value    | 0    | 获取目标真实点云时宽度扩充的范围 |
| float | length_enlarge_value   | 0    | 获取目标真实点云时长度扩充的范围 |

centerpoint 独有的配置

| 参数类型                  | 参数名               | 默认值 | 含义                |
| --------------------- | ----------------- | --- | ----------------- |
| ModelInfo             | info              | /   | 模型通用配置            |
| PointCloudPreProcess  | preprocess        | /   | 预处理               |
| PointCloudPostProcess | postprocess       | /   | 后处理               |
| int32                 | point2box_max_num | 5   | 每个点最多可以属于多少个box   |
| float                 | quantize          | 0.2 | 将尺寸量化为quantize的倍数 |

### 2. 部署apollo并查看效果

apollo 的环境搭建参见 [perception 2.0 综述](https://apollo.baidu.com/community/article/1133)。

#### 进入 Docker 环境

```bash
# 进入容器
aem enter

# 下载安装依赖包： 会拉取安装core目录下的cyberfile.xml里面所有的依赖包
# 同时会自动下载centerpoint模型并放置到指定位置
buildtool build --gpu
```

上述步骤部署的是 apollo 官方的 centerpoint 模型，如果要部署自己的 centerpoint 模型：需要将上一步中导出的文件放置到`/apollo/modules/perception/data/models/center_point_paddle`下面即可：

```bash
|-- /apollo/modules/perception/data/models/center_point_paddle
    |-- apollo_deploy.yaml
    |-- centerpoint.pdiparams
    |-- centerpoint.pdiparams.info
    |-- centerpoint.pdmodel
```

#### 启动Dreamview+

```bash
aem bootstrap start --plus
```

#### 启动lidar感知程序

选择相应车型配置：

![选择相应车型配置.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E9%80%89%E6%8B%A9%E7%9B%B8%E5%BA%94%E8%BD%A6%E5%9E%8B%E9%85%8D%E7%BD%AE_3bb824c.png)

启动 transform 模块：

![启动transform模块.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%90%AF%E5%8A%A8transform%E6%A8%A1%E5%9D%97_6f4ffc3.png)

启动 lidar 感知模块：

![启动lidar模块.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E5%90%AF%E5%8A%A8lidar%E6%A8%A1%E5%9D%97_4d8bb34.png)

#### 下载并播放感知包

- 方式一：在 dreamview+ 左下角点击 Resource Manger，下载 sensor_rgb 数据包，下载完成后选择 sensor_rgb 数据包点击播放。
  
  ![方式一播包1.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%96%B9%E5%BC%8F%E4%B8%80%E6%92%AD%E5%8C%851_cbbbcd5.png)
  
  ![方式一播包2.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%96%B9%E5%BC%8F%E4%B8%80%E6%92%AD%E5%8C%852_953aab2.png)

- 方式二：终端启动下载好的 sensor_rgb 数据包，下载链接参考开放资源record下载。
  
  ```bash
  cyber_recorder play -f sensor_rgb.record
  ```
  
  dreamview+ 查看 lidar 检测结果：
  
  ![播放效果.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/%E6%92%AD%E6%94%BE%E6%95%88%E6%9E%9C_f6ef232.png)
