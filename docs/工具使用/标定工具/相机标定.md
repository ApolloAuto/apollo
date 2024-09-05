## 概述

通过标定板对相机外参进行精准标定，标定后，可得到相机坐标系相对于激光雷达坐标系的位置信息和姿态信息。


## 模块启动依赖


标定工具依赖定位 Localization 以及 gnss、liar、camera 驱动，标定前请您确保相关依赖正确安装配置。


## 注意事项

标定环境：为了标定板识别的准确性和成功率，测试时避免强光照，尽量选择有遮光的地方，或者早晨和傍晚进行测试。


## 操作步骤

### 步骤一：选择模式

在 Dreamview+ 页面，选择 **Camera Calibration** 模式。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_0baa215.png)

### 步骤二：环境监测

1. 选择相机。此处以 front_12mm 相机为例。

   选择相机后，开始进行当前相机采集环境检测。

2. 当显示 Camera 状态和 Lidar 状态均正常时，可以进行下一步操作。



### 步骤三：数据采集

#### 1. 固定标定板。

  ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4687df0.png)

  关于标定版的型号选择及购买链接，参见 [标定版型号选择](Apollo_Studio/本地工具台/附录.md)。

  > 注意：
  > - 标定板固定⽅式：通过⽀架调整⾼度，与摄像头保持⽔平，达到“悬浮”空中的效果，标定板边缘没有其他物体。
  > - 标定时一律采用支架将标定板架起，环境中避免出现和标定板形状类似的物体，测试时标定板摆放位置半径 1.5m 内不要有其它物体，标定板的姿态如图所示（标定板斜着用支架夹起）。采集时需要保证标定板完全静止，不要出现抖动。
  > - 通过实时图像来保证整个标定板都在图像视野范围内。

#### 2. 单击 "开始采集"

#### 3. 待相机成像图框中显示图像后，单击 "采集"

  ![a293403e60c04593aa8c90eb3.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/a293403e60c04593aa8c90eb3_a293403.jpg)

  > 注意：
  > - 通过实时的点云，保证落在标定板上的点云线束最少为 4 根，在距离相机不同的两个距离（具体距离参考下表）进行采集，每个距离分别在图像的左边缘，中间和右边缘进行摆放。
  > - 图像建议最多采集 20 张，低于 1 张不能提交。考虑采集效果，最好采满 6 张。

  |  | 鱼眼相机 | 2.3mm相机 | 6mm相机 | 12mm相机 |
  |---|---|---|---|---|
  | 16线激光雷达 | 1m、2m | 1.5m、2.5m | 2.5m、3.5m | 3m、5m |
  | 32线激光雷达 | 1m、2m | 2m、3.5m | 3m、4.5m | 4m 、6m |
  | 64线及以上线束激光雷达 | 1m、2m | 2m、3.5m | 4m、6m | 5m、7m |

#### 4. 选取采集正常的图像，单击 "保留"

   ![2524532429cb24083ae4d43bf.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/2524532429cb24083ae4d43bf_2524532.jpg)

   单张图片效果评价标准：所有的点均在标定板上，线与线之间大致平行，显示的线和点云中看到的标定板上的线相同，示例效果见下图：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_560d84f.png)

#### 5. 选取 6 张图像，并单击 "下一步"

   ![8cffa7849ba623c40bb2e36f1.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/8cffa7849ba623c40bb2e36f1_8cffa78.jpg)

   > 注意：建议采集六张图片后，可以先进行标定求解，若效果不佳，可以返回继续采集图片。

### 步骤四：外参标定

1. 查看图像和点云的融合结果。

   下图是点云投影到二维图像上的效果，彩色的离散点表示点云，不同的颜色代表距离 lidar 中心不同的距离，根据颜色的变化可以判断出点云的边缘，根据点云边缘和二维图像边缘的对齐程度来判断标定结果是否好，标注效果见下图：

   ![5ba7e5e2964f4d3b1116d03d6.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/5ba7e5e2964f4d3b1116d03d6_5ba7e5e.jpg)

2. 单击 **参数替换** 。

   ![0234a14b5968dc874b3c18e0f.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/0234a14b5968dc874b3c18e0f_0234a14.jpg)

   参数替换将用新的标定参数覆盖车端已经存在的参数。

3. 单击 **确认** 。

   ![e72cceccba7fb35556878e25f.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/e72cceccba7fb35556878e25f_e72ccec.jpg)

   采集之后显示绿色表示参数替换成功：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_342a1a1.png)


## 常见问题

### 1. 采集失败的原因分析以及解决方法
- 返回”no corners found“，可能的原因如下，可以逐条核查：

  - 标定板没有完全在图像视野内，需将标定板移到视野内，
  - 标定板在图像中所占像素太少，需将标定板向镜头的方向移动。

  - 光照太强，需更换标定板摆放的角度，或者进行避光处理。

- 返回"extract rectangle corners failed"，可能原因如下，可以逐条核查：

  - 通过点云图确认打在标定板上的线数是否大于等于4条，若不是，请将标定板向前移动，

  - 确认环境中是否有与标定板类似的物体，或者标定板周围是否有其他物体，若有，请移除。


