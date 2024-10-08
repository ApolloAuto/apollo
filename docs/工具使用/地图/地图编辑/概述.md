## 概述

地图编辑指在点云底图中标注出算法所需要的高精地图信息，例如车道、人行道、十字路口等。

## 操作步骤

### 步骤一：打开底图

1. 进入地图编辑界面，点击 **文件** > **打开底图**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_17f3a37.png)

   > 说明：地图编辑建议受用 chrome 浏览器，且可以多个浏览器访问。

2. 选择想要编辑的底图，并单击 **打开** 完成底图加载。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_aa42e14.png)

   底图加载完成后，按照下面的步骤进行底图绘制。

### 步骤二：编辑底图

绘制线、车道、路口、人行横道、减速等、红绿灯、停车位等元素的详细操作，参见地图编辑章节。

### 步骤三：保存底图

1. 在 **文件** 下拉框中选择 **保存** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_159783d.png)

2. 输入文件名称，并单击 **保存**，完成标注地图的保存操作。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4c9e4f3.png)

### 步骤四：发布底图

1. 在 **文件** 下拉框中选择 **发布** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_e244bbd.png)

2. 输入文件名称，并单击 **保存** ，完成标注地图发布操作。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9abda2c.png)

   > 说明：
   > - 保存：指的是用户可以对要修改的地图进行多次编辑；保存到`data/editor_map`路径下。
   > - 发布：指的是自动驾驶使用的地图，发布后保存到`data/map_data`路径下，可以通过发布地图为自动驾驶提供地图文件。

## 视图相关操作

### 背景放大/缩小

- Windows：鼠标滚轮，向前靠近（即放大），向后远离（即缩小）。
- Mac OS：将两根手指放在触控板上，然后将彼此分开的手指按入以放大视图，或者并拢移动手指以缩小视图。

### 移动视图

- Windows：右键拖动，
- Mac OS：触控板双指拖动。

### 旋转视图

- Windows：按住 ALT 键，左键横向拖动
- Mac OS：按住 option 键，触控板单指横向拖动。

### 旋转元素
先点击选中线、车道、路口、人行横道等元素，元素高亮显示，同时元素周围出现旋转按钮，点击旋转按钮，完成左/右或逆时针/顺时针旋转。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6c6b6a8.png)

### 移动元素

先点击选中线、车道、路口、人行横道等元素，元素高亮显示，再次按住鼠标左键拖动，移动元素位置。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5c40f94.png)

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_42418c5.png)

### 连接元素

按 Shift 键同时选中想要连接的多个元素，如线、车道等，点击相应的连接按钮，如将线连接生成车道，车道连接等。

### 多选元素

单击选中第一个元素，按住 Shift 键同时选中第二个元素。

> 说明：多选的元素必须类型一致，否则选不中。
