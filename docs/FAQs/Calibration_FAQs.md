
**-English-**
## How to Check  Sensor Output?


Use the `rostopic` command. For example, type the following command to check the
output of HDL-64ES3:

```bash
 rostopic echo /apollo/sensor/velodyne64/VelodyneScanUnified
```

If the topic data is displayed on the terminal, the LiDAR works normally.

**-Chinese-**
## 如何查看传感器是否有数据输出?  

使用 rostopic 命令。例如，查看 HDL-64ES3 的输出，可以在终端中输入: 
 
```bash
 rostopic echo /apollo/sensor/velodyne64/VelodyneScanUnified
```
 若该 topic 的数据会显示在终端上，则激光雷达工作正常。

**-English-**

### How to Check INS Status?

Using Novatel INS as an example, type the following command to check the INS status:  

```bash 
rostopic echo /apollo/sensor/gnss/ins_stat
```

Find the `pos_type` field:  If the value is 56, it has entered a good positioning status (RTK_FIXED) and can be used for calibration. If it is not 56, reliable calibration results cannot be obtained.

**-Chinese-**

### 如何查看车辆的定位状态?  

以使用 Novatel 组合惯导为例，在终端中输入: 

```bash
rostopic echo /apollo/sensor/gnss/ins_stat
```  

找到“pos_type”字段，若该字段的值为 56，则表示进入了良好的定位状态 (RTK_FIXED)，可以用于标定。若不为 56，则无法获得可靠的标定结果。

**-English-**

### How to Complete a Quality Inspection?

At present, you complete the quality verification manually with a visual inspection of the results. 

When the calibration is completed, the point cloud stitched during the calibration process is provided.  In the point cloud, details of the calibration field can be easily identified. Assess the calibration quality for clarity. Look at objects such as building facades, street lights, poles and road curbs.   If the point cloud is blurry and a ghosting effect can be found, the calibration is poor. If the calibration result is good, a sharp and clear stitched point cloud is shown.

Figure 1 shows the comparison between the stitched point clouds with good (a) and insufficient(b) calibration quality.

![](images/good_calib.png)
<p align="center">
(a)
</p>

![](images/poor_calib.png)
<p align="center">
(b)
</p>

<p align="center">
Figure 1. (a) a high quality calibration result (b) an insufficient one.
</p>

**-Chinese-**

### 如何进行质检?

目前进行质检方法主要通过人工来完成。标定完成后，页面会提供标定过程中拼接得到的点云。若标定结果良好，会得到锐利和清晰的拼接点云，可反映出标定场地的细节。通常质检的参照物有平整的建筑立面、路灯和电线杆以及路沿等。若标定质量较差，则会使拼接点云出现一些模糊、重影的效果。图1是两张不同标定质量的拼接点云对比。  

![](images/good_calib.png)

<p align="center">
(a)
</p>

![](images/poor_calib.png)

<p align="center">
(b)
</p>

<p align="center">
图1. (a) 高质量的标定结果 (b) 质量较差的标定结果。
</p>
