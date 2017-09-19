### 如何查看传感器是否有数据输出?  

使用 rostopic 命令。例如，查看 HDL-64ES3 的输出，可以在终端中输入: 
 
```bash
 rostopic echo /apollo/sensor/velodyne64/VelodyneScanUnified
```
 若该 topic 的数据会显示在终端上，则激光雷达工作正常。
 
