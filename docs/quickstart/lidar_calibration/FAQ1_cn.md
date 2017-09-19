### 如何查看车辆的定位状态?  

以使用 Novatel 组合惯导为例，在终端中输入: 

```bash
rostopic echo /apollo/sensor/gnss/ins_stat
```  

找到“pos_type”字段，若该字段的值为 56，则表示进入了良好的定位状态 (RTK_FIXED)，可以用于标定。若不为 56，则无法获得可靠的标定结果。