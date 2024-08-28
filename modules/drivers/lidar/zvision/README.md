# 1 Introduction

Apollo-9.0.0 driver for zvision lidar.

# 2 Run
You can run a sample dag file in the modules/drivers/zvision/dag directory.
1. Run zvision EZ6 lidar
   ```
    mainboard -d /apollo/modules/drivers/lidar/zvision/dag/zvision_ez6.dag
   ```

# 3 Config

Change the zvision/drivers/BUILD file based on the lidar device type.

Such as: **ez6_sample_a**   or   **ez6_sample_b**

![image-20240723140119480](C:\Users\de'l'l'l\AppData\Roaming\Typora\typora-user-images\image-20240723140119480.png)

# 4 View

You can use cyber_visualizer tools show lidar point cloud

```
cyber_visualizer 
```

