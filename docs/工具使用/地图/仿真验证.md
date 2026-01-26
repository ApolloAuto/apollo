## 概述

使用仿真验证地图可用性。

## 操作步骤

1. 编辑并发布地图。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f09d0ea.png)

2. 编辑发布地图的名称，并单击保存。

3. 重启 Dreamview+。
   
   ```bash
   aem bootstrap  restart --plus
   ```
   
   > 说明：如果 Dreamview+ 与地图工具不在一个 docker 中，需将地图文件拷贝到 Dreamview+ 所在 docker 目录下。

4. 在 **Mode** 中选择 PNC 模式，在 **Operations** 中选择 **Sim_Control** 。

5. 在 **Environment Resources** > **HDMap** 资源中选择刚才发布的地图文件，加载成功。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_40002e0.png)

6. 单击 **Routing Editing** 。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a6a46c0.png)

7. 设置主车的起始点与终点，并单击 **Save Editing** 。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_11d3eab.png)

8. 返回主页，点击 **START** ，查看仿真结果。
   
    ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_3464bdf.png)
   
   车辆按照 Routing 和 Running 轨迹线跑起来，说明地图可用。
