# 运行线下演示

如果你没有车辆及车载硬件， Apollo还提供了一个计算机模拟环境，可用于演示和代码调试。 

线下演示需要设置docker的release环境，请参照 [how_to_build_and_release](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)文档中的[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#docker)章节。

Apollo演示的安装步骤：

1. 运行如下命令启动docker的release环境:

    ```
    bash docker/scripts/release_start.sh
    ```

2. 运行如下命令进入docker的release环境:

    ```
    bash docker/scripts/release_into.sh
    ```

3. 运行如下命令回放位于`docs/demo_guide/demo.bag`的rosbag:

    ```
    rosbag play docs/demo_guide/demo.bag --loop
    # 或者
    bash docs/demo_guide/rosbag_helper.sh download # 下载rosbag
    rosbag play docs/demo_guide/demo_1.5.np.bag --loop
    ```

    选项 `--loop` 用于设置循环回放模式.

4. 打开Chrome浏览器，在地址栏输入**localhost:8887**即可访问Apollo HMI。 HMI主页如下图所示。
    ![](images/start_hmi.png)

5. 点击中间控制面板的 'Dreamview' 开关.
    ![](images/dreamview_enable.png)

6. 点击右上角的"Dreamview" 按钮打开Dreamview.
    ![](images/dreamview_launch.png)

7. Dreamview会以默认网址**localhost:8888**打开.
    ![](images/dv_trajectory.png)现在你能看到有一辆汽车在模拟器里移动!

恭喜你完成了Apollo的演示步骤！
