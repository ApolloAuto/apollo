# 运行线下演示

如果你没有车辆及车载硬件， Apollo 还提供了一个计算机模拟环境，可用于演示和代码调
试。

线下演示首先要 Fork 并且 Clone Apollo 在 GitHub 的代码，然后需要设置 docker 的
release 环境，请参照
[how_to_build_and_release](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)文
档中
的[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#docker)章
节。

Apollo 演示的安装步骤：

1. 运行如下命令启动 docker 的 release 环境:

   ```
   bash docker/scripts/dev_start.sh
   ```

2. 运行如下命令进入 docker 的 release 环境:

   ```
   bash docker/scripts/dev_into.sh
   ```

3. 在 Docker 中编译 Apollo:

   ```
   bash apollo.sh build
   ```

   `Note:` 如果没有 GPU，请使用下面的命令

   ```
   bash apollo.sh build_cpu
   ```

4. 启动 DreamView

   ```
   bash scripts/bootstrap.sh
   ```

5. 下载 demo record:

   ```
   cd docs/demo_guide/
   python record_helper.py demo_3.5.record
   ```

6. 运行如下命令回放 record:

   ```
   cyber_recorder play -f docs/demo_guide/demo_3.5.record --loop
   ```

   选项 `--loop` 用于设置循环回放模式.

7. 打开 Chrome 浏览器，在地址栏输入**localhost:8888**即可访问 Apollo Dreamview，
   如下图所示： ![](images/dv_trajectory.png) 现在你能看到有一辆汽车在模拟器里移
   动!

恭喜你完成了 Apollo 的演示步骤！
