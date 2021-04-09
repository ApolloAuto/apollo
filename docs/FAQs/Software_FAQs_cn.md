# 软件FAQ

## 除了Ubuntu之外，其他操作系统还能使用吗？

我们只对Ubuntu进行了测试，这意味着它是我们目前正式支持的唯一操作系统。Ubuntu是一个理想的操作系统，因为ROS需要支持它。欢迎开发者尝试不同的操作系统，如果能够成功地使用它们，可以分享补丁与社区。

## 在链接到localhost：8888（DreamView）时有问题

Dreamview网页服务器由Dreamview节点提供（节点是ROS概念中的可执行文件）。在访问Dreamview页面之前，您需要跟着[指南](https://github.com/ApolloAuto/apollo/blob/master/README.md)编译DOCKER容器内的系统（包括Dreamview节点）。一旦编译成功，Dreamview节点将在步骤“bash scripts/bootstrap.sh”之后启动。

因此，如果你不能访问Dreamview，请检查：

* 确保您的Dreamview进程正确运行。在最新版本中，如果Dreamview无法启动，`bash scripts/bootstrap.sh`会报告`dreamview: ERROR (spawn error)`。对于早期版本，请使用命令检查：`supervisorctl status dreamview` 或者 `ps aux | grep dreamview`。如果Dreamview没有运行，请参阅[如何调试ValueVIEW启动问题](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_debug_dreamview_start_problem.md)。
* 确保地址和端口不被防火墙拦截。
* 如果您没有通过主机访问Dreamview页面，确保您使用的是<Apollo_host_ip>：8888而不是localhost：8888。

## 我如何进行分步调试？

大多数bug可以通过日志记录找到（使用AERROR, AINFO, ADEBUG）。如果需要一步一步的调试，我们建议使用gdb

## 我如何运行离线可视化感知？

参考How-to指南[这里](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_run_offline_perception_visualizer.md)。

## 在安装预构建的Apollo内核后，ubuntu 14.04登录循环问题。

这里有一种解决方案：
 * 重新启动Ubuntu系统，按下并按住“shift”按钮，然后进入GRUB菜单。
 * 选择一个通用的和可引导的选项（不是带有Apollo内核的引导加载程序）来启动。
 * 按“CTRL+ALT+F1”，按照[Link](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_hardware_system_installation_guide_v1.md)来安装[NVIDIALIUX-X86Y64-75.39]。
 * 重启并使用带有Apollo内核的默认引导加载程序启动计算机。


**更多的软件常见问题。**
