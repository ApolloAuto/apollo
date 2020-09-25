# 软件 FAQ

## 除了 Ubuntu 之外，能用其它操作系统吗？

我们只对 Ubuntu 进行了测试，这意味着它是我们目前正式支持的唯一操作系统。欢迎开发
者尝试不同的操作系统，如果能够成功地使用它们，可以分享补丁与社区。

## 在链接到 localhost：8888（DreamView）时有问题

Dreamview 网页服务器由 Dreamview 节点提供。在访问 Dreamview 页面之前，您需要按
照[Apollo 软件安装指南](../quickstart/apollo_software_installation_guide.md) 编
译 DOCKER 容器内的系统（包括 Dreamview 节点）。一旦编译成功，Dreamview 节点将在
步骤 `bash scripts/bootstrap.sh` 中启动。

因此，如果你不能访问 Dreamview，请检查：

- 确保您的 Dreamview 进程正确运行。在最新版本中，如果 Dreamview 无法启动
  ，`bash scripts/bootstrap.sh`会报告`dreamview: ERROR (spawn error)`。或使用命
  令检查： `ps aux | grep dreamview`。
- 确保地址和端口不被防火墙拦截。
- 如果您是通过其它主机访问 Dreamview 页面，确保您使用的是
  `<apollo_host_ip>:8888` 而不是 `localhost:8888`。

## 我如何进行分步调试？

大多数 bug 可以通过日志记录找到（使用 AERROR, AINFO, ADEBUG）。如果需要一步一步
的调试，我们建议使用 GDB.
