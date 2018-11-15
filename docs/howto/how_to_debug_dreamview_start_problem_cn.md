## 如何调试Dreamview启动问题

### Dreamview的启动步骤

如果在`docker / scripts / dev`序列中启动Dreamview时遇到问题，请首先检查是否使用了如下所示的正确命令。

```bash
$ bash docker/scripts/dev_start.sh
$ bash docker/scripts/dev_into.sh
$ cd /apollo
$ bash apollo.sh build
$ bash scripts/dreamview.sh
```
### Dreamview启动失败

如果Dreamview无法启动，请使用下面的脚本检查Dreamview的启动日志并重新启动Dreamview。

```bash
# Start Dreamview in foreground to see any error message it prints out during startup
$ bash scripts/dreamview.sh start_fe

# check dreamview startup log
$ cat data/log/dreamview.out
terminate called after throwing an instance of 'CivetException'
  what():  null context when constructing CivetServer. Possible problem binding to port.

$ sudo apt-get install psmisc

# to check if dreamview is running from other terminal
$ sudo lsof -i :8888

# kill other running/pending dreamview
$ sudo fuser -k 8888/tcp

# restart dreamview again
$ bash scripts/dreamview.sh
```

### 用gdb调试

如果dreamview的启动日志中没有任何有效内容，您可以尝试使用gdb调试dreamview，请使用以下命令：

```
$ gdb --args /apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/dreamview/conf/dreamview.conf
# or
$ source scripts/apollo_base.sh;
$ start_gdb dreamview
```

一旦gdb启动，按下`r`和`enter`键运行，如果dreamview崩溃，然后用`bt`获得回溯。

如果您在gdb backtrace中看到错误“非法指令”以及与 **libpcl_sample_consensus.so.1.7** 相关的内容，那么您可能需要自己从源代码重建pcl lib并替换docker中的那个。

这通常发生在您尝试在CPU不支持FMA/FMA3指令的机器上运行Apollo/dreamview时，它会失败，因为docker image附带的预构建的pcl lib是使用FMA/ FMA3支持编译的。