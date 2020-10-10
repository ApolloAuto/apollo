oprofile是运行在linux系统上对应用程序进行性能测试的工具。linux系统中已经自带了oprofile的相关工具，但是oprofile module在arm平台没有支持，所以我们需要手动安装oprofile。

oprofile对多线程支持良好，可以对函数调用次数及源码进行分析，所以非常适合在TX2上使用。

#### 下载
下载最新版本的oprofile
```bash
$ wget http://prdownloads.sourceforge.net/oprofile/oprofile-1.4.0.tar.gz
$ tar zxvf oprofile-1.4.0.tar.gz
$ cd oprofile-1.4.0
```
#### 安装oprofile
```bash
$ sudo apt-get install libpopt-dev libiberty-dev binutils-dev
$ ./configure
$ make -j4
$ sudo make install
```
#### 测试
安装好后，执行`operf`命令查看能否正常获取cpu信息。如果出现如下报错：

`unable to open /sys/devices/system/cpu/cpu0/online`

这是因为默认TX2只开启了4个CPU，有2个CPU处于未开启状态。
执行如下命令开启额外的两个CPU：
```bash
$ sudo nvpmodel -m 0
```

#### 使用方法
oprofile提供了多种命令，通常情况下我们使用比较多的是`operf`，`opreport`和`opannotate`。
以测试perception模块为例。
* 1.修改`script/apollo_bash.sh`脚本，文件第239行的`nohup`后面增加`operf`指令，如图：

![operf_command](images/TX2/operf_command.png)

* 2.使用脚本如`./script/perception.sh`启动perception模块

这样operf就会进行perception进程的运行数据统计。使用任意方法停止perception进程即可停止数据收集。

#### 测试数据查看
停止perception进程后，在当前文件夹下将生成文件夹oprofile_data。
使用指令`opreport`查看模块的总体占比：
```bash
$ opreport
```
结果示例为：

![opreport_](images/TX2/opreport_.png)

使用`opreport`查看函数占比：
```bash
$ opreport -l bazel-bin/modules/perception/perception 
```
因为输出信息很多，所以需要将上述结果保存为文本文件
```bash
$ opreport -l bazel-bin/modules/perception/perception > perception_op_funcs.md
```
结果示例为：

![opreport_file](images/TX2/opreport_file.png)

使用`opannotate`查看详细的源码数据统计：
```bash
$ opannotate -s bazel-bin/modules/perception/perception > perception_op_details.md
```

注意事项：同时只能运行一个operf进程，所以对perception进行数据统计时无法再用operf启动其他模块。

oprofile官方网站：
[http://oprofile.sourceforge.net/news/](http://oprofile.sourceforge.net/news/)

oprofile用户手册：
[http://oprofile.sourceforge.net/doc/index.html](http://oprofile.sourceforge.net/doc/index.html)

