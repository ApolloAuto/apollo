# 调试工具
## 介绍
本工具为control模块的调试工具，主要对control模块的输入、输出以及部分中间变量进行显示，并保存相应的数据。
## Prerequisites
使用本工具需要先进入docker环境中
```
cd ~/apollo
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
```
并且，本工具需要运行在apollo6.0版本中，由于apollo6.0与apollo5.5有所差别，目前暂时缺少一些环境变量，需要自行安装

`安装tkinter`
```
sudo apt-get update
sudo apt-get install python3-tk
```
`安装xlsxwriter`
```
pip install xlsxwriter
```
**上述步骤完成后，可将更改保存到docker image中，否则以后每次启动都需要重新安装，修改docker image可参考[Apollo运行环境完善](https://wiki.baidu.com/pages/viewpage.action?pageld=1457082911)**
## 使用说明
```
python bazel-bin/modules/control/tools/control_info --bag [bag]
```
bag为需要分析的数据包

`eg:`
```
python bazel-bin/modules/control/tools/control_info --bag data/bag/0428/1-2021-04-28-11-40-00_s/20210428114000.record.00000
```
## 代码讲解
代码中主要包含三种方法和一个参数：
1. **__callback_XXX()** 用来解析数据包内不同topic的数据，将数据进行分类并存储。
2. **show_XXX()** 用来plot需要分析的数据，例如，使用show_longitudinal()来分析纵向控制相关的数据，但相关的变量比较多，很多变量并不是使用者关注的，所以设计了**plot_XXX()**来plot具体的变量，**使用者可按需自行修改，注意不要将两个不同topic的变量用于plot的x轴和y轴，例如`plot(self.controltime, self.canbus_brakefbk)`这样容易出现报错，因为controltime和canbus_brakefbk属于不同topic的数据，容易出现长度不一致的情况。可使用Print_len()打印变量长度信息。**
3. **add_XXX()_sheet** 用来将相关数据添加到Excel文件进行定量分析。**程序会判断excel文件是否存在，如果存在则不做任何操作，不会覆盖原有文件，如果文件内容有变动，删除原有文件重新运行即可。**
4. **show_planning_flag**为是否展示planning信息的参数。planning模块每100ms会发生一系列trajectory_point，将每次的trajectory_point保存到Excel并plot出来。
## 其他
本工具保留了按键采集的功能，点击画布，按下按键**q**可快捷关闭画布，其他功能使用者可按需自行修改。