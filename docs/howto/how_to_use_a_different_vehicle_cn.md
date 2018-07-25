# 如何选择使用不同的车辆

为了添加不同的车辆，你必须首先按照[Readme文档中的安装步骤](https://github.com/ApolloAuto/apollo/blob/master/README.md) 安装并成功启动Apollo。

为了在DreamView中支持一种新车辆，请遵循如下步骤：

1. 在`modules/calibration/data`目录下，为新车辆创建一个新目录

2. 在`modules/calibration/data`目录下已经有一个名为`mkz_example`的参考样例。参考该目录下的文件列表并包含所有必须的配置文件。如需要，务必根据新添车辆的参数更新所有配置文件。

3. 重启DreamView，你便能在`车辆`下拉菜单中看到新添加的车辆（名字和新创建的目录名字一样）。

4. 选择新配置的车辆。
