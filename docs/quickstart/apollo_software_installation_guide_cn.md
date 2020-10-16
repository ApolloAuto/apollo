# Apollo软件安装指南

本文档介绍了在Ubuntu 18.04.5 LTS（Bionic Beaver）（建议用于Apollo 6.0的Ubuntu版本）上安装Apollo软件所需的步骤。

## 前期准备

在开始之前，请确保已按照[必备软件安装指南](../specs/prerequisite_software_installation_guide.md)中的说明完成了所有必备步骤。

同时也要确保Docker正在运行。键入`systemctl status docker`以检查Docker守护进程的运行状态，并根据需要键入`systemctl start docker`以启动 Docker。

## 下载Apollo源文件

运行以下命令克隆[Apollo的GitHub仓库](https://github.com/ApolloAuto/apollo.git)。

```bash
# 使用 SSH 的方式
git clone git@github.com:ApolloAuto/apollo.git

# 使用 HTTPS 的方式
git clone https://github.com/ApolloAuto/apollo.git

```

切换到`master`分支:

```bash
cd apollo
git checkout master
```

对于中国用户, 如果从GitHub克隆仓库有困难，可参考[国内环境下如何克隆Apollo仓库](../howto/how_to_clone_apollo_repo_from_china_cn.md)。

（可选）为方便起见，可以在Apollo的根目录运行以下命令来设置指向该目录的环境变量`APOLLO_ROOT_DIR`：

```bash
echo "export APOLLO_ROOT_DIR=$(pwd)" >> ~/.bashrc  && source ~/.bashrc
```

![tip](images/tip_icon.png) 在下面的章节中，我们会把Apollo的根目录称为`$APOLLO_ROOT_DIR`。

## 启动Docker容器

在`${APOLLO_ROOT_DIR}`目录, 输入：

```bash
bash docker/scripts/dev_start.sh
```

来启动Apollo的Docker开发容器。

如果成功，你将在屏幕下方看到以下信息：

```bash
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_dev_michael container, please run the following command:
[ OK ]   bash docker/scripts/dev_into.sh
[ OK ] Enjoy!
```

## 进入Apollo的Docker容器

运行以下命令以登录到新启动的容器：

```bash
bash docker/scripts/dev_into.sh
```

## 在容器内构建Apollo

在Docker容器的`/apollo`目录中, 输入:

```bash
./apollo.sh build
```

以构建整个Apollo工程。或者输入

```bash
./apollo.sh build_opt
```

来进行优化模式的构建。可以参考[Apollo构建和测试说明](../specs/apollo_build_and_test_explained.md)来全面了解Apollo的构建和测试。

## 启动并运行Apollo

请参考[如何启动并运行Apollo](../howto/how_to_launch_and_run_apollo.md)中的[运行Apollo](../howto/how_to_launch_and_run_apollo.md#run-apollo)部分。

## （可选）在Dreamview中支持新的车型

为了在Dreamview中支持新的车型，请按照以下步骤操作：

1. 在`modules/calibration/data`目录下为你的车型创建一个新文件夹。

2. 在`modules/calibration/data`文件夹中已经有一个叫作`mkz_example`的示例文件夹。请参考此结构，并以此结构包含所有必需的配置文件。如果需要的话，请记得使用自己的参数更新配置文件。

3. 重启Dreamview，你将能够在可选车型列表中看到你的新车型（名称与你创建的文件夹相同）。
