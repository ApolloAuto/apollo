# Apollo 内核的安装

## 在您开始前

Apollo 内核仅在实车的运行时环境中被依赖。如果您只是想在 Apollo 软件平台上开发/测
试您的软件，或者只是运行自动驾驶模拟软件（如 LGSVL 模拟器），则无需安装 Apollo
内核。

## 如何安装 Apollo 内核

1.  从[Apollo 内核的 Github 发布页](https://github.com/ApolloAuto/apollo-kernel/releases)上
    下载（最新的）Apollo 内核发布版软件包。

2.  下载成功后运行如下命令安装内核：

```bash
tar zxvf linux-4.4.32-apollo-1.5.5.tar.gz
cd install
sudo bash install_kernel.sh
```

3.  重启系统。

4.  重启后，编译 ESD CAN 驱动。按
    照[ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)
    的说明编译 ESD CAN 驱动。
